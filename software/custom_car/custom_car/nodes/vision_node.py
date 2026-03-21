"""
vision_node — camera capture and lane detection pipeline.

Responsibilities:
  - Capture frames from the USB camera
  - Run lane detection via lib/lane_detect (stateless functions)
  - Publish the lane error signal for pid_node
  - Publish raw and debug frames for the web dashboard
  - Accept lane shift commands (left/right) with active-gaze camera pan

Topic interface:
  Subscriptions:
    /vision/threshold   (std_msgs/Int32)    — live threshold tuning from UI
    /lane_cmd           (std_msgs/String)   — 'left' or 'right' lane shift

  Publications:
    /camera/raw_frame   (sensor_msgs/Image) — raw BGR frame
    /camera/debug_frame (sensor_msgs/Image) — annotated ROI strip for dashboard
    /lane_error         (std_msgs/Int32)    — signed pixel error from screen centre
    /servo_cmd          (std_msgs/Int32)    — camera pan command (hw degrees, -90..90)

Parameters:
    threshold           (int,   default 80)   — brightness cutoff for tape detection
    screen_center       (int,   default 320)  — X pixel treated as zero-error
    cam_shift_deg       (int,   default 35)   — how far to pan during a lane shift
    cam_shift_sec       (float, default 1.0)  — how long to hold the pan before re-centering
    lane_shift_px       (int,   default 550)  — how far to move last_lane_x on a shift command
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

from custom_car.lib.lane_detect import LaneConfig, build_debug_overlay, detect_lane

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
LOOP_HZ       = 20   # target processing rate


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__('vision_node')

        # --- Parameters ---
        self.declare_parameter('threshold',      80)
        self.declare_parameter('screen_center',  320)
        self.declare_parameter('cam_shift_deg',  35)
        self.declare_parameter('cam_shift_sec',  1.0)
        self.declare_parameter('lane_shift_px',  550)
        self.add_on_set_parameters_callback(self._param_cb)

        # --- CV bridge + camera ---
        self._bridge = CvBridge()
        self._cap    = self._open_camera()

        # --- Publishers ---
        self._raw_pub   = self.create_publisher(Image,  '/camera/raw_frame',   10)
        self._debug_pub = self.create_publisher(Image,  '/camera/debug_frame', 10)
        self._error_pub = self.create_publisher(Int32,  '/lane_error',         10)
        self._servo_pub = self.create_publisher(Int32,  '/servo_cmd',          10)

        # --- Subscriptions ---
        self.create_subscription(Int32,  '/vision/threshold', self._threshold_cb, 10)
        self.create_subscription(String, '/lane_cmd',         self._lane_cmd_cb,  10)

        # --- Lane tracking state ---
        self._last_lane_x: int   = CAMERA_WIDTH // 2
        self._panning:     bool  = False
        self._pan_end:     float = 0.0
        self._pending_shift_px: int = 0

        # --- Processing loop ---
        self.create_timer(1.0 / LOOP_HZ, self._process_loop)

        self.get_logger().info('Vision node online.')

    # ------------------------------------------------------------------
    # Camera setup
    # ------------------------------------------------------------------

    def _open_camera(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        if not cap.isOpened():
            self.get_logger().error('Failed to open camera device 0.')
        return cap

    # ------------------------------------------------------------------
    # Main processing loop
    # ------------------------------------------------------------------

    def _process_loop(self) -> None:
        self._check_pan_timeout()

        success, frame = self._cap.read()
        if not success:
            self.get_logger().warning('Camera read failed — skipping frame.')
            return

        self._publish_raw(frame)

        config = self._build_lane_config()
        result = detect_lane(frame, config, self._last_lane_x)

        if result.found:
            self._last_lane_x = result.lane_x

        error = self._last_lane_x - self.get_parameter('screen_center').value
        self._error_pub.publish(Int32(data=int(error)))

        self._publish_debug(result.debug_frame)

    # ------------------------------------------------------------------
    # Active gaze / lane shift
    # ------------------------------------------------------------------

    def _check_pan_timeout(self) -> None:
        """Re-centre the camera after the pan hold period expires."""
        if self._panning and time.time() > self._pan_end:
            self._panning = False
            self._servo_pub.publish(Int32(data=0))
            self._last_lane_x += self._pending_shift_px
            self._last_lane_x  = max(0, min(CAMERA_WIDTH, self._last_lane_x))
            self._pending_shift_px = 0
            self.get_logger().info('Gaze: camera re-centred.')

    def _lane_cmd_cb(self, msg: String) -> None:
        direction   = msg.data.lower()
        shift_deg   = self.get_parameter('cam_shift_deg').value
        shift_sec   = self.get_parameter('cam_shift_sec').value
        shift_px    = self.get_parameter('lane_shift_px').value

        if direction == 'left':
            pan_angle           = -shift_deg
            self._pending_shift_px -= shift_px
        elif direction == 'right':
            pan_angle           = shift_deg
            self._pending_shift_px += shift_px
        else:
            self.get_logger().warning(f'Unknown lane_cmd: "{msg.data}"')
            return

        self._servo_pub.publish(Int32(data=pan_angle))
        self._pan_end  = time.time() + shift_sec
        self._panning  = True
        self.get_logger().info(f'Gaze: panning {direction} ({pan_angle} deg).')

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_raw(self, frame) -> None:
        try:
            self._raw_pub.publish(
                self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )
        except Exception as e:
            self.get_logger().warning(f'Raw frame publish failed: {e}')

    def _publish_debug(self, debug_frame) -> None:
        if debug_frame is None:
            return
        overlay = build_debug_overlay(debug_frame, self._last_lane_x, 160)
        try:
            self._debug_pub.publish(
                self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            )
        except Exception as e:
            self.get_logger().warning(f'Debug frame publish failed: {e}')

    # ------------------------------------------------------------------
    # Config helpers
    # ------------------------------------------------------------------

    def _build_lane_config(self) -> LaneConfig:
        return LaneConfig(
            threshold=self.get_parameter('threshold').value,
        )

    def _threshold_cb(self, msg: Int32) -> None:
        self.set_parameters([
            rclpy.parameter.Parameter('threshold', value=msg.data)
        ])

    def _param_cb(self, params) -> SetParametersResult:
        for p in params:
            self.get_logger().info(f'Parameter updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
