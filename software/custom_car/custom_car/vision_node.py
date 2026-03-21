import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import cv2
import numpy as np
from cv_bridge import CvBridge

# local import
from .lane_detect import find_lanes

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # --- CV Bridge Setup ---
        self.bridge = CvBridge()
        
        # Initialize memory at center screen
        self.last_lane_x = 320 
        
        # --- Hardware Setup ---
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # --- Publishers ---
        self.rimage_pub = self.create_publisher(Image, '/camera/raw_frame', 10)
        self.image_pub = self.create_publisher(Image, '/camera/mod_frame', 10)
        self.error_pub = self.create_publisher(Int32, '/lane_error', 10)
        
        # ACTIVE GAZE: Camera Servo Publisher mapped to your exact topic
        self.cam_servo_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/scmd_vel', 10)

        # --- Parameters & Subscribers ---
        self.threshold_val = 60 # Defaulted to 60 for the wood-floor HSV setup
        self.screen_center = 320 
        self.create_subscription(Int32, '/vision/threshold', self._threshold_cb, 10)
        self.create_subscription(String, '/lane_cmd', self._lane_cmd_cb, 10)

        # --- Camera Gaze State ---
        self.cam_panning = False
        self.cam_pan_end_time = 0.0
        self.pending_lane_shift = 0

        # --- Processing Loop ---
        self.timer = self.create_timer(0.05, self._process_loop)

        # --- HHAAR cascades ot whatever ---
        self.stop_cascade   = cv2.CascadeClassifier()
        self.right_cascade  = cv2.CascadeClassifier()
        self.left_cascade   = cv2.CascadeClassifier()
        
        self.get_logger().info("Vision Node: Active Gaze initialized. Ready to swap lanes.")

    def _threshold_cb(self, msg):
        self.threshold_val = msg.data

    def _lane_cmd_cb(self, msg):
        shift_amount = 550  
        cam_shift_amount = 35
        cam_shift_time = 1
        
        # Start the 0.5s timer for the camera pan
        self.cam_pan_end_time = time.time() + cam_shift_time
        self.cam_panning = True
        
        # NOTE: Assuming Positive is Left and Negative is Right. 
        # Swap the 45 and -45 if your camera servo looks the wrong way!
        if msg.data == 'left':
            self.cam_servo_pub.publish(Int32(data=-1*cam_shift_amount))  # Glance Left
            self.pending_lane_shift -= shift_amount
            self.get_logger().info("GAZE: Forcing LEFT & Panning Camera -45 deg")
            
        elif msg.data == 'right':
            self.cam_servo_pub.publish(Int32(data=cam_shift_amount)) # Glance Right
            self.pending_lane_shift += shift_amount
            self.get_logger().info("GAZE: Forcing RIGHT & Panning Camera +45 deg")
            

    def _process_loop(self):
        # 1. Active Gaze Check: See if it's time to snap the camera back to center
        if self.cam_panning and time.time() > self.cam_pan_end_time:
            self.cam_panning = False
            self.cam_servo_pub.publish(Int32(data=0)) # Center is 0  
            self.last_lane_x += self.pending_lane_shift
            self.pending_lane_shift = 0

            # Clamp the memory so we don't accidentally set a target way off-screen
            self.last_lane_x = max(0, min(640, self.last_lane_x))
            self.get_logger().info("GAZE: Slowly moving back to CENTER (0 deg)")

        success, frame = self.cap.read()
        if not success:
            return

        try:
            rimg_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.rimage_pub.publish(rimg_msg)
        except:
            pass # do nothing if we drop a frame

        # 2. Call find_lanes with the proximity memory
        combined_img, detected_lanes = find_lanes(
            frame, 
            self.threshold_val, 
            self.last_lane_x, 
            expected_lanes=1
        )

        # 3. Determine target_x with Delta Filtering
        if len(detected_lanes) > 0:
            new_target = detected_lanes[0]
            
            # DYNAMIC FILTER: If the camera is actively panning, the image is moving fast.
            # We widen the allowable jump from 120px to 300px so it doesn't reject the new lane.
            flicker_limit = 100 # if self.cam_panning else 275
            
            if abs(new_target - self.last_lane_x) < flicker_limit:
                target_x = new_target
                self.last_lane_x = target_x # Update memory
            else:
                target_x = self.last_lane_x
                # self.get_logger().warn("Flicker detected! Filtering outlier.")
        else:
            # Memory Mode: If line is lost, hold the last known X
            target_x = self.last_lane_x

        # 4. Publish Error
        error = int(target_x - self.screen_center)
        self.error_pub.publish(Int32(data=error))

        # 5. Dashboard Output (Resized for the segmented view)
        if combined_img is not None and combined_img.size > 0:
            small_combined = cv2.resize(combined_img, (640, 160))
            try:
                img_msg = self.bridge.cv2_to_imgmsg(small_combined, encoding='bgr8')
                self.image_pub.publish(img_msg)
            except Exception as e:
                pass # Fail silently if the bridge drops a frame

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()