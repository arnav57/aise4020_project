"""
pid_node — lane-keeping PID controller with camera feedforward.

Responsibilities:
  - Subscribe to /lane_error and compute a steering command
  - Add a feedforward term based on the camera's current pan angle
    (if the camera is looking right, pre-steer right before the PID catches up)
  - Publish desired velocity to /cmd_vel_pid (consumed by arbiter_node)
  - Expose all gains as live ROS2 parameters for runtime tuning

Topic interface:
  Subscriptions:
    /lane_error         (std_msgs/Int32)  — signed pixel error from vision_node
    /lane_keep/enable   (std_msgs/Bool)   — enable / disable autopilot
    /servo_cmd          (std_msgs/Int32)  — current camera pan angle (hw degrees)

  Publications:
    /cmd_vel_pid        (geometry_msgs/Twist) — PID steering output → arbiter

Parameters:
    kp      (double, default 0.005)
    ki      (double, default 0.000)
    kd      (double, default 0.050)
    k_cam   (double, default 0.0075)  — feedforward gain on camera angle
    speed   (double, default 0.15)    — forward speed when lane-keeping (m/s)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist

from custom_car.lib.pid import PIDConfig, PIDController


class PIDNode(Node):
    def __init__(self) -> None:
        super().__init__('pid_node')

        # --- Parameters ---
        self.declare_parameter('kp',    0.005)
        self.declare_parameter('ki',    0.000)
        self.declare_parameter('kd',    0.050)
        self.declare_parameter('k_cam', 0.0075)
        self.declare_parameter('speed', 0.15)
        self.add_on_set_parameters_callback(self._param_cb)

        # --- PID controller (from lib) ---
        self._pid = PIDController(self._read_pid_config())

        # --- State ---
        self._enabled      = False
        self._camera_angle = 0
        self._last_time    = self.get_clock().now()

        # --- Pub / Sub ---
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)
        self.create_subscription(Int32, '/lane_error',       self._error_cb,  10)
        self.create_subscription(Bool,  '/lane_keep/enable', self._enable_cb, 10)
        self.create_subscription(Int32, '/servo_cmd',        self._servo_cb,  10)

        self.get_logger().info('PID node online.')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _enable_cb(self, msg: Bool) -> None:
        self._enabled = msg.data
        if not self._enabled:
            self._pid.reset()
            self._cmd_pub.publish(Twist())
        self.get_logger().info(
            f'Lane-keep: {"ENABLED" if self._enabled else "DISABLED"}'
        )

    def _servo_cb(self, msg: Int32) -> None:
        self._camera_angle = msg.data

    def _error_cb(self, msg: Int32) -> None:
        if not self._enabled:
            return

        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if dt <= 0:
            return

        error    = float(msg.data)
        steering = self._pid.compute(error, dt)

        # Feedforward: camera already looking in a direction → pre-steer that way
        k_cam    = self.get_parameter('k_cam').value
        steering += self._camera_angle * k_cam

        # Final clamp (PID output already clamped, but feedforward can push it over)
        steering = max(-1.2, min(1.2, steering))

        twist = Twist()
        twist.linear.x  = self.get_parameter('speed').value
        twist.angular.z = -steering   # negative: positive error = right = steer left
        self._cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _param_cb(self, params) -> SetParametersResult:
        self._pid.update_config(self._read_pid_config())
        for p in params:
            self.get_logger().info(f'PID parameter updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def _read_pid_config(self) -> PIDConfig:
        return PIDConfig(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
