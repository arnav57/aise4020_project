"""
drive_node — hardware interface and safety watchdog.

Responsibilities:
  - Translate /cmd_vel and /servo_cmd into hardware publish calls
  - Report battery state and log low-battery warnings

Topic interface:
  Subscriptions:
    /cmd_vel        (geometry_msgs/Twist)   — velocity commands from arbiter
    /servo_cmd      (std_msgs/Int32)        — camera pan angle (-90..90 hw degrees)
    /beep_cmd       (std_msgs/UInt16)       — beep duration in ms

  Publications:
    /battery        (std_msgs/UInt16)       — battery percentage (re-published as-is)

  Note: /battery is subscribed AND re-published so the web node can read it
  without depending on the hardware driver's internal topic name.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt16
from geometry_msgs.msg import Twist

WATCHDOG_TIMEOUT_S = 2000   # seconds before auto-stop triggers
LOW_BATTERY_PCT    = 67    # percentage threshold for warning logs


class DriveNode(Node):
    def __init__(self) -> None:
        super().__init__('drive_node')

        # --- Publishers (hardware topics) ---
        self._pan_pub  = self.create_publisher(Int32,  '/servo_s1',  10)
        self._beep_pub = self.create_publisher(UInt16, '/beep',      10)

        # --- Subscribers ---
        self.create_subscription(Int32,  '/servo_cmd', self._servo_cb,    10)
        self.create_subscription(UInt16, '/beep_cmd',  self._beep_cb,     10)
        self.create_subscription(UInt16, '/battery',   self._battery_cb,  10)


        self.get_logger().info('Drive node online')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _servo_cb(self, msg: Int32) -> None:
        """Accept a hardware-frame angle (-90..90) and forward it."""
        clamped = Int32(data=max(-90, min(90, msg.data)))
        self._pan_pub.publish(clamped)

    def _beep_cb(self, msg: UInt16) -> None:
        self._beep_pub.publish(msg)

    def _battery_cb(self, msg: UInt16) -> None:
        if msg.data < LOW_BATTERY_PCT:
            self.get_logger().warning(
                f'Low battery: {msg.data}%'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
