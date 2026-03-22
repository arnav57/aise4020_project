"""
arbiter_node — command priority gate and the sole /cmd_vel publisher.

This is the fix for the old codebase's biggest bug: multiple nodes publishing
to /cmd_vel and racing each other. Nothing publishes /cmd_vel except this node.

Priority order (highest wins):
  1. ESTOP   — a zero-velocity command from yolo_node (e.g. stop sign)
  2. MANUAL  — velocity commands from the web UI (operator override)
  3. AUTO    — velocity commands from pid_node (lane-keep autopilot)

An ESTOP latches for ESTOP_LATCH_S seconds, then clears automatically.
MANUAL commands time out after MANUAL_TIMEOUT_S of silence (operator walked away).

Topic interface:
  Subscriptions:
    /cmd_vel_pid     (geometry_msgs/Twist) — from pid_node
    /cmd_vel_manual  (geometry_msgs/Twist) — from web_node
    /cmd_vel_estop   (geometry_msgs/Twist) — from yolo_node

  Publications:
    /cmd_vel         (geometry_msgs/Twist) — to drive_node (hardware)
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

ESTOP_LATCH_S    = 3.0   # seconds the ESTOP holds after it's triggered
MANUAL_TIMEOUT_S = 1.0   # seconds of silence before manual mode releases


class ArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__('arbiter_node')
        self._auto_enabled = False

        # --- The one true /cmd_vel publisher ---
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Command sources ---
        self.create_subscription(Twist, '/cmd_vel_pid',    self._auto_cb,   10)
        self.create_subscription(Twist, '/cmd_vel_manual', self._manual_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_estop',  self._estop_cb,  10)
        self.create_subscription(Bool,  '/lane_keep/enable', self._auto_enable_cb, 10)

        # --- State ---
        self._auto_cmd:   Twist = Twist()
        self._manual_cmd: Twist = Twist()

        self._estop_until:  float = 0.0
        self._manual_until: float = 0.0

        # Publish at 20 Hz regardless of incoming rate
        self.create_timer(0.05, self._arbitrate)

        self.get_logger().info('Arbiter node online.')

    # ------------------------------------------------------------------
    # Subscription callbacks — just store the latest command
    # ------------------------------------------------------------------

    def _auto_cb(self, msg: Twist) -> None:
        self._auto_cmd = msg

    def _manual_cb(self, msg: Twist) -> None:
        self._manual_cmd  = msg
        self._manual_until = time.time() + MANUAL_TIMEOUT_S

    def _estop_cb(self, msg: Twist) -> None:
        self._estop_until = time.time() + ESTOP_LATCH_S
        self.get_logger().warning('ESTOP received — motors halted.')

    def _auto_enable_cb(self, msg: Bool) -> None:
        self._auto_enabled = msg.data

    # ------------------------------------------------------------------
    # Arbitration loop
    # ------------------------------------------------------------------

    def _arbitrate(self) -> None:
        now = time.time()

        if now < self._estop_until:
            self._cmd_pub.publish(Twist())   # priority 1: hard stop
            return

        if now < self._manual_until:
            self._cmd_pub.publish(self._manual_cmd)   # priority 2: operator
            return

        if self._auto_enabled:
            self._cmd_pub.publish(self._auto_cmd)   # priority 3: autopilot


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
