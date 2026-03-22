"""
lidar_node — LiDAR scan processor.

Responsibilities:
  - Read raw /scan data from the MS200 (or compatible) LiDAR
  - Extract the minimum distance in a configurable forward cone
  - Publish a clean /obstacle_distance topic for other nodes to consume

Topic interface:
  Subscriptions:
    /scan                (sensor_msgs/LaserScan)  — raw LiDAR data

  Publications:
    /obstacle_distance   (std_msgs/Float32)       — closest obstacle ahead (metres)
                                                    999.0 means no obstacle detected

Parameters:
    cone_half_angle_deg  (int, default 10)  — half-width of the forward cone in degrees
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

NO_OBSTACLE = 999.0   # Sentinel value: nothing detected in range


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_node')

        self.declare_parameter('cone_half_angle_deg', 10)
        self.add_on_set_parameters_callback(self._param_cb)

        self._dist_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.get_logger().info('Lidar node online.')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan) -> None:
        distance = self._extract_forward_distance(msg)
        self._dist_pub.publish(Float32(data=distance))

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------

    def _extract_forward_distance(self, msg: LaserScan) -> float:
        """
        Find the minimum range in the forward-facing cone.

        The MS200 places 0 degrees at the middle index of the ranges array.
        We grab ±cone_half_angle_deg around that index.
        """
        total = len(msg.ranges)
        if total == 0:
            return NO_OBSTACLE

        half = self.get_parameter('cone_half_angle_deg').value
        mid  = total // 2
        cone = msg.ranges[max(0, mid - half): mid + half + 1]

        valid = [r for r in cone if 0.0 < r < float('inf')]
        return float(min(valid)) if valid else NO_OBSTACLE

    # ------------------------------------------------------------------
    # Parameter callback
    # ------------------------------------------------------------------

    def _param_cb(self, params: list) -> SetParametersResult:
        for p in params:
            self.get_logger().info(f'Parameter updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
