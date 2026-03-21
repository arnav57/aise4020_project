import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, UInt16
from geometry_msgs.msg import Twist
import time

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        # publishers for the car movement, and the camera LR servo, and the beep
        self.raw_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.raw_pan_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.beep_pub    = self.create_publisher(UInt16, '/beep', 10)

        # subscribers for things we should probably listen to directly from the car
        self.create_subscription(UInt16, '/battery', self._battery_cb, 10)
        self.create_subscription(Int32, '/servo_s1', self._pan_feedback_cb, 10)

        # storage for stuff we need
        self.current_battery = 0.0
        self.current_pan_hw = 0  # Raw hardware value (-90 to 90)
        self.last_cmd_time = self.get_clock().now()
        
        # Checks every 100ms if we've received a command recently
        #self.watchdog_timer = self.create_timer(0.1, self._watchdog_check)

        self.get_logger().info("Hardware Interface Node Online (Watchdog Active)")

    def _battery_cb(self, msg):
        # assume the topic sends a percetage
        self.current_battery = float(msg.data)
        if self.current_battery < 20:
            self.get_logger().warning(f"LOW BATTERY: {self.current_battery}V")

    def _pan_feedback_cb(self, msg):
        self.current_pan_hw = msg.data

    def _watchdog_check(self):
        """If no cmd sent in last 500ms, kill the motors."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            stop_msg = Twist()
            self.raw_vel_pub.publish(stop_msg)

    ### PUBLIC METHODS
    
    def set_velocity(self, linear_x, angular_z):
        """Sends velocity to motors and resets the safety watchdog."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.raw_vel_pub.publish(msg)
        self.last_cmd_time = self.get_clock().now()

    def set_pan(self, ui_angle):
        """Level-shifts the 0-180 UI signal to the -90 to 90 HW signal."""
        # Hardware Center (0) = UI Center (90)
        hw_angle = int(ui_angle) - 90
        
        # Clipping logic (Saturation)
        hw_angle = max(-90, min(90, hw_angle))
        
        self.raw_pan_pub.publish(Int32(data=hw_angle))

    def set_beep(self, beep):
        """ Sets the beep """
        self.beep_pub.publish(UInt16(data=beep))

def main(args=None):
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