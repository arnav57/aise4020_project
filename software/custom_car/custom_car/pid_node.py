import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        # 1. Declare parameters (Added 'k_cam' for the feedforward gain)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('ki', 0.000)
        self.declare_parameter('kd', 0.050)
        self.declare_parameter('k_cam', 0.0075) # Feedforward multiplier
        self.declare_parameter('speed', 0.15)
        
        # 2. Register the parameter callback
        self.add_on_set_parameters_callback(self._parm_cb)
        
        # Subscriptions
        self.create_subscription(Int32, '/lane_error', self._error_cb, 10)
        self.create_subscription(Bool, '/lane_keep/enable', self._enable_cb, 10)
        
        # NEW: Listen to the camera's physical angle
        self.create_subscription(Int32, '/servo_s1', self._camera_angle_cb, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.enabled = False
        self.prev_error = 0
        self.integral = 0 
        self.camera_angle = 0 # Stores the current pan angle

        self.get_logger().info("PID Node: PID + Feedforward (Active Gaze) initialized.")

    def _camera_angle_cb(self, msg):
        # Keep track of where the camera is looking (-45 to 45 degrees)
        self.camera_angle = msg.data

    def _enable_cb(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.cmd_pub.publish(Twist())
        self.get_logger().info(f"Auto-Steer: {'ENABLED' if self.enabled else 'DISABLED'}")

    def _error_cb(self, msg):
        if not self.enabled:
            return

        # Fetch LIVE gains and speed
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        k_cam = self.get_parameter('k_cam').get_parameter_value().double_value
        speed = self.get_parameter('speed').get_parameter_value().double_value

        error = float(msg.data)
        
        # --- 1. Standard PID Controller Math (Corrects small pixel errors) ---
        p_term = kp * error
        
        self.integral += error
        i_term = ki * self.integral
        
        d_term = kd * (error - self.prev_error)
        
        pid_steering = -(p_term + i_term + d_term)
        
        # --- 2. Feedforward Math (Transforms camera angle into steering) ---
        # If camera looks right (+45), it adds positive steering (left turn).
        ff_steering = self.camera_angle * k_cam
        
        # --- 3. Sensor Fusion ---
        total_steering = pid_steering + ff_steering
        
        # Saturation (Clamping to prevent servo binding)
        total_steering = max(-1.2, min(1.2, total_steering))

        twist = Twist()
        twist.linear.x = speed 
        twist.angular.z = total_steering
        self.cmd_pub.publish(twist)
        
        self.prev_error = error

    def _parm_cb(self, params):
        for param in params:
            if param.name == 'k_cam':
                self.get_logger().info(f"!!! GAIN CHANGE: K_cam (Feedforward) is now {param.value}")
            # (You can add the other print statements here if you want)
        
        return SetParametersResult(successful=True)

def main(args=None):
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