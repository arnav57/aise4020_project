import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import time

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        
        self.get_logger().info("Initializing AI Brain. Downloading YOLOv8 weights if missing...")
        
        # Load the ultra-fast Nano model
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("YOLOv8 Loaded and Ready.")
        
        # --- Subscriptions ---
        # Image queue size 1 to drop old frames if AI falls behind
        self.create_subscription(Image, '/camera/raw_frame', self._image_cb, 1) 
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        
        # --- Publishers ---
        self.lane_cmd_pub = self.create_publisher(String, '/lane_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/scmd_vel', 10) # For braking
        
        # --- Variables ---
        self.action_cooldown = 0.0
        self.frame_skip = 5
        self.frame_count = 0
        self.front_distance = 999.0 # Default to infinity so we don't accidentally trigger

    def _scan_cb(self, msg):
        # Find the exact middle of the array (which is 0 degrees / straight ahead on your MS200)
        total_rays = len(msg.ranges)
        if total_rays == 0:
            return
            
        mid_idx = total_rays // 2
        
        # Grab a 20-degree cone dead ahead (10 degrees left, 10 degrees right)
        # Because angle_increment is ~1 degree, 10 indices = 10 degrees.
        front_ranges = msg.ranges[mid_idx - 10 : mid_idx + 10]
        
        # Filter out 0.0 values (which mean the laser missed or errored)
        valid_ranges = [r for r in front_ranges if r > 0.0 and r < float('inf')]
        
        if valid_ranges:
            # Get the distance to the closest object in that forward cone (in meters)
            self.front_distance = min(valid_ranges)
        else:
            self.front_distance = 999.0

    def _image_cb(self, msg):
        current_time = time.time()
        
        # 1. Cooldown & Frame Skip check
        if current_time < self.action_cooldown:
            return
            
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        # 2. Convert ROS Image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Bridge Error: {e}")
            return

        # 3. RUN INFERENCE!
        # COCO Classes: 11 (Stop Sign), 39 (Bottle), 67 (Cell Phone)
        results = self.model.predict(frame, classes=[11, 39, 67], verbose=False)
        
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            
            # THE SENSOR FUSION THRESHOLD
            # How close should the object be before we act? (e.g., 0.60 meters)
            trigger_distance = 0.5 
            
            if confidence > 0.30:
                self.get_logger().info(f"AI Sees Object {class_id}. LIDAR Dist: {self.front_distance:.2f}m")
                
                if self.front_distance < trigger_distance:
                    
                    if class_id == 11:
                        self.get_logger().info(f"?? STOP SIGN REACHED at {self.front_distance:.2f}m! BRAKING!")
                        
                        # Slam on the brakes! Publish a 0 speed command.
                        stop_cmd = Twist()
                        stop_cmd.linear.x = 0.0
                        stop_cmd.angular.z = 0.0
                        self.cmd_vel_pub.publish(stop_cmd)
                        
                        self.action_cooldown = current_time + 3.0
                        break
                        
                    elif class_id == 39:
                        self.get_logger().info(f"?? BOTTLE REACHED at {self.front_distance:.2f}m! SWITCHING LEFT!")
                        cmd = String()
                        cmd.data = 'left'
                        self.lane_cmd_pub.publish(cmd)
                        self.action_cooldown = current_time + 3.0
                        break
                        
                    elif class_id == 67:
                        self.get_logger().info(f"?? PHONE REACHED at {self.front_distance:.2f}m! SWITCHING RIGHT!")
                        cmd = String()
                        cmd.data = 'right'
                        self.lane_cmd_pub.publish(cmd)
                        self.action_cooldown = current_time + 3.0
                        break

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()