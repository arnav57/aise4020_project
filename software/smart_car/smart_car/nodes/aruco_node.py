import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

MARKER_MAP = {
    0: "STOP",
    1: "SPEED_30",
    2: "GIVE_WAY",
}

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # Tuned for small markers at close range
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 10

        # Handle OpenCV 4.7+ and older versions
        try:
            self.detector = aruco.ArucoDetector(
                self.dictionary, self.parameters)
            self.use_new_api = True
        except AttributeError:
            self.use_new_api = False
            self.get_logger().warn(
                'OpenCV older than 4.7 detected, using legacy API')

        self.sub = self.create_subscription(
            Image,
            "/camera/raw_frame",
            self.image_callback,
            10
        )

        self.pub_id = self.create_publisher(
            Int32, '/infrastructure/marker_id', 10)
        self.pub_type = self.create_publisher(
            String, '/infrastructure/marker_type', 10)

        self.get_logger().info(
            f'ArUco node online)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = aruco.detectMarkers(
                frame, self.dictionary, parameters=self.parameters)

        if ids is None:
            return

        for marker_id in ids.flatten():
            marker_id = int(marker_id)
            instruction = MARKER_MAP.get(marker_id, "UNKNOWN")
            self.handle_instruction(instruction, marker_id)

    def handle_instruction(self, instruction, marker_id):
        # Publish the raw ID always
        id_msg = Int32()
        id_msg.data = marker_id
        self.pub_id.publish(id_msg)

        # Publish the instruction string
        type_msg = String()
        type_msg.data = instruction
        self.pub_type.publish(type_msg)

        # CHANGE THE CODE INSIDE EACH BLOCK freely, these are your 3 customizable options

        if instruction == "STOP":
            # Option 1 — runs when marker ID 0 is detected
            self.get_logger().info('Instruction: STOP')

        elif instruction == "SPEED_30":
            # Option 2 — runs when marker ID 1 is detected
            self.get_logger().info('Instruction: SPEED_30')

        elif instruction == "GIVE_WAY":
            # Option 3 — runs when marker ID 2 is detected
            self.get_logger().info('Instruction: GIVE_WAY')

        else:
            self.get_logger().warn(
                f'Marker ID {marker_id} has no defined instruction')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()