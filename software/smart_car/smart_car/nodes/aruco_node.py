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
    3: "SPEED_100"
}

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # --- states ---
        self._prev_instruction = None  # str
        self._prev_id          = None  # int

        # --- aruco thingy setup ---

        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 10

        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        # -- Subscribers --
        self.sub = self.create_subscription(Image, "/camera/raw_frame", self.image_callback, 10)

        # -- Pubs -- 
        # one for id, type, add another for valid, we implement an edge detector here
        # so we only have to latch in the id and type once in the other nodes
        self.pub_id = self.create_publisher(Int32, '/vision/marker_id', 10)
        self.pub_type = self.create_publisher(String, '/vision/marker_type', 10)


        self.get_logger().info(f'Aruco node online')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None:
            return

        for marker_id in ids.flatten():
            marker_id = int(marker_id)
            instruction = MARKER_MAP.get(marker_id, "UNKNOWN")
            self.handle_instruction(instruction, marker_id)

    def _publish(self, instruction, marker_id):
        # only publish the msg if the instruction or marker id's change
        msg_valid_int = False

        # constrain marker id to be inside 0,1,2,3
        if marker_id not in range(0,4):
            return False

        if marker_id != self._prev_id:
            id_msg = Int32()
            id_msg.data = marker_id
            self.pub_id.publish(id_msg)
            msg_valid_int = True

        if instruction != self._prev_instruction:
            instr_msg = String()
            instr_msg.data = instruction
            self.pub_type.publish(instr_msg)
            msg_valid_int = True

        # update states at the end
        self._prev_id          = marker_id
        self._prev_instruction = instruction

        return msg_valid_int




    def handle_instruction(self, instruction, marker_id):

        # condtionally publish the data (this is our edge detector)
        # tells us if the message is valid
        valid_publish = self._publish(instruction, marker_id)

        # only log if the message is valid (new value on id or instruction)

        if valid_publish:
            if instruction == "STOP":
                # Option 1 — runs when marker ID 0 is detected
                self.get_logger().info('Instruction: STOP')

            elif instruction == "SPEED_30":
                # Option 2 — runs when marker ID 1 is detected
                self.get_logger().info('Instruction: SPEED_30')

            elif instruction == "GIVE_WAY":
                # Option 3 — runs when marker ID 2 is detected
                self.get_logger().info('Instruction: GIVE_WAY')

            elif instruction == "SPEED_100":
                self.get_logger().info('Instruction: SPEED_100')


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