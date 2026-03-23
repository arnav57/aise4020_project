import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import time

MARKER_MAP = {
    0: "STOP",
    1: "SPEED_60",
    2: "SPEED_100",
    3: "TURN_LEFT",
    4: "TURN_RIGHT",
}

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # --- states ---
        self._prev_instruction = None  # str
        self._prev_id          = None  # int
        self._last_seen_time   = 0.0   # time we last saw a sign
        self._memory_timeout   = 1.0   # timeout till memory of last sign is cleared

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

        # need a pub for turn left/right
        self.turn_pub = self.create_publisher(String, '/lane_cmd', 10)

        self.get_logger().info(f'Aruco node online')

    # --- Sub Callbacks ---

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        #self.get_logger().info(f"IDs: {ids}")

        if ids is None:
            # log it if we transition to None
            if self._prev_id is not None and (time.time() - self._last_seen_time > self._memory_timeout):
                self.get_logger().info("Instruction Memory Cleared")
                # clear module state, so we can detect next sign
                self._prev_id           = None
                self._prev_instruction  = None
            
            return
        self._last_seen_time = time.time()

        # only consider the FIRST marker we find here, by deisgn we dont care about multiple markers
        marker_id = int(ids[0][0])

        # check if the id is within our mapping, then we handle the instruction, easy to move the constrain here.
        if marker_id in MARKER_MAP:
            instruction = MARKER_MAP.get(marker_id, "UNKNOWN")
            self.handle_instruction(instruction, marker_id)
        else:
            pass # we cba about the other ones

    # --- Publishing and Handling Instrucitns ---

    def _publish(self, instruction, marker_id):
        # only publish the msg if the marker id's change
        # since instruction is derived from ID it doesnt make sense to add the instruction check

        msg_valid_int = False

        if marker_id != self._prev_id:
            # assmelbe data
            id_msg          = Int32()
            instr_msg       = String()
            id_msg.data     = marker_id
            instr_msg.data  = instruction

            self.pub_id.publish(id_msg)
            self.pub_type.publish(instr_msg)

            msg_valid_int = True

        # update states at the end + return the validiity
        self._prev_id          = marker_id
        self._prev_instruction = instruction

        return msg_valid_int

    def handle_instruction(self, instruction, marker_id):

        # condtionally publish the data (this is our edge detector)
        # tells us if the message is valid
        valid_publish = self._publish(instruction, marker_id)

        # only log if the message is valid (new value on id or instruction)

        if valid_publish:
            ## NOTE:
            # STOP IS HANDLED INSIDE arbiter_node.py
            # SPEED COMMANDS ARE HANDLED IN pid_node.py
            # TURN COMMANDS ARE HANDLED HERE!! we just publish to the /lane_cmd topic
            # we just log the instruction here to locally 'handle' it

            self.get_logger().info(f"Instruction: '{instruction}'")

            if instruction == "TURN_LEFT":
                s = String()
                s.data = "left"
                self.turn_pub.publish(s)
            elif instruction == "TURN_RIGHT":
                s = String()
                s.data = "right"
                self.turn_pub.publish(s)



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