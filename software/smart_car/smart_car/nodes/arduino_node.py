import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
import json

from smart_car.lib.lane_detect import LaneConfig, build_debug_overlay, detect_lane
from smart_car.lib.arduino import ArduinoInterface

class ArduinoNode(Node):

	def __init__(self) -> None:
		super().__init__('arduino_node')

		self._blinker_timer = None

		# --- Arduino Communication ---
		self._arduino = ArduinoInterface()
		if self._arduino.connected:
			self.get_logger().info("Arduino Connected on interface /ttyUSB1")
		else:
			self.get_logger().warning("Arduino failed to connect on interface /ttyUSB1")

		# --- Publishers ---

		# --- Subscribers ---
		self.create_subscription(String, '/lane_cmd', self._lane_cmd_cb,  10)

	def _lane_cmd_cb(self, msg : String) -> None:
		direction = msg.data.lower()

		# reset any pre-existing blinker timer
		if self._blinker_timer is not None:
			self._blinker_timer.cancel()
			self._blinker_timer = None

		if direction == 'left':
			self.get_logger().info(f"sending cmd: 'LEFT_ON' to arduino")
			resp = self._arduino.send("LEFT_ON")
			self.get_logger().info(f"arduino responded with: {resp}")
		elif direction == 'right':
			self.get_logger().info(f"sending cmd: 'RIGHT_ON' to arduino")
			resp = self._arduino.send("RIGHT_ON")
			self.get_logger().info(f"arduino responded with: {resp}")

		# schedule timer to auto send the OFF cmd
		self._blinker_timer = self.create_timer(2.0, self._blinker_off)

	def _blinker_off(self) -> None:
		self._arduino.send('LEFT_OFF')
		self._arduino.send('RIGHT_OFF')
		self.get_logger().info("sent lights off cmd to arduino")

		# reset the timer to none
		if self._blinker_timer is not None:
			self._blinker_timer.cancel()
			self._blinker_timer = None

	def destroy_node(self) -> None:
		self._arduino.close()
		self._cap.release()
		super().destroy_node()

def main(args=None) -> None:
	rclpy.init(args=args)
	node = ArduinoNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()