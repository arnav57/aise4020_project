import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
import json

from smart_car.lib.lane_detect import LaneConfig, build_debug_overlay, detect_lane
from smart_car.lib.arduino import ArduinoInterface

class ArduinoNode(Node):

	def __init__(self) -> None:
		super().__init__('arduino_node')

		#  --- state vars (makes code cleaner) ---
		self._blinker_timer  = None
		self._brake_timer    = self.create_timer(0.25, self._apply_brake_lights)
		self._left_light_on  = False
		self._right_light_on = False
		self._brake_light_on = True
		self._brake_on       = False # must be false cuz we implement a software edge detector here (need brake lights to go on on reset)

		# --- Arduino Communication ---
		self._arduino = ArduinoInterface()
		if self._arduino.connected:
			self.get_logger().info("Arduino Connected on interface /ttyUSB1")
		else:
			self.get_logger().warning("Arduino failed to connect on interface /ttyUSB1")

		# --- Publishers ---
		## none for now?

		# --- Subscribers ---
		self.create_subscription(String, '/lane_cmd', self._lane_cmd_cb,  10)
		self.create_subscription(Twist,  '/cmd_vel', self._cmd_vel_cb, 10)


	# --- Better Send Serial Message (has Logs) ---

	def send_cmd(self, cmd: str) -> None:
		self.get_logger().info(f"sending cmd: '{cmd}' to arduino")
		resp = self._arduino.send(cmd)
		self.get_logger().info(f"arduino responded with: {resp}")

	# --- Sub Callback Functions ---

	def _cmd_vel_cb(self, msg: Twist) -> None:
		# turn on brake lights if linear and angular z is 0
		# these are floats
		linear_x  = msg.linear.x
		angular_z = msg.angular.z

		# check the linear/ang speed
		if (linear_x == 0.0 and angular_z == 0.0):
			self._brake_light_on = True
		else:
			self._brake_light_on = False

		#self.get_logger().info(f"{linear_x}, {angular_z}") 


	def _lane_cmd_cb(self, msg : String) -> None:
		direction = msg.data.lower()

		# reset any pre-existing blinker timer
		if self._blinker_timer is not None:
			self._blinker_timer.cancel()
			self._blinker_timer = None

		if direction == 'left':
			self.send_cmd("LEFT_ON")
			self._left_light_on = True
		elif direction == 'right':
			self.send_cmd("RIGHT_ON")
			self._right_light_on = True

		# schedule timer to auto send the OFF cmd
		self._blinker_timer = self.create_timer(3.0, self._blinker_off)

	# --- Misc Functions ---

	def _blinker_off(self) -> None:
		if self._left_light_on:
			self.send_cmd('LEFT_OFF')
			self._left_light_on = False

		if self._right_light_on:
			self.send_cmd('RIGHT_OFF')
			self._right_light_on = False

		# reset the timer to none
		if self._blinker_timer is not None:
			self._blinker_timer.cancel()
			self._blinker_timer = None

	def _apply_brake_lights(self) -> None:
		### intent is coded directly as a edge detector here!
		### we only send commands when an 'edge' is detected

		if self._brake_light_on and not self._brake_on:
			self.send_cmd("BRAKE_ON")
			self._brake_on = True
		elif self._brake_light_on and self._brake_on:
			pass # do nothing if lights are on and they need to stay on
		elif not self._brake_light_on and not self._brake_on:
			pass # do nothing if lights are off and they need to stay off
		elif not self._brake_light_on and self._brake_on:
			self.send_cmd("BRAKE_OFF")
			self._brake_on = False

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