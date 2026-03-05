#### File: device/type.py
#### By: Arnav Goyal 

# global packages
from enum import Enum, auto

class DeviceType(Enum):
	""" 
	An Enum that defines all types of devices we can use in this car
	"""
	CAMERA     = auto()
	ULTRASONIC = auto()
	INFRARED   = auto()
	LIDAR      = auto()
	DISPLAY    = auto() 

	def __str__(self):
		""" Custom str implementation for logging/printing """
		return self.name.capitalize()