#### File: device/device.py
#### By: Arnav Goyal 

# global packages
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod

# local imports
from .type import DeviceType

class BaseDevice(Node, ABC):
    """
    A Basic ROS 2 node that all physical devices should extend. This should NEVER be instantiated directly!
    Only its child classes should be instantiated. ABC extension here guarantees this is never directly instantiated
    """

    ###############################################
    ####               CONSTRUCTOR              ###
    ###############################################

    def __init__(self, node_name: str, device_type: DeviceType):
        """
        Class Constructor
        """
        
        # init parnet
        super().__init__(node_name)
        
        self.device_type = device_type

        # setup better logging (less verbose)
        self._setup_logger(node_name=node_name, device_type=device_type)
        
        # echo a startup message
        self.get_logger().info(f"Starting Node: {node_name}")
        
        # declaration of ros parameters. This can be changed easily later if needed
        self.declare_parameter('update_rate', 10.0)
        
        # create is_active field
        self.is_active    = self._initialize()
        if not self.is_active:
            self.get_logger().error("Initialization Failed. This node will not spin")

        # create the timer for the device's loop
        timer_period = 1.0 / self.get_parameter('update_rate').value
        self.loop_timer = self.create_timer(timer_period, self._loop)
        self.update_rate = self.get_parameter('update_rate').value
        
        self.get_logger().info(f"Node Initialized, Publish & Loop rate set to {self.update_rate} Hz")

    ###############################################
    ####             CUSTOM LOGGING             ###
    ###############################################

    def _setup_logger(self, node_name: str, device_type: DeviceType):
        """
        Internal method to change the shitty default ROS2 Logger
        """
        _logger_name = f"{device_type}:{node_name}"
        self._custom_logger = rclpy.logging.get_logger(_logger_name)

    @property
    def get_logger(self):
        """
        Overrides the get_logger method from within this class to return the
        custom logger we have instead of the default one from ROS2
        """
        return lambda: self._custom_logger

    ###############################################
    ####            OVERRIDE METHODS            ###
    ###############################################
    
    @abstractmethod
    def _initialize(self) -> bool:
        """ 
        Returns True if hardware is detected and ready! This sets the self.is_active var
        """
        pass

    @abstractmethod
    def _shutdown(self) -> None:
        """
        Shutdown func, release GPIO pins, etc etc
        """
        pass

    @abstractmethod
    def _loop(self) -> None:
        """
        Main loop of the device while its active
        """
        pass

    ###############################################
    ####             GENERAL METHODS            ###
    ###############################################
    
    def destroy_node(self):
        """
        Override the default shutdown method to call _shutdown first
        """
        self._shutdown()
        super().destroy_node()