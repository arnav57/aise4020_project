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
        
        # echo a startup message
        self.get_logger().info(f"starting node: {node_name}")
        
        # declaration of ros parameters. This can be changed easily later if needed
        self.declare_parameter('update_rate', 10.0)
        
        # create is_active field
        self.is_active       = False
        self._attempted_init = False

        # create the timer for the device's loop
        timer_period = 1.0 / self.get_parameter('update_rate').value
        self.loop_timer = self.create_timer(timer_period, self._internal_callback)
        self.update_rate = self.get_parameter('update_rate').value
        
        self.get_logger().info(f"node created, publish & loop rate set to {self.update_rate} Hz")
        self.get_logger().info(f"my ros-name is: '{self.get_fully_qualified_name()}'")

        # connect the guaranteed shutdown callback to the internal func
        self._context.on_shutdown(self._internal_shutdown_callback)

    def _internal_callback(self):
        """
        Private method that handles the FSM logic in here. We only run _initialize after the node is fully up and running
        """
        if not self._attempted_init:
            self.get_logger().info("attempting to initialize...")
            self.is_active = self._initialize()
            self._attempted_init = True

            if not self.is_active:
                self.get_logger().error("initialization failed...")
            else:
                self.get_logger().info("initialized successfully!")

        if self.is_active:
            self._loop()

    def _internal_shutdown_callback(self):
        """
        We need this because ROS2 guarantees this guy will run on shutdown!
        """
        self._shutdown()


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