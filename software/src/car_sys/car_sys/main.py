import rclpy
from rclpy.executors import MultiThreadedExecutor

# Local imports from your sub-modules
from car_sys import LcdDisplay
# from .device.camera import CameraDevice  # (Once you make it)

def main(args=None):
    """
    The master entry point for the car_sys package.
    Initializes the ROS network and manages the lifecycle of all car devices.
    """
    rclpy.init(args=args)

    try:
        # 1. Instantiate your devices
        lcd_node = LcdDisplay()
        # camera_node = CameraDevice()

        # 2. Use an Executor to run multiple nodes in one process
        # This allows the LCD and other devices to run concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(lcd_node)
        # executor.add_node(camera_node)

        lcd_node.get_logger().info("Master runner is online. Spinning all devices...")

        try:
            # This blocks until Ctrl+C
            executor.spin()
        finally:
            # 3. Clean shutdown
            executor.shutdown()
            lcd_node.destroy_node()
            # camera_node.destroy_node()

    except Exception as e:
        print(f"Master Runner crashed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()