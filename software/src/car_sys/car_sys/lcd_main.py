# car_sys/car_sys/lcd_main.py
import rclpy
from car_sys import LcdDisplay

def main(args=None):
    rclpy.init(args=args)
    node = LcdDisplay("car_lcd")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()        