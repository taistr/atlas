import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import RPi.GPIO as GPIO
except RuntimeError: # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock as GPIO

class Hello(Node):
    def __init__(self):
        super().__init__("hello")
        self.get_logger().info("Hello World!")

def main(args: dict = None):
    rclpy.init(args=args)
    
    actuation = Hello()
    rclpy.spin(actuation)
    actuation.destroy_node()

    actuation = Hello()
    try:
        actuation.get_logger().info("Starting Hello World!")
        rclpy.spin(actuation)
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    actuation.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
