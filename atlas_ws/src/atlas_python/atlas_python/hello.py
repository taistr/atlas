import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
    
    hello = Hello()
    try:
        hello.get_logger().info("Starting Hello World!")
        rclpy.spin(hello)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    hello.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
