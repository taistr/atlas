import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import RPi.GPIO as GPIO
except RuntimeError: # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock as GPIO

class Actuation(Node):
    def __init__(self):
        super().__init__("actuation")
        self.get_logger().info("Actuation Online!")

def main(args: dict = None):
    rclpy.init(args=args)
    
    actuation = Actuation()
    rclpy.spin(actuation)
    actuation.destroy_node()

    actuation = Actuation()
    try:
        actuation.get_logger().info("Starting actuation node, shut down with CTRL-C")
        rclpy.spin(actuation)
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    actuation.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
