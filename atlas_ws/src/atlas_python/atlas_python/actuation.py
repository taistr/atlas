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

    rclpy.shutdown()

if __name__ == "__main__":
    main()
