import sys
import rclpy
import rclpy._rclpy_pybind11
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class Hello(Node):
    def __init__(self):
        super().__init__("hello")
        self.get_logger().info("Hello World!")

def main(args: dict = None):
    rclpy.init(args=args)
    node = Hello()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass

if __name__ == "__main__":
    main()
