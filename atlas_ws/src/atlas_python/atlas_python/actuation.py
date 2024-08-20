import sys
import rclpy
from rclpy.node import Node

class Actuation(Node):
    def __init__(self) -> None:
        super().__init__("actuation")
        

