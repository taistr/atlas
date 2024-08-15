import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.initialise_parameters()
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            sys.exit(1)

        self.capture_timer = self.create_timer(
            1.0 / self.get_parameter("capture_rate").value,
            self.capture_image
        )
        
        self.capture_publisher = self.create_publisher(
            Image,
            "atlas/capture",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )

        self.get_logger().info("Camera Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the camera node"""
        self.declare_parameter(
            "capture_rate",
            value=30.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which the camera captures images (frames per second)"
            )
        )

    def capture_image(self) -> None:
        """Capture an image from the camera and publish it"""

        ret, image = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        
        image_message = CvBridge().cv2_to_imgmsg(image)
        self.capture_publisher.publish(image_message)


def main(args: dict = None):
    rclpy.init(args=args)
    
    camera = Camera()
    try:
        rclpy.spin(camera, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    camera.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
