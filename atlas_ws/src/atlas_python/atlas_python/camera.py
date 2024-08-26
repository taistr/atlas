import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.initialise_parameters()
        
        self.cap = cv2.VideoCapture(self.get_parameter("camera_number").value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter("camera_width").value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter("camera_height").value)
        self.cap.set(cv2.CAP_PROP_FPS, self.get_parameter("camera_fps").value)

        self.get_logger().info(
            f"Camera Parameters: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ {self.cap.get(cv2.CAP_PROP_FPS)} FPS"
        )

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            sys.exit(1)
        
        self.capture_publisher = self.create_publisher(
            Image,
            "atlas/capture",
            5,
        )

        self.get_logger().info("Camera Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the camera node"""
        self.declare_parameter( #MAKE SURE THIS IS SET TO PI CAMERA
            "camera_number",
            0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Camera number to use for capturing images",
            ),
        )
        self.declare_parameter(
            "camera_height",
            240,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Height of the camera image",
            ),
        )
        self.declare_parameter(
            "camera_width",
            320,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Width of the camera image",
            ),
        )
        self.declare_parameter(
            "camera_fps",
            30,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Frames per second of the camera",
            ),
        )
        self.declare_parameter(
            "image_encoding",
            "bgr8",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="ROS Image encoding of the image",
            ),
        )

    def capture_image(self) -> None:
        """Capture an image from the camera and publish it"""

        ret, image = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        
        image_message = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        self.capture_publisher.publish(image_message)
        self.executor.create_task(self.capture_image)


def main(args: dict = None):
    rclpy.init(args=args)
    
    camera = Camera()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(camera)
        executor.create_task(camera.capture_image)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    camera.destroy_node()

if __name__ == "__main__":
    main()
