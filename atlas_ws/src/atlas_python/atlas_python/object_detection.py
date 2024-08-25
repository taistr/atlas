import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import pathlib
import cv2

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")
        self.initialise_parameters()
        
        model_directory_path = pathlib.Path(get_package_share_directory("atlas_python")) / "models"
        self.model = YOLO(str(model_directory_path / self.get_parameter("model_name").value))

        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            "atlas/capture",
            self.detect_objects,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )
        self.detection_publisher = self.create_publisher(
            Image,
            "atlas/detection",
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )

        self.get_logger().info("Object Detection Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the camera node"""
        self.declare_parameter(
            "model_name",
            value="best_ncnn_model",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the model to use for object detection"
            )
        )

    def detect_objects(self, image: Image) -> None:
        """Detect objects in the received image"""
        frame = self.bridge.imgmsg_to_cv2(image)
        results = self.model(frame, conf=0.5)
        det_annotated = results[0].plot()
        image_message = CvBridge().cv2_to_imgmsg(det_annotated)
        self.detection_publisher.publish(image_message)

def main(args: dict = None):
    rclpy.init(args=args)
    
    object_detection = ObjectDetection()
    try:
        rclpy.spin(object_detection, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    object_detection.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
