import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image
from atlas_msgs.srv import Detection
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import pathlib
import torch

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")
        self.initialise_parameters()
        
        self.get_logger().info("Loading model...")
        model_path = pathlib.Path(
            get_package_share_directory("atlas_python")
        ) / "models" / self.get_parameter("model_name").value
        self.model = YOLO(str(model_path))

        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            5,
        )
        self.detection_service = self.create_service(
            Detection,
            "atlas/detect_objects",
            self.detect_objects,
        )
        self.detection_publisher = self.create_publisher(
            Image,
            "atlas/detect_image",
            5,
        )

        self.latest_image = None

        self.get_logger().info("Object Detection Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the camera node"""
        self.declare_parameter(
            "model_name",
            value="29_08_640px.onnx",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the model to use for object detection"
            )
        )

    def image_callback(self, msg: Image) -> None:
        """Callback function for the image subscriber"""
        # TODO: Fix the channels of the image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") #Stores the image as BGR8 as per the CvBridge documentation

    def detect_objects(self, request: Detection.Request, response: Detection.Response) -> Detection.Response:
        """Detect objects in the received image"""
        frame_center_x = 640 // 2 # TODO: Replace with the actual frame width from the camera node

        if self.latest_image is None:
            self.get_logger().info("There was no camera image to detect objects on!")
            response.detection = False
            return response
        
        self.get_logger().info("Detecting objects...")
        results = self.model(self.latest_image, conf=0.5)
        self.get_logger().info("Detection complete!")

        # Publish annotated image
        image_message = CvBridge().cv2_to_imgmsg(results[0].plot())
        self.detection_publisher.publish(image_message)

        # Populate response with detected objects
        boxes = results[0].boxes

        if boxes.cls.tolist(): # If an object was detected
            # Find the most confident object
            _, max_index = torch.max(boxes.conf, dim=0)

            # Extract the xyxy coordinates of the most confident object
            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            # Calculate the heading offset
            slope_angle = 0.0632085093204184
            intercept_angle = 0.7163631056314085
            x_distance = x_centre - frame_center_x

            # Calculate the distance to the object (hint: use a dictionary)
            slope_distance = 50.924376407217764
            intercept_distance = 0.1332096544887631
            
            response.detection = True
            response.angle = slope_angle * x_distance + intercept_angle
            response.distance = slope_distance / height + intercept_distance

            return response
        else:
            response.detection = False
            return response

def main(args: dict = None):
    rclpy.init(args=args)
    node = ObjectDetection()
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
