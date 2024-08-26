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
            value="26_08_best.pt",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the model to use for object detection"
            )
        )

    def image_callback(self, msg: Image) -> None:
        """Callback function for the image subscriber"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg) #Stores the image as BGR8 as per the CvBridge documentation
        

    def detect_objects(self, request: Detection.Request, response: Detection.Response) -> Detection.Response:
        """Detect objects in the received image"""
        frame_width = 640 # change later
        frame_height = 480
        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2
        # Run inference on latest image
        results = self.model(self.latest_image, conf=0.7)

        # Publish annotated image
        image_message = CvBridge().cv2_to_imgmsg(results[0].plot())
        self.detection_publisher.publish(image_message)

        #TODO: Populate response with detected objects
        boxes = results[0].boxes

        if boxes.cls.tolist(): #If an object was detected
            # Find the most confident object
            _, max_index = torch.max(boxes.conf, dim=0)

            # Extract the xyxy coordinates of the most confident object
            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            # TODO: Calculate the heading offset
            slope_angle=0.0632085093204184
            intercept_angle=0.7163631056314085
            x_distance = x_centre - frame_center_x

            # TODO: Calculate the distance to the object (hint: use a dictionary)
            slope_distance=50.924376407217764
            intercept_distance=0.1332096544887631

            response.detection = True
            response.angle = slope_angle*x_distance+intercept_angle
            response.distance = slope_distance/height+intercept_distance
        else:
            response.detection = False
        return response


def main(args: dict = None):
    rclpy.init(args=args)
    
    object_detection = ObjectDetection()
    try:
        rclpy.spin(object_detection, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

if __name__ == "__main__":
    main()
