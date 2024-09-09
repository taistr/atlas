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
   
    def line_intersection(p1, p2, q1, q2):
        """ Return the intersection point of lines (p1, p2) and (q1, q2) if they intersect """
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        A, B, C, D = p1, p2, q1, q2
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    def does_intersect_line(x_obj, y_obj, line_start, line_end, height):
        # Define the vertical line segment from (x_obj, y_obj) to (x_obj, height)
        vertical_start = (x_obj, y_obj)
        vertical_end = (x_obj, height)
        return line_intersection(vertical_start, vertical_end, line_start, line_end)

    def detect_objects(self, request: Detection.Request, response: Detection.Response) -> Detection.Response:
        """Detect objects in the received image"""
        frame_width = 640 # change later
        frame_height = 480
        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2

        frame=self.latest_image

        # Line detection
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply GaussianBlur to reduce noise and improve edge detection
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)

        # Apply binary thresholding to highlight the white lines
        _, binary = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)

        # Detect edges using the Canny edge detector
        edges = cv2.Canny(binary, 50, 150, apertureSize=3)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)

        # Run inference on latest image
        results = self.model(frame, conf=0.7)

        # Publish annotated image
        image_message = CvBridge().cv2_to_imgmsg(results[0].plot())
        self.detection_publisher.publish(image_message)

        # Populate response with detected objects
        boxes = results[0].boxes

        if boxes.cls.tolist(): #If an object was detected
            # Find the most confident object 
            "Do we want to use highest confidence object or closest object?"
            _, max_index = torch.max(boxes.conf, dim=0)

            # Extract the xyxy coordinates of the most confident object
            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            is_object_in_front = False

            if lines is not None:
                for line in lines:
                    rho, theta = line[0]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    # Check intersection with vertical line
                    if does_intersect_line(x_centre, y_centre, (x1, y1), (x2, y2), height):
                        is_object_in_front = False
                        break
                else:
                    is_object_in_front = True

            if is_object_in_front:
                #print("Object is in front of the line (clear path to bottom of image).")
                # Calculate the heading offset
                slope_angle=0.0632085093204184
                intercept_angle=0.7163631056314085
                x_distance = x_centre - frame_center_x

                # Calculate the distance to the object (hint: use a dictionary)
                slope_distance=50.924376407217764
                intercept_distance=0.1332096544887631

                response.detection = True
                response.angle = slope_angle*x_distance+intercept_angle
                response.distance = slope_distance/height+intercept_distance
            else:
                #print("Object is behind the line (no clear path to bottom of image).")
                response.detection = False

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
