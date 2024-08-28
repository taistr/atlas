import sys
#from cv_bridge import CvBridge
from ultralytics import YOLO
import pathlib
import torch
import cv2
import logging
from dataclasses import dataclass

# Dependencies
from atlas_camera import Camera

FRAME_WIDTH = 640 
FRAME_HEIGHT = 480

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

@dataclass
class DetectionResult:
    detection: bool
    angle: float
    distance: float

class ObjectDetection():
    def __init__(self, model_path=pathlib.Path("./models/26_08_640px.pt")):        
        self.model = YOLO(model_path)
        self.logger = logging.getLogger(__name__)

        self.camera = Camera()
        self.latest_image = None
        self.response = DetectionResult(
            detection=False,
            angle=float(0),
            distance=float(0)
        )

        self.logger.info("ObjectDetection initialised!")
    
    def cleanup(self) -> None:
        """Clean up the rest of the resources"""
        self.camera.cleanup()

    def detect_object(self) -> DetectionResult:
        """Detect objects in the received image"""
        frame_center_x = FRAME_WIDTH // 2

        self.latest_image = self.camera.capture_image()
        results = self.model(self.latest_image, conf=0.7)

        # Populate response with detected objects
        boxes = results[0].boxes

        if boxes.cls.tolist(): #If an object was detected
            # Find the most confident object
            _, max_index = torch.max(boxes.conf, dim=0)

            # Extract the xyxy coordinates of the most confident object
            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            # Calculate the heading offset
            slope_angle=0.0632085093204184
            intercept_angle=0.7163631056314085
            x_distance = x_centre - frame_center_x

            # Calculate the distance to the object (hint: use a dictionary)
            slope_distance=50.924376407217764
            intercept_distance=0.1332096544887631

            return DetectionResult(
                detection=True,
                angle=slope_angle * x_distance + intercept_angle,
                distance=slope_distance / height + intercept_distance
            )
        else:
            return DetectionResult(
                detection=False,
                angle=float(0),
                distance=float(0)
            )
