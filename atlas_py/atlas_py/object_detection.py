import sys
from ultralytics import YOLO
import pathlib
import torch
import cv2
import logging
from dataclasses import dataclass
import numpy as np

# Dependencies
from atlas_camera import DEFAULT_FRAME_WIDTH

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

@dataclass
class DetectionResult:
    """
    Represents the result of an object detection operation.

    Attributes:
        detection (bool): Whether an object was detected.
        angle (float): The calculated angle offset of the detected object relative to the robot's heading.
        distance (float): The estimated distance to the detected object.
    """
    detection: bool
    angle: float
    distance: float

class ObjectDetection:
    """
    A class to handle object detection using a pre-trained YOLO model.

    Attributes:
        logger (logging.Logger): Logger for the object detection class.
        model (YOLO): Pre-trained YOLO model used for object detection.
        response (DetectionResult): The latest detection result.
    """

    def __init__(self, model_path="/home/atlas/Desktop/atlas/atlas_py/atlas_py/models/29_08_640px.onnx"):
        """
        Initializes the ObjectDetection class by loading the YOLO model.

        :param model_path: Path to the YOLO model weights file.
        """
        self.logger = logging.getLogger(__name__)
        self.model = YOLO(model_path, task="detect")
        self.response = DetectionResult(
            detection=False,
            angle=0.0,
            distance=0.0
        )
        self.logger.info("ObjectDetection initialised!")

    def detect_object(self, frame: np.ndarray) -> DetectionResult:
        """
        Detect objects in the provided image frame and calculate their relative angle and distance.

        :param frame: The image frame in which to detect objects.
        :return: A DetectionResult object containing detection status, angle, and distance.
        """
        # Calculate the horizontal center of the frame
        frame_center_x = DEFAULT_FRAME_WIDTH // 2

        # Run object detection using the YOLO model with a confidence threshold of 0.5
        results = self.model(frame, conf=0.5)

        # Extract bounding boxes from the detection results
        boxes = results[0].boxes

        # Check if any objects were detected
        if boxes.cls.tolist():
            # Find the index of the most confident detected object
            _, max_index = torch.max(boxes.conf, dim=0)

            # Extract the center x, center y, width, and height of the most confident object's bounding box
            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            # Coefficients for calculating the heading offset angle based on x distance from frame center
            slope_angle = 0.0632085093204184
            intercept_angle = 0.7163631056314085
            x_distance = x_centre - frame_center_x

            # Coefficients for estimating the distance to the object based on bounding box height
            slope_distance = 50.924376407217764 * 1.015
            intercept_distance = 0.1332096544887631

            # Return detection results with calculated angle and distance
            return DetectionResult(
                detection=True,
                angle=slope_angle * x_distance + intercept_angle,
                distance=slope_distance / height + intercept_distance
            )
        else:
            # Return a result indicating no detection
            return DetectionResult(
                detection=False,
                angle=0.0,
                distance=0.0
            )
