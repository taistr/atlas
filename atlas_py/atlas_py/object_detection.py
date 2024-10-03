from ultralytics import YOLO
import torch
import logging
from dataclasses import dataclass
import numpy as np
from enum import Enum

# Dependencies
from atlas_camera import DEFAULT_FRAME_WIDTH

FRAME_CENTER_X = DEFAULT_FRAME_WIDTH // 2
MINIMUM_DETECTION_CONFIDENCE = 0.5
DEFAULT_MODEL_PATH = "/home/atlas/Desktop/atlas/atlas_py/atlas_py/models/29_08_640px.onnx"

class DetectionClass(Enum):
    """
    Enumerates the objects that can be detected by the YOLO model.
    """
    TENNIS_BALL = 0
    CARDBOARD_BOX = 1

@dataclass
class DetectionResult:
    """
    Represents the result of an object detection operation.

    Attributes:
        detection (bool): Whether an object was detected.
        angle (float): The calculated angle offset of the detected object relative to the robot's heading.
        distance (float): The estimated distance to the detected object.
    """
    detection: DetectionClass | None
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

    def __init__(self, model_path=DEFAULT_MODEL_PATH):
        """
        Initializes the ObjectDetection class by loading the YOLO model.

        :param model_path: Path to the YOLO model weights file.
        """
        self.logger = logging.getLogger(__name__)
        self.model = YOLO(model_path, task="detect")
        self.logger.info("ObjectDetection initialised!")

    def detect_object(self, frame: np.ndarray, detection_class: DetectionClass = DetectionClass.TENNIS_BALL) -> DetectionResult:
        """
        Detect objects in the provided image frame and calculate their relative angle and distance.

        :param frame: The image frame in which to detect objects.
        :param detection_class: The class of object to detect.
        :return: A DetectionResult object containing detection status, angle, and distance.
        """

        results = self.model(frame, conf=MINIMUM_DETECTION_CONFIDENCE)
        boxes = results[0].boxes

        detection_results = []
        iterable = 0
        for detection in boxes.cls.tolist():
            if detection == DetectionClass.TENNIS_BALL.value and detection_class == DetectionClass.TENNIS_BALL:
                x_centre, _, _, height = boxes.xywh.tolist()
                angle = self.calculate_ball_angle(x_centre)
                distance = self.calculate_ball_distance(height)
                detection_results.append(
                    DetectionResult(
                        detection=True,
                        angle=angle,
                        distance=distance
                    )
                )
            elif detection == DetectionClass.CARDBOARD_BOX.value and detection_class == DetectionClass.CARDBOARD_BOX:
                x_centre, _, _, height = boxes.xywh.tolist()
                angle = self.calculate_box_angle(x_centre)
                distance = self.calculate_box_distance(height)
                detection_results.append(
                    DetectionResult(
                        detection=True,
                        angle=angle,
                        distance=distance
                    )
                )
            
            iterable += 1
        
        # return the object with the smallest distance or return there was no detection
        if detection_results:
            return min(detection_results, key=lambda x: x.distance)
        else:
            DetectionResult(
                detection=False,
                angle=0.0,
                distance=0.0
            )
    
    def calculate_box_angle(self, x_centre: float) -> float:
        """
        Calculate the angle offset of the detected object relative to the robot's heading.

        :param x_centre: The x-coordinate of the center of the detected object.
        :return: The calculated angle offset.
        """
        raise NotImplementedError("Method not implemented yet.")

    def calculate_box_distance(self, height: float) -> float:
        """
        Calculate the estimated distance to the detected object.
        
        :param height: The height of the detected object's bounding box.
        :return: The estimated distance to the detected object.
        """
        raise NotImplementedError("Method not implemented yet.")

    def calculate_ball_angle(self, x_centre: float) -> float:
        """
        Calculate the angle offset of the detected object relative to the robot's heading.

        :param x_centre: The x-coordinate of the center of the detected object.
        :return: The calculated angle offset.
        """
        x_distance = x_centre - FRAME_CENTER_X
        slope_angle = 0.0632085093204184
        intercept_angle = 0.7163631056314085
        return slope_angle * x_distance + intercept_angle
    
    def calculate_ball_distance(self, height: float) -> float:
        """
        Calculate the estimated distance to the detected object.
        
        :param height: The height of the detected object's bounding box.
        :return: The estimated distance to the detected object.
        """
        slope_distance = 50.924376407217764 * 1.015
        intercept_distance = 0.1332096544887631
        return slope_distance / height + intercept_distance

