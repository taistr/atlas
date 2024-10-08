from ultralytics import YOLO
import torch
import logging
from dataclasses import dataclass
import numpy as np
from enum import Enum

# Dependencies
from atlas_camera import DEFAULT_FRAME_WIDTH, CAMERA_FOV

FRAME_CENTER_X = DEFAULT_FRAME_WIDTH // 2
MINIMUM_BALL_DETECTION_CONFIDENCE = 0.5
MINIMUM_BOX_DETECTION_CONFIDENCE = 0.5
DEFAULT_BALL_MODEL_PATH = "/home/atlas/Desktop/atlas/atlas_py/atlas_py/models/29_08_640px.onnx"
DEFAULT_BOX_MODEL_PATH = "/home/atlas/atlas_py/atlas_py/models/box/08_10_640px_box.pt"

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

class BallDetection:
    """
    A class to handle tennis ball detection using a pre-trained YOLO model.

    Attributes:
        logger (logging.Logger): Logger for the object detection class.
        model (YOLO): Pre-trained YOLO model used for object detection of balls.
        response (DetectionResult): The latest detection result.
    """

    def __init__(self, ball_model_path: str = DEFAULT_BALL_MODEL_PATH):
        """
        Initializes the BallDetection class by loading the YOLO model.

        :param model_path: Path to the YOLO model weights file.
        """
        self.logger = logging.getLogger(self.__class__.__name__)

        self.ball_model = YOLO(ball_model_path, task="detect")

        self.logger.info("Ball Detection initialised!")

    def detect_balls(self, frame: np.ndarray) -> DetectionResult:
        """
        Detect balls in the provided image frame and calculate their relative angle and distance.

        :param frame: The image frame in which to detect objects.
        :return: A DetectionResult object containing detection status, angle, and distance.
        """

        results = self.ball_model(frame, conf=MINIMUM_BALL_DETECTION_CONFIDENCE)
        boxes = results[0].boxes

        detection_results = []
        iterable = 0
        for detection in boxes.cls.tolist():
            x_centre, _, _, height = boxes.xywh[iterable].tolist()
            angle = self.calculate_ball_angle(x_centre)
            distance = self.calculate_ball_distance(height)
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
            return DetectionResult(
                detection=False,
                angle=0.0,
                distance=0.0
            )
    
    def calculate_ball_angle(self, x_centre: float) -> float:
        x_distance = FRAME_CENTER_X - x_centre  # Reverse x_distance
        slope_angle = 0.0632085093204184
        return slope_angle * x_distance

    
    def calculate_ball_distance(self, height: float) -> float:
        """
        Calculate the estimated distance to the detected object.
        
        :param height: The height of the detected object's bounding box.
        :return: The estimated distance to the detected object.
        """
        slope_distance = 50.924376407217764 * 1.015
        intercept_distance = 0.1332096544887631
        return slope_distance / height + intercept_distance
    
class BoxDetection:
    """
    A class to handle box detection using a pre-trained YOLO model

    """

    def __init__(self, box_model_path: str = DEFAULT_BOX_MODEL_PATH) -> None:
        """
        """
        self.logger = logging.getLogger(self.__class__.__name__)

        self.box_model = YOLO(box_model_path, task="detect")

        self.logger.info("Box Detection online!")

    def detect_box(self, frame: np.ndarray) -> DetectionResult:
        """
        Detect a box in the provided image frame and calculate its relative angle.

        :param frame: The image frame in which to detect objects.
        :return: A DetectionResult object containing detection status, angle, and distance. Distance will always be returned as 0.
        """

        results = self.model(frame, conf=MINIMUM_BALL_DETECTION_CONFIDENCE)
        boxes = results[0].boxes

        # If any objects were detected
        if boxes.cls.tolist():
            _, max_index = torch.max(boxes.conf, dim=0)

            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            # calculate the angle
            angle = self.detect_box_angle(x_centre)

            return DetectionResult(
                detection=True,
                angle=angle,
                distance=0 # TODO: Implement distance calculation
            )
        else:
            return DetectionResult(
                detection=False,
                angle=0,
                distance=0
            )
        
    def detect_box_angle(self, x_centre: float) -> float:
        """
        Detect the angle of a box relative to the robot.

        :param x_centre: the centre of the object in the frame
        :return: the angle of the object in degrees
        """
        x_relative = x_centre - FRAME_CENTER_X
        angle = (x_relative / DEFAULT_FRAME_WIDTH) * CAMERA_FOV / 2

        return angle





