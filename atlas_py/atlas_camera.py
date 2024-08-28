import sys
import cv2
import logging
import numpy as np

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class Camera():
    def __init__(
            self,
            camera_number: int = 0,
            ): 
        self.logger = logging.getLogger(__name__)

        self.camera_number = camera_number
        
        self.logger.info("Camera initialised")

    def capture_image(self) -> np.ndarray | None:
        """Capture an image from the camera and return it"""

        cap = cv2.VideoCapture(self.camera_number)

        if not cap.isOpened():
            self.logger("Error: Could not open webcam.")
            return
        
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            return frame
        else:
            return


