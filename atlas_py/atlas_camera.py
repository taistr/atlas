import sys
import cv2
import logging
import numpy as np

# Camera params
CAMERA_NUMBER = 0
CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class Camera():
    def __init__(
            self,
            camera_number: int = 0,
            camera_width: int = 640,
            camera_height: int = 480,
            ): 
        self.logger = logging.getLogger(__name__)

        self.camera_number = camera_number
        self.camera_width = camera_width
        self.camera_height = camera_height
        
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


# def take_image(args: dict = None):
#     camera = Camera()
#     try:
#         ret, image = camera.capture_image()
#     except:
#         self.logger.info("camera.py:Camera failed")
#         camera.cleanup()
#         sys.exit(1)
#     finally:
#         camera.cleanup()

#     return image


