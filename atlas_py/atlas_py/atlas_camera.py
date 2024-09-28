import cv2
import logging
import numpy as np

# Camera params
DEFAULT_ID = 0
DEFAULT_FRAME_WIDTH = 480
DEFAULT_FRAME_HEIGHT = 640

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class Camera():
    def __init__(
            self,
            id: int = DEFAULT_ID,
            frame_width: int = DEFAULT_FRAME_WIDTH,
            frame_height: int = DEFAULT_FRAME_HEIGHT,
            ): 
        self.logger = logging.getLogger(__name__)

        self.video_capture = cv2.VideoCapture(id, cv2.CAP_V4L2)

        if not self.video_capture.isOpened():
            raise RuntimeError("Error: Could not open webcam.")
        
        self.logger.info("Camera initialised")

    def __del__(self):
        self.video_capture.release()
        cv2.destroyAllWindows()
        self.logger.info("Camera released")

    def capture_image(self) -> np.ndarray | None:
        """Capture an image from the camera and return it"""
        
        ret, frame = self.video_capture.read()

        if ret:
            return frame
        else:
            return
        
