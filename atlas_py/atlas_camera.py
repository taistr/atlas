import sys
import cv2
import logging

# Camera params
CAMERA_NUMBER = 0
CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640
CAMERA_FPS = 30

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class Camera():
    def __init__(
            self,
            camera_number: int = 0,
            camera_width: int = 640,
            camera_height: int = 480,
            camera_fps: int = 30
            ): 
        self.logger = logging.getLogger(__name__)

        self.cap = cv2.VideoCapture(camera_number)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, camera_fps)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")
        
        self.logger.info("Camera initialised")

    def capture_image(self):
        """Capture an image from the camera and return it"""

        ret, image = self.cap.read()
        if not ret:
            raise RuntimeError("Could not capture an image from the camera")
        

        return image

    def cleanup(self) -> None:
        """Clean up the rest of the resources"""
        self.cap.release()


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


