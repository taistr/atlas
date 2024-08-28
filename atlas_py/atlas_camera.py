import sys
#from cv_bridge import CvBridge
import cv2

# Camera params
CAMERA_NUMBER = 0
CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640
CAMERA_FPS = 30
IMAGE_ENCODING = "bgr8"


class Camera():
    def __init__(self):        
        self.cap = cv2.VideoCapture(CAMERA_NUMBER)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not self.cap.isOpened():
            print("Failed to open camera")
            try:
                self.cap.release()
            except:
                pass
            finally:
                sys.exit(1)

        print("camera.py:I CAN SEE U KUNTZ")

    def capture_image(self):
        """Capture an image from the camera and return it"""

        ret, image = self.cap.read()
        if not ret:
            print("camera.py:Failed to capture image")
            return
        
        return ret, image

    def cleanup(self) -> None:
        """Clean up the rest of the resources"""
        self.cap.release()


def take_image(args: dict = None):
    camera = Camera()
    try:
        ret, image = camera.capture_image()
    except:
        print("camera.py:Camera failed")
        camera.cleanup()
        sys.exit(1)
    finally:
        camera.cleanup()

    return image


