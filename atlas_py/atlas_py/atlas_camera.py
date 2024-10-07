import cv2
import logging
import numpy as np
import threading

# Camera parameters
DEFAULT_ID = 2  # Default camera ID
CAMERA_FOV = 35
DEFAULT_FRAME_WIDTH = 640  # Default frame width
DEFAULT_FRAME_HEIGHT = 480  # Default frame height
DEFAULT_FPS = 30  # Default frames per second (FPS)
# FOURCC codes for MJPEG and YUYV formats
FOURCC_MJPEG = cv2.VideoWriter_fourcc(*'MJPG')  # MJPEG format
FOURCC_YUYV = cv2.VideoWriter_fourcc(*'YUYV')   # YUYV format

class Camera:
    """
    Represents a camera interface for capturing frames using OpenCV.

    Attributes:
        logger (logging.Logger): Logger for the Camera class.
        video_capture (cv2.VideoCapture): OpenCV video capture object.
    """

    def __init__(
            self,
            id: int = DEFAULT_ID,
            frame_width: int = DEFAULT_FRAME_WIDTH,
            frame_height: int = DEFAULT_FRAME_HEIGHT,
            fps: int = DEFAULT_FPS,
            fourcc_format=FOURCC_MJPEG
    ):
        """
        Initializes the Camera object with the specified parameters.

        :param id: Camera ID, usually 0 for the default camera.
        :param frame_width: The width of the captured frames.
        :param frame_height: The height of the captured frames.
        :param fps: The frame rate for capturing.
        :param fourcc_format: The FOURCC format for the captured frames.
        """
        self.logger = logging.getLogger(__name__)

        # Open the video capture with V4L2 (Video for Linux)
        self.video_capture = cv2.VideoCapture(id, cv2.CAP_V4L2)

        # Check if the camera was opened successfully
        if not self.video_capture.isOpened():
            raise RuntimeError("Error: Could not open webcam.")
        
        # Set the desired resolution and FPS
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.video_capture.set(cv2.CAP_PROP_FPS, fps)

        # Set the pixel format (FOURCC)
        self.video_capture.set(cv2.CAP_PROP_FOURCC, fourcc_format)

        self.logger.info(f"Camera initialized with resolution {frame_width}x{frame_height} at {fps} FPS "
                         f"using format {'MJPEG' if fourcc_format == FOURCC_MJPEG else 'YUYV'}")
        
    def __del__(self):
        """
        Releases the camera resources and closes any open windows upon object deletion.
        """
        self.video_capture.release()
        cv2.destroyAllWindows()
        self.logger.info("Camera released")

    def capture_image(self) -> np.ndarray | None:
        """
        Capture an image from the camera and return it as a numpy array.

        :return: The captured frame as a numpy array, or None if capture fails.
        """
        ret, frame = self.video_capture.read()

        if ret:
            return frame
        else:
            return None

class FrameGrabber:
    """
    A class that continuously grabs frames from a Camera object in a separate thread.

    Attributes:
        camera (Camera): The Camera object to grab frames from.
        frame (np.ndarray): The latest captured frame.
        running (bool): Flag indicating whether the frame grabbing is active.
        lock (threading.Lock): Lock to ensure thread-safe access to the latest frame.
    """

    def __init__(self, camera: Camera):
        """
        Initializes the FrameGrabber with the specified Camera.

        :param camera: The Camera object to grab frames from.
        """
        self.logger = logging.getLogger(__name__)
        self.camera = camera
        self.frame = None
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        """
        Start the frame grabbing thread.
        """
        self.running = True
        self.thread = threading.Thread(target=self._grab_frames)
        self.thread.start()

    def stop(self):
        """
        Stop the frame grabbing thread.
        """
        self.running = False
        self.thread.join()  # Ensure the thread stops cleanly
        self.logger.info("FrameGrabber stopped.")

    def _grab_frames(self):
        """
        Continuously grab frames from the camera in a separate thread.
        """
        while self.running:
            frame = self.camera.capture_image()
            if frame is not None:
                with self.lock:
                    self.frame = frame

    def get_latest_frame(self) -> np.ndarray | None:
        """
        Retrieve the latest captured frame.

        :return: The latest captured frame as a numpy array, or None if no frame is available.
        """
        with self.lock:
            return self.frame
