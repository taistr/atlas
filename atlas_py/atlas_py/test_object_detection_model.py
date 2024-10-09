import cv2
import logging
from ultralytics import YOLO
from atlas_camera import Camera, FrameGrabber
import numpy as np

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def test_camera_with_yolo():
    """
    Test script using Camera and FrameGrabber with YOLO object detection.
    """
    logger = logging.getLogger("CameraYOLOTest")

    # Initialize the YOLO model
    model = YOLO("/home/atlas/Desktop/atlas/atlas_py/atlas_py/models/box/08_10_640px_box.pt")
    logger.info("YOLO model loaded successfully.")

    # Initialize Camera and FrameGrabber
    try:
        logger.info("Initializing camera...")
        camera = Camera()
        frame_grabber = FrameGrabber(camera)
        frame_grabber.start()
    except Exception as e:
        logger.error(f"Failed to initialize camera: {e}")
        return

    try:
        while True:
            # Capture a frame using the FrameGrabber
            logger.info("Capturing frame...")
            frame = frame_grabber.get_latest_frame()
            
            if frame is None or not isinstance(frame, (np.ndarray)):
                logger.error("Failed to capture a valid frame.")
                continue

            # Run object detection on the captured frame
            logger.info("Running object detection on the captured frame...")
            results = model(frame)

            # Extract the annotated frame (with bounding boxes, labels, etc.)
            annotated_frame = results[0].plot()  # Draw detections on the frame

            # Display the annotated frame
            cv2.imshow('YOLO Object Detection', annotated_frame)

            # Break the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    except KeyboardInterrupt:
        logging.info("Detection stopped by user.")

    finally:
        # Stop the frame grabber and release camera resources
        frame_grabber.stop()
        del camera
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera_with_yolo()
