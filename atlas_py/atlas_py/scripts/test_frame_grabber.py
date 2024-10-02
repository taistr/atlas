import logging
import cv2
import time
from atlas_camera import Camera, FrameGrabber
from ultralytics import YOLO
from object_detection import ObjectDetection, DetectionClass

# Configure logging
logging.basicConfig(level=logging.DEBUG)

def main():
    # Initialize the camera and frame grabber
    camera = Camera()  # Using default camera settings
    frame_grabber = FrameGrabber(camera)
    
    # Start frame grabbing
    frame_grabber.start()

    # Initialize the object detection model
    detection_model = ObjectDetection()

    try:
        while True:
            # Get the latest frame
            frame = frame_grabber.get_latest_frame()
            
            if frame is None:
                logging.debug("No frame captured, skipping detection.")
                time.sleep(0.1)  # Wait before trying again
                continue
            
            # Perform object detection on the latest frame
            detection_result = detection_model.detect_object(frame, DetectionClass.TENNIS_BALL)

            if detection_result.detection:
                logging.info(f"Tennis Ball Detected! Angle: {detection_result.angle:.2f}, Distance: {detection_result.distance:.2f}")
            else:
                logging.info("No object detected.")
            
            # Display the frame (optional)
            cv2.imshow("Camera Feed", frame)

            # Press 'q' to quit the loop
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
    main()
