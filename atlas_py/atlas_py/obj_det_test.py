import cv2
from ultralytics import YOLO

def main():
    # Load the YOLO model
    model = YOLO("/home/atlas/Desktop/atlas/atlas_py/atlas_py/models/29_08_640px.")

    while True:
        # Initialize the webcam
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            continue
        ret, frame = cap.read()
        cap.release()

        if not ret:
            print("Error: Failed to capture image.")
            break

        # Run object detection on the frame
        results = model(frame)

        # Extract the annotated frame (with bounding boxes, labels, etc.)
        annotated_frame = results[0].plot()  # This draws the detections on the frame

        # Display the annotated frame
        cv2.imshow('YOLO Object Detection', annotated_frame)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
