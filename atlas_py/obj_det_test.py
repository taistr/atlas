import cv2
from ultralytics import YOLO

def main():
    # Initialize the webcam
    cap = cv2.VideoCapture(0)

    # Load the YOLO model
    model = YOLO("/home/tyson/atlas/atlas_py/models/26_08_640px.pt")

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()

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
