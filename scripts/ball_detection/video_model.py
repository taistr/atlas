import cv2
from ultralytics import YOLO
import time

# Load the trained YOLOv8 model (use a smaller model if available for speed)
model = YOLO('best.pt')

# Open a connection to the camera
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set camera resolution (reduce for speed)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Loop to continuously get frames from the camera
while True:
    #start_time = time.time()
    start_time1 = time.time()
    # Capture frame-by-frame
    ret, frame = cap.read()
    end_time1=time.time()
    print(f"Frame capture time: {end_time1-start_time1:.5f}")
    # If the frame was not retrieved properly, break the loop
    if not ret:
        break

    # Resize the frame for faster processing (optional)
    resized_frame = cv2.resize(frame, (640, 480))  # Adjust resolution as needed
    start_time = time.time()
    # Run YOLOv8 predictions on the resized frame
    results = model(resized_frame, conf=0.5)  # Adjust confidence threshold if needed
    end_time=time.time()
    print(f"Predict time: {end_time-start_time:.5f}")
    # Display the frame with predictions
    annotated_frame = results[0].plot()

    # Show the frame in a window named 'YOLOv8 Predictions'
    cv2.imshow('YOLOv8 Predictions', annotated_frame)

    # Calculate FPS and print (optional)
    #fps = 1 / (time.time() - start_time)
    #print(f"FPS: {fps:.2f}")

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
