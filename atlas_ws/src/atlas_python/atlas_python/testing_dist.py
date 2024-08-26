import cv2
from ultralytics import YOLO
import numpy as np
import pandas as pd

# Load the trained YOLOv8 model
model = YOLO('tennisball_best.pt')

# Initialize a list to store the results
results_list = []


while True:

    # Open a connection to the camera
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # Set camera resolution
    frame_width = 640
    frame_height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    # Calculate the center of the camera frame
    center_x = frame_width // 2
    center_y = frame_height // 2


    # Capture a single frame
    ret, frame = cap.read()

    if not ret:
        print("New frame not taken")

    # Check if the frame was retrieved properly
    if ret:
        # Run YOLOv8 predictions on the frame
        results = model(frame, conf=0.5)  # Adjust confidence threshold if needed

        # Annotate the frame with predictions
        annotated_frame = results[0].plot()


        # Display original frame
        cv2.imshow('Original Frame', frame)

        # Annotate the frame
        annotated_frame = results[0].plot()

        # Display annotated frame
        cv2.imshow('Annotated Frame', annotated_frame)


        # Variables to store the largest tennis ball's data
        largest_height = 0
        largest_ball_center_x = None
        largest_ball_center_y = None

        # Check if any detections were made
        for detection in results[0].boxes:
            # Assuming '0' is the class ID for the tennis ball
            if detection.cls == 0:
                # Get the bounding box coordinates (x_min, y_min, x_max, y_max)
                x_min, y_min, x_max, y_max = detection.xyxy[0].cpu().numpy()

                # Calculate the bounding box height
                height = y_max - y_min

                # If this is the largest height so far, update the largest ball data
                if height > largest_height:
                    largest_height = height
                    largest_ball_center_x = int((x_min + x_max) / 2)
                    largest_ball_center_y = int((y_min + y_max) / 2)

                print("Height" {largest_height})

        # If a largest tennis ball was found, calculate the distance and display it
        if largest_ball_center_x is not None:
            # Calculate the x-axis distance from the center of the frame
            x_distance = largest_ball_center_x - center_x
            print("Distance from center:" {x_distance})

            # Draw a line on the x-axis between the center of the frame and the center of the largest tennis ball
            cv2.line(annotated_frame, (center_x, center_y), (largest_ball_center_x, center_y), (0, 255, 0), 2)

            # Annotate the x-axis distance on the frame
            cv2.putText(annotated_frame, f"X Distance: {x_distance:.2f}px",
                        (largest_ball_center_x, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Show the annotated frame
        cv2.imshow('YOLOv8 Predictions', annotated_frame)

        # Wait for a key press to close the image window
        key = cv2.waitKey(1)
        if key == ord('q'):  # Press 'q' to quit the loop
            break

        # Ask the user for the distance from the robot to the tennis ball
        distance = input("Enter the distance from the robot to the tennis ball (or type 'cancel' to exit): ")
        if distance.lower() == 'cancel':
            break

        try:
            distance = float(distance)
            results_list.append([distance, largest_height])
        except ValueError:
            print("Invalid distance. Please enter a numeric value.")

        # Ask if the user wants to take another picture
        take_another = input("Do you want to take another picture? (y/n): ")
        if take_another.lower() in ['n', 'cancel']:
            break
            
        cap.release()
    else:
        print("Error: Could not capture a frame.")


# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Save the results to a table
if results_list:
    # Convert to a DataFrame
    df = pd.DataFrame(results_list, columns=['Distance', 'Tennis_ball_height'])

    # Save to a CSV file
    df.to_csv('tennis_ball_data.csv', index=False)
    print("Data saved to tennis_ball_data.csv")

print("Program terminated.")