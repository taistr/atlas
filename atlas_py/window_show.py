import cv2

def main():
    # Open a connection to the webcam (0 is usually the default camera)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image.")
            break

        # Display the captured frame
        cv2.imshow('Webcam Feed', frame)

        # Wait for a key press
        key = cv2.waitKey(1) & 0xFF

        # If 'q' is pressed, exit the loop
        if key == ord('q'):
            break

        # If 'c' is pressed, save the captured image
        if key == ord('c'):
            cv2.imwrite('captured_image.jpg', frame)
            print("Image captured and saved as 'captured_image.jpg'.")

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
