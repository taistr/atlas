from atlas_camera import Camera
import cv2
import threading

class FrameGrabber:
    def __init__(self, camera: Camera):
        self.camera = camera
        self.frame = None
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        """Start the frame grabbing thread."""
        self.running = True
        self.thread = threading.Thread(target=self._grab_frames)
        self.thread.start()

    def stop(self):
        """Stop the frame grabbing thread."""
        self.running = False
        self.thread.join()

    def _grab_frames(self):
        """Continuously grab frames from the camera in a separate thread."""
        while self.running:
            frame = self.camera.capture_image()
            if frame is not None:
                with self.lock:
                    self.frame = frame

    def get_latest_frame(self):
        """Retrieve the latest captured frame."""
        with self.lock:
            return self.frame


def main():
    # Initialize the camera and frame grabber
    cam = Camera()
    grabber = FrameGrabber(cam)
    grabber.start()

    try:
        while True:
            # Get the latest frame from the grabber
            frame = grabber.get_latest_frame()

            if frame is not None:
                # Display the frame
                cv2.imshow('Camera Test', frame)

            # Check for a key press every 100ms
            key = cv2.waitKey(100)  # Waits 100 milliseconds for a key press

            # Exit if 'q' is pressed
            if key & 0xFF == ord('q'):
                print("Exiting on 'q' press.")
                break

    except KeyboardInterrupt:
        print("\nProgram interrupted by the user (Ctrl+C).")

    finally:
        # Stop the grabber and release resources
        print("Cleaning up resources...")
        grabber.stop()
        del cam
        cv2.destroyAllWindows()
        print("Resources released. Program terminated.")


if __name__ == "__main__":
    main()
