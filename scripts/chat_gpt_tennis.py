import cv2
import matplotlib.pyplot as plt
import supervision as sv
from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="u84JNXWzKJ4CoR3FYCqo"
)

# Capture image from the camera
cap = cv2.VideoCapture(0)  # 0 is usually the default camera
ret, image = cap.read()  # ret will be True if the image is captured successfully
cap.release()  # Release the camera

if not ret:
    print("Failed to capture image")
    exit()

# Get the image dimensions
height, width, _ = image.shape

# Calculate the center of the image
image_center = (width // 2, height // 2)

# Use the captured image for inference
results = CLIENT.infer(image, model_id="tennis-ball-icifx/1")

# Load the results into the supervision Detections API
detections = sv.Detections.from_inference(results)

# Assuming we are interested in the first detection
if len(detections.xyxy) > 0:
    bbox = detections.xyxy[0]
    # Calculate the center of the bounding box
    bbox_center = (
        int((bbox[0] + bbox[2]) / 2),
        int((bbox[1] + bbox[3]) / 2)
    )

    # Calculate the horizontal distance between the image center and the bounding box center
    distance_x = bbox_center[0] - image_center[0]

    # Draw the bounding box
    cv2.rectangle(image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)

    # Draw a dot at the center of the image
    cv2.circle(image, image_center, radius=5, color=(0, 255, 0), thickness=-1)

    # Draw a dot at the center of the bounding box
    cv2.circle(image, bbox_center, radius=5, color=(255, 0, 0), thickness=-1)

    # Annotate the image with the x-distance information
    label = f"X Distance: {distance_x} pixels"
    cv2.putText(image, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

# Convert the image from BGR (OpenCV format) to RGB (Matplotlib format)
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Display the image using Matplotlib
plt.imshow(image_rgb)
plt.axis('off')  # Hide the axes
plt.show()
