import sys
#from cv_bridge import CvBridge
from ultralytics import YOLO
import pathlib
import torch
import cv2

# Dependencies
from atlas_camera import take_image

# Object dectection params
MODEL_DIRECTORY_PATH = pathlib.Path("./obj_detection_models/26_08_640px.pt")
FRAME_WIDTH = 640 # change later
FRAME_HEIGHT = 480



class ObjectDetection():
    def __init__(self):        
        self.model = YOLO(str(MODEL_DIRECTORY_PATH))

        # self.bridge = CvBridge()

        self.latest_image = None
        self.response = {
            "detection" : False,
            "angle" : float(0),
            "distance" : float(0)
        }

        print("object_detection.py:I know what I'm seeing now!")
        

    def detect_object(self):
        """Detect objects in the received image"""
        frame_center_x = FRAME_WIDTH // 2
        frame_center_y = FRAME_HEIGHT // 2

        self.latest_image = take_image()
        # Run inference on latest image
        results = self.model(self.latest_image, conf=0.7)

        #TODO: Populate response with detected objects
        boxes = results[0].boxes

        if boxes.cls.tolist(): #If an object was detected
            # Find the most confident object
            _, max_index = torch.max(boxes.conf, dim=0)

            # Extract the xyxy coordinates of the most confident object
            x_centre, y_centre, width, height = boxes.xywh[max_index].tolist()

            # TODO: Calculate the heading offset
            slope_angle=0.0632085093204184
            intercept_angle=0.7163631056314085
            x_distance = x_centre - frame_center_x

            # TODO: Calculate the distance to the object (hint: use a dictionary)
            slope_distance=50.924376407217764
            intercept_distance=0.1332096544887631


            self.response["detection"] = True
            self.response["angle"] = slope_angle*x_distance+intercept_angle
            self.response["distance"] = slope_distance/height+intercept_distance
        else:
            self.response["detection"] = False
        return self.response


def object_detection():    
    object_detection = ObjectDetection()
    try:
        response = object_detection.detect_object()
        return response
    except:
        print("object_detection.py:Object Detection Failed")
        sys.exit(1)

