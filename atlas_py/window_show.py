import cv2
from atlas_camera import Camera


camera = Camera()
while True:
    image = camera.capture_image()
    cv2.imshow("Image Window", image)
    cv2.waitKey(0)                           