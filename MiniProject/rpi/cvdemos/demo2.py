import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
from matplotlib import pyplot as plt
import numpy as np

FRAME_RATE = 8
CAM_X = 64
CAM_Y = 64

# Initialize the camera
camera = PiCamera(resolution=(CAM_X,CAM_Y), framerate = FRAME_RATE)
camera.vflip = True
camera.hflip = True


# Adjust for light exposure:
sleep(2)
PiCamera.shutter_speed = PiCamera.exposure_speed
PiCamera.exposure_mode = 'off'
g = PiCamera.awb_gains
PiCamera.awb_mode = 'off'
PiCamera.awb_gains = g

# Main loop
while (1):
    image = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
    camera.capture(image, 'bgr')
    image = image.reshape((CAM_Y,CAM_X,3))

    # BGR -> HSV
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Color ranges:
    thresh_blue_1 = np.array([100, 40, 10])
    thresh_blue_2 = np.array([150, 130, 200])

    # Apply the thresholds above to get a mask of only blues:
    mask = cv.inRange(hsv, thresh_blue_1, thresh_blue_2)

    # AND the mask with the original image to get the blue parts of
    # the image only:
    res = cv.bitwise_and(image,image, mask = mask)
    
    # Display images for comparison
    cv.imshow('hsv',hsv)
    cv.imshow('res', res)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()
