import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
from matplotlib import pyplot as plt
import tkinter

FRAME_RATE = 8
CAM_X = 300
CAM_Y = 300

# Initialize the camera
camera = PiCamera(resolution=(CAM_X,CAM_Y), framerate = FRAME_RATE)
camera.vflip = True
camera.hflip = True
#camera.start_preview()
#camera.stop_preview()

# Adjust for light exposure:
sleep(2)
PiCamera.shutter_speed = PiCamera.exposure_speed
PiCamera.exposure_mode = 'off'
g = PiCamera.awb_gains
PiCamera.awb_mode = 'off'
PiCamera.awb_gains = g

cptr = cv.VideoCapture(0)

filename = input("Enter File Name: ")
filename += '.jpg'
camera.capture(filename)
image = cv.imread(filename, cv.IMREAD_GRAYSCALE)
resized = cv.resize(image, None, fx=0.2, fy =0.2, interpolation = cv.INTER_AREA)
plt.imshow(resized, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])
plt.show()
cv.destroyAllWindows()
