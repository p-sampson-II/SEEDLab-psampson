import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
from matplotlib import pyplot as plt
import time

FRAME_RATE = 10
CAM_X = 64
CAM_Y = 64
UPSCALE_X = 640
UPSCALE_Y = 640

Y_RATIO = UPSCALE_Y/CAM_Y
X_RATIO = UPSCALE_X/CAM_X

def mouseHSV(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        colData = hsv[int(y/Y_RATIO),int(x/X_RATIO)]
        h = colData[0]
        s = colData[1]
        v = colData[2]
        print("Hue: %i Saturation: %i Value: %i" %(h,s,v))
        
cv.namedWindow('hsv')
cv.setMouseCallback('hsv', mouseHSV)


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

image = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)

def threadCapture(name):
    logging.info("Thread %s: begin", name)
    camera.capture(image, 'bgr')

# Main loop
while (1):
    image = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
    camera.capture(image, 'bgr')
    image = image.reshape((CAM_Y,CAM_X,3))
    cv.imshow('image', image)
