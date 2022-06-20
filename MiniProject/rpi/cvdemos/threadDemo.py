from time import sleep
import time
import numpy as np
import cv2 as cv
from picamera import PiCamera
from matplotlib import pyplot as plt
import threading
import logging

FRAME_RATE = 49
CAM_X = 64
CAM_Y = 64
UPSCALE_X = 640
UPSCALE_Y = 640

Y_RATIO = UPSCALE_Y/CAM_Y
X_RATIO = UPSCALE_X/CAM_X
thresh_blue_1 = np.array([20, 120, 140])
thresh_blue_2 = np.array([30, 255, 200])

image = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
image = image.reshape((CAM_Y,CAM_X,3))
times=[0,0]

format = "%(asctime)s.%(msecs)03d: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,datefmt="%H:%M:%S")

# Initialize the camera
camera = PiCamera(resolution=(CAM_X,CAM_Y), framerate = FRAME_RATE, sensor_mode=5)
camera.vflip = True
camera.hflip = True


# Adjust for light exposure:
sleep(2)
PiCamera.shutter_speed = PiCamera.exposure_speed
PiCamera.exposure_mode = 'off'
g = PiCamera.awb_gains
PiCamera.awb_mode = 'off'
PiCamera.awb_gains = g

def detectBlue(image):
        image.reshape((CAM_Y,CAM_X,3))
        hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
        mask = cv.inRange(hsv, thresh_blue_1, thresh_blue_2)
        ret,thresh = cv.threshold(mask,20,255,cv.THRESH_BINARY)
        cv.imshow('thresh', thresh)
        marker = np.argwhere(thresh)
        return marker
        
def centerOfMass(marker):
    for pixel in marker:
        pixel_x_avg += pixel[1]
        pixel_y_avg += pixel[0]
    if(marker.shape[0]):
        pixel_x_avg /= pixelct
        pixel_y_avg /= pixelct
    else:
        pixel_x_avg = 0
        pixel_y_avg = 0
    return(pixel_x_avg, pixel_y_avg)

def identifyQuadrant(marker, pixel_x_avg, pixel_y_avg):
    if marker.shape[0] < 20:
        quadrant = 5
    elif pixel_x_avg <= (CAM_X/2): # If the average location is on the left
        if pixel_y_avg <= (CAM_Y/2): # If the average location is on the top
            quadrant = 2 # The marker is in the second quadrant
            #i2c.command(1,np.pi/4)
        else: # Average location is on the bottom
            quadrant = 3 # The marker is in the third quadrant
            #i2c.command(1,np.pi/2)
    else: # The average location is on the right
        if pixel_y_avg <= (CAM_Y/2): # The average location is on top
            quadrant = 1 # First Quadrant
            #i2c.command(1,0)
        else: # The average location is on the bottom
            quadrant = 4 # Fourth Quadrant
            #i2c.command(1,(3*np.pi/4))
    return quadrant

def threadProcImg():
    marker = detectBlue(image)
    xAvg,yAvg = centerOfMass(marker)
    quad = identifyQuadrant(marker, xAvg,yAvg)
    logging.info("%i"%quad)

def threadImgCapture():
    camera.capture(image, 'bgr')

if __name__ == "__main__":
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,datefmt="%H:%M:%S")
    proc = threading.Thread(target=threadProcImg, args=(), daemon = True)
    grab = threading.Thread(target=threadImgCapture, args=(), daemon = True)
    proc.start()
    grab.start()
    while(1):
        times[1] = time.time()
        dt = times[1]-times[0]
        times[0] = times[1]
        #logging.info("%f"% dt)
        proc.join()
        grab.join()
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
        
    logging.info("Main thread: Goodbye.")
    cv.destroyAllWindows()
