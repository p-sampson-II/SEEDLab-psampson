import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
from matplotlib import pyplot as plt
import time
import i2c
import logging

FRAME_RATE = 49
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

image = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)

format = "%(asctime)s.%(msecs)03d: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,datefmt="%H:%M:%S")

times=[0,0]

# Main loop
while (1):
    times[1] = time.time()
    dt = times[1]-times[0]
    times[0] = times[1]
    logging.info("%f"% dt)
    image = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
    camera.capture(image, 'bgr')
    image = image.reshape((CAM_Y,CAM_X,3))
    # BGR -> HSV
    hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)

    # Color ranges:
    thresh_blue_1 = np.array([20, 120, 140])
    thresh_blue_2 = np.array([30, 255, 200])

    # Apply the thresholds above to get a mask of only blue:
    mask = cv.inRange(hsv, thresh_blue_1, thresh_blue_2)
    #cv.fastNlMeansDenoising(mask,mask,10)
    # AND the mask with the original image to get the blue parts of
    # the image only:
    #res = cv.bitwise_and(image,image, mask = mask).astype('uint8')
    #res = cv.cvtColor(res, cv.COLOR_RGB2GRAY)
    ret,thresh = cv.threshold(mask,20,255,cv.THRESH_BINARY)

    marker = np.argwhere(thresh) # The pixels where the threshold for the particular shade of blue we have chosen is present
    pixelct = marker.shape[0] # The number of pixels contained in the marker
    
    pixel_x_avg = 0 # To be the average x-location of the marker on the camera
    pixel_y_avg = 0 # To be the average y-location of the marker on the camera
    for pixel in marker: # Concatenate all of the x and y locations
        pixel_x_avg += pixel[1]
        pixel_y_avg += pixel[0]

    if pixelct: # If there is a nonzero number of pixels in the marker divide the concatenation by the number of pixels
        pixel_x_avg /= pixelct
        pixel_y_avg /= pixelct
    else: # Otherwise report the average location as (0, 0)
        pixel_x_avg = 0
        pixel_y_avg = 0

    if not(pixel_x_avg and pixel_y_avg) or pixelct < 20: # If the average location is (0, 0)
        quad = 5 # Report quadrant as unspecified
    elif pixel_x_avg <= (CAM_X/2): # If the average location is on the left
        if pixel_y_avg <= (CAM_Y/2): # If the average location is on the top
            quad = 2 # The marker is in the second quadrant
            i2c.command(1,np.pi/4)
        else: # Average location is on the bottom
            quad = 3 # The marker is in the third quadrant
            i2c.command(1,np.pi/2)
    else: # The average location is on the right
        if pixel_y_avg <= (CAM_Y/2): # The average location is on top
            quad = 1 # First Quadrant
            i2c.command(1,0)
        else: # The average location is on the bottom
            quad = 4 # Fourth Quadrant
            i2c.command(1,(3*np.pi/4))

    
    #contours, hierarchy, x = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    #if(X_RATIO != 1 and Y_RATIO != 1):
    #    image = cv.resize(image, (UPSCALE_X, UPSCALE_Y), interpolation = cv.INTER_LINEAR)
    #    thresh = cv.resize(thresh, (UPSCALE_X, UPSCALE_Y), interpolation = cv.INTER_LINEAR)
    # Display images for comparison
    #cv.imshow('hsv', image)
    cv.imshow('thresh', thresh)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()
