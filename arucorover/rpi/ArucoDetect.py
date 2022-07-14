#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import sys
import numpy as np
import math
import time
import i2c
#import imutils

from picamera import PiCamera
from time import sleep

FRAME_RATE = 49
CAM_X = 640
CAM_Y = 640
UPSCALE_X = 640
UPSCALE_Y = 640

Y_RATIO = UPSCALE_Y/CAM_Y
X_RATIO = UPSCALE_X/CAM_X

def immean(mask):
    marker = np.argwhere(mask) # The pixels where the threshold for the particular shade of blue we have chosen is present
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

    return pixel_x_avg, pixel_y_avg

# Initialize the camera
camera = PiCamera(resolution=(CAM_X,CAM_Y), framerate = FRAME_RATE, sensor_mode=5)

sleep(2)
PiCamera.shutter_speed= PiCamera.exposure_speed
PiCamera.exposure_mode = 'off'
g = PiCamera.awb_gains
PiCamera.awb_mode = 'off'
PiCamera.awb_gains = g

#Start live video
vid = cv2.VideoCapture(0)

#Checks to see if camera is working
if not vid.isOpened():
    print("Cannot open camera")
    exit()

detected = False
begin = True
arucoExit = False

i2c.command(i2c.CMD_BEGIN, begin)
i2c.command(i2c.CMD_TURN, math.pi/8)

while(True):
    frame = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
    camera.capture(frame, 'bgr')
    frame = frame.reshape((CAM_Y,CAM_X,3))
    location = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
    location = location.reshape((CAM_Y,CAM_X,3))
    
    myAruco = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, myAruco, parameters=params)
    
    if len(corners) > 0:
        if not detected:
            detected = True
            i2c.command(i2c.CMD_DETECT, detected)
        ids = ids.flatten()
        
        for( markerCorner, markerID) in zip(corners,ids):
            corners = markerCorner.reshape((4,2))
            (tL,tR,bR,bL) = corners
            tR = (int(tR[0]), int(tR[1]))
            bR = (int(bR[0]), int(bR[1]))
            bL = (int(bL[0]), int(bL[1]))
            tL = (int(tL[0]), int(tL[1]))
            
            #cv2.line(frame, tL, tR, (0,255,0),2)
            #cv2.line(frame, tR, bR, (0,255,0),2)
            #cv2.line(frame, bR, bL, (0,255,0),2)
            #cv2.line(frame, bL, tL, (0,255,0),2)
            
            cX = int((tL[0]+bR[0])/2)
            cY = int((tL[1]+bR[1])/2)
            cv2.circle(location,(cX,cY), 4, (255,255,255), -1)
            cv2.circle(frame,(cX,cY), 4, (0,0,255), -1)
            
            cv2.putText(frame,str(markerID),(tL[0],tL[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0,255,0), 2)

            loc = immean(location)
            if loc[1] > CAM_Y*(7/8):
                arucoExit = True
                i2c.command(i2c.CMD_EXIT, arucoExit)
            else:
                arucoExit = False
            angle = (53/2)*(CAM_X - 2 * loc[0])/CAM_X
            radians = math.radians(angle)
            i2c.command(i2c.CMD_TURN, radians)
    else:
        detected = False
        i2c.command(i2c.CMD_TURN, math.pi/8)
        
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
vid.release()
cv2.destroyAllWindows()
