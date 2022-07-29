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

FRAME_RATE = 8
CAM_X = 640
CAM_Y = 640
UPSCALE_X = 640
UPSCALE_Y = 640

Y_RATIO = UPSCALE_Y/CAM_Y
X_RATIO = UPSCALE_X/CAM_X

class ArucoCam():
    frame = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)

    def camInit():
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
            Global.status = 1
            return

    def imgCapture():
        newFrame = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
        camera.capture(newFrame, 'bgr')
        frame = newFrame.reshape((CAM_Y,CAM_X,3))

    def imgProcess():
        myAruco = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, myAruco, parameters=params)

        if len(corners) > 0:

            ids = ids.flatten()

            for( markerCorner, markerID) in zip(corners,ids):
                corners = markerCorner.reshape((4,2))
                (tL,tR,bR,bL) = corners
                tR = (int(tR[0]), int(tR[1]))
                bR = (int(bR[0]), int(bR[1]))
                bL = (int(bL[0]), int(bL[1]))
                tL = (int(tL[0]), int(tL[1]))

                cX = int((tL[0]+bR[0])/2)
                cY = int((tL[1]+bR[1])/2)
                cv2.putText(frame,str(markerID),(tL[0],tL[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0,255,0), 2)
                if markerID == Global.arucoIndex:
                    if not Global.isMarker:
                        Global.isMarker = True
                    if cY > CAM_Y*(7/8):
                        Global.isExitBottom = True
                    else:
                        Global.isExitBottom = False
                    angle = (53/2)*(CAM_X - 2 * cX)/CAM_X
                    radians = math.radians(angle)

        else:
            Global.isExitBottom = False

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            vid.release()
            cv2.destroyAllWindows()
            Global.status = 0
            return



