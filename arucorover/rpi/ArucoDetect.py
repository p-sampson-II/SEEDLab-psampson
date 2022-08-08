#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import sys
import numpy as np
import math
import time
import i2c
import Global
#import imutils

from picamera import PiCamera
from time import sleep

FRAME_RATE = 8
CAM_X = 1024
CAM_Y = 1008
UPSCALE_X = 640
UPSCALE_Y = 640

ANGLE_TOLERANCE = 0.05

Y_RATIO = UPSCALE_Y/CAM_Y
X_RATIO = UPSCALE_X/CAM_X

class ArucoCam():
    frame = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)

    def camInit(self):
        # Initialize the camera
        self.camera = PiCamera(resolution=(CAM_X,CAM_Y), framerate = FRAME_RATE, sensor_mode=5)

        sleep(2)
        PiCamera.shutter_speed= PiCamera.exposure_speed
        PiCamera.exposure_mode = 'off'
        g = PiCamera.awb_gains
        PiCamera.awb_mode = 'off'
        PiCamera.awb_gains = g

        #Start live video
        self.vid = cv2.VideoCapture(0)

        #Checks to see if camera is working
        if not self.vid.isOpened():
            print("Cannot open camera")
            Global.status = 1
            return

    def imgCapture(self):
        newFrame = np.empty((CAM_Y * CAM_X * 3,), dtype = np.uint8)
        self.camera.capture(newFrame, 'bgr')
        self.frame = newFrame.reshape((CAM_Y,CAM_X,3))

    def imgProcess(self):
        myAruco = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(self.frame, myAruco, parameters=params)

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
                cv2.putText(self.frame,str(markerID),(tL[0],tL[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0,255,0), 2)
                if markerID == Global.arucoIndex:
                    if not Global.isMarker:
                        Global.isMarker = True
                    if cY > CAM_Y*(3/4):
                        Global.isExitBottom = True
                    else:
                        Global.isExitBottom = False
                    angle = (53/2)*(CAM_X - 2 * cX)/CAM_X
                    Global.camAngle = -math.radians(angle)
                    if abs(Global.camAngle) < ANGLE_TOLERANCE:
                        Global.isCentered = True
                    else:
                        Global.isCentered = False
                else:
                    Global.isMarker = False

        else:
            Global.isExitBottom = False
            Global.isMarker = False

        cv2.imshow('frame', self.frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            camera.release()
            cv2.destroyAllWindows()
            Global.status = 0
            return



