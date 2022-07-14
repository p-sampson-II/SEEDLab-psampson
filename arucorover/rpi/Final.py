#Aaron Hsu
#FinalDemo

#All of the imported modules
import numpy as np
import cv2 as cv
import math
import i2c
import os
import time

from picamera import PiCamera
from time import sleep

#Victory Theme
cmd = 'mpg321 ./StarWarsTheme.mp3'

#CrossReached = 0

#Auto White Balance
sleep(2)
PiCamera.shutter_speed= PiCamera.exposure_speed
PiCamera.exposure_mode = 'off'
g = PiCamera.awb_gains
PiCamera.awb_mode = 'off'
PiCamera.awb_gains = g

#Pi Camera Resolution
PiCamera.resolution = (100,100)
PiCamera.framerate = 24

#Start live video
cap = cv.VideoCapture(0)

#Checks to see if camera is working
if not cap.isOpened():
    print("Cannot open camera")
    exit()

#Variables for Turn State
Turn = 0
Turn_old = 0

#Variables for timer to send i2c
timer = 0
timer_old = 0

#Just a holder
Stop = 0

# Send the Arduino a message that it's time to start:
i2c.command(i2c.CMD_TURN, float(3.14159/8))

#While Loop to read each frame
while True:
    ret, img = cap.read() #Reads Frame

    #If frame cannot be read
    if not ret:
        print("Cannot read frame")
        break
    #Scales frame
    width = int(img.shape[1]/1.5)
    height = int(img.shape[0]/1.5)
    new = (width, height)
    img = cv.resize(img, new) #New resized frame (smaller)
    #Converts frame from BGR format to HSV
    hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    #Uppper and Lower bounds for color detection
    lower = np.array([75,150,0])
    upper = np.array([150,255,255])
    mask = cv.inRange(hsv, lower, upper) #Mask for color detection
    #Frame to display only yellow objects and black everywhere else
    res = cv.bitwise_and(img, img, mask=mask)
    #Kernel for the filters
    kernel = np.ones((5,5),np.uint8)
    kernel1= cv.getStructuringElement(cv.MORPH_RECT,(3,3))
    #Saving and then reading frame in grayscale
    img2 = cv.imwrite('/home/pi/Desktop/Demo.jpg', res)
    img2 = cv.imread('/home/pi/Desktop/Demo.jpg',0)
    #For cross detection
    edge = cv.Canny(img2, 90, 130)
    #Uses 'opening' and 'closing' effect on frame
    img3 = cv.morphologyEx(img2, cv.MORPH_OPEN, kernel)
    img3 = cv.morphologyEx(img3, cv.MORPH_CLOSE, kernel)
    new = cv.morphologyEx(edge, cv.MORPH_CLOSE, kernel1)

    #Threshold to make frame binary
    ret,thresh1 = cv.threshold(img3,1,255,cv.THRESH_BINARY)
    nonzero = cv.findNonZero(thresh1)
    
    #Takes the mean of the array for coordinates
    xy = cv.mean(nonzero)
    #Equations to find angle
    angle = 53 / 2
    w = width / 2
    w1 = w - xy[0]
    ratio = w1 / w
    angle = angle * ratio
    radians = math.radians(angle)
    #print(angle)
    
    #Cross Detection for the end
    if xy[0] == 0 and xy[1] == 0:
        Stop = 0
    else:
        contours = cv.findContours(new, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        if contours:
            finalContours = max(contours, key=cv.contourArea)
            contour = img.copy()
            cv.drawContours(contour, [finalContours], 0, (0,0,255), 1)
            peri = cv.arcLength(finalContours, True)
            approx = cv.approxPolyDP(finalContours, 0.03 * peri, True)
            #Sends stop flag
            if len(approx) > 9:
                print(len(approx))
                i2c.command(0x07,1)
                sleep(0.5)
                #if not CrossReached:
                #CrossReached = 1
                #print("Cross reached.")
                #os.system(cmd)

    #Displays live video
    cv.imshow('frame',res)
    
    #Sends flag for no tape
    if xy[0] == 0 and xy[1] == 0:
        #print("Searching...")
        Turn= 1 #Continue turning till blue tape is found
        i2c.command(i2c.CMD_TAPE, 0)

    #Sends flag for tape on screen
    if abs(radians) < 0.4:
        #print("Turning Left")
        Turn = 2 #Continue turning left because tape is found
        #print("see")
        i2c.command(i2c.CMD_TAPE, 1)
    else:
        i2c.command(i2c.CMD_TAPE,0)
    
    #Gets the edge points of the tape
    if xy[0] == 0 and xy[1] == 0:
        Right = 0
        Max = 0
        Min = 0
    else:
        Y,X = np.nonzero(thresh1)
        Right = np.amax(X,0)        #Finds right edge of tape
        Max = np.amax(Y,0)          #Finds front edge of tape
        Min = np.amin(Y,0)          #Finds back edge of tape

    #Timer for sending angle
    timer = time.perf_counter()
    if timer > timer_old+0.1:
       i2c.command(0x02, radians)
       #print("Sent.")
       timer_old = timer

    #print(stop)
    #print(Turn)
    #print(Go)
    #print(Max)
    #print (Min)
    #print(radians)
    
    #To end live video
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
