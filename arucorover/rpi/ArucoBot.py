from i2c import command
import threading
from Rover import VirtualRover
from ArucoDetect import ArucoCam
import Global
import logging
import time

def roverLoopUnthreaded():
    while(Global.status == 2):
        myCam.imgCapture()
        myCam.imgProcess()
        rov.next()
        print(Global.isExitBottom)
        #command(Global.cmd[0],Global.cmd[1])

def roverLoop():
    while(Global.status == 2):
        threadCpt = threading.Thread(target=myCam.imgCapture())
        threadProc = threading.Thread(target=myCam.imgProcess())
        threadRov = threading.Thread(target=rov.next())
        threadCpt.start()
        threadProc.start()
        threadRov.start()
        threadCpt.join()
        threadProc.join()
        threadRov.join()
        print(Global.isExitBottom)
        command(Global.cmd[0],Global.cmd[1])

if __name__ == "__main__":
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format,level=logging.INFO, datefmt="%H:%M:%S")
    myCam = ArucoCam()
    myCam.camInit()
    rov = VirtualRover()
    roverLoop()
