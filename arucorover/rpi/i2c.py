#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 16 11:39:39 2022

Description:    i2c.py takes a floating point angle input (radians) and sends a
                corresponding code to an arduino peripheral via I2C. The resulting
                angle is displayed on an LCD display.

@author: Paul Sampson
"""
from enum import Enum
import sys
import os
import board
import time
import struct
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

from smbus import SMBus
from time import sleep

lcd_columns = 16
lcd_rows = 2

smb = SMBus(1) #initialize SMBus

MOTOR_ADDR = 0x04

# Motor command directory. values are to be used in the i2c register field:
CMD_VELOC = 0x01
CMD_TURN = 0x02
CMD_DETECT = 0x03
CMD_EXIT = 0x04
CMD_BEGIN = 0x05

# Sends the arduino the command to move forward or backward a certain distance
# in metes.
def command (register, value):
    if(register <= 0x02):
        message = list(bytearray(struct.pack("f", value))) # convert float to a list of bytes
        #print(["0x%02x" % b for b in message])
        try:
            smb.write_block_data(MOTOR_ADDR, register, message) # send the command
        except IOError:
            print("Failed to communicate with motor controller.") # i2c failure
    else:
        message = list(bytearray(value))
        try:
            smb.write_block_data(MOTOR_ADDR, register, message) # send the command
        except IOError:
            print("Failed to communicate with motor controller.") # i2c failure
    #sleep(0.1)

def demo(angle, isTape):
    # Prompt arduino to move the wheel to the specified angle:
    print("Sending command for turning %f radians:" % angle)
    command(CMD_TURN, angle)
    time.sleep(0.5)
    command(CMD_TAPE, isTape)
    time.sleep(0.5)

angle = 0
isTape = 0

def main():
    if(len(sys.argv) > 1):
        try:
            angle = float(sys.argv[1])
            distance = float(sys.argv[2])
        except ValueError:
            print("Invalid input arguments!")
        if(angle):
            demo(angle)
        time.sleep(1)
        sys.exit()
    while(True):
        while(True):
            try: # Prompt user input
                angle = float(input("Angle: "))
                isTape = int(input("isTape: "))
                break
            except ValueError: # The user did not enter a float!
                print ("Invalid floating-point value!")
                continue
        demo(angle, isTape)

if __name__ == "__main__":
        main()
