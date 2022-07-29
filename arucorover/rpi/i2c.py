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
CMD_VELOC = 0x02
CMD_TURN = 0x03
CMD_POS = 0x01
CMD_STAT = 0x04

TIMEOUT = 30

opSuccess = False

# Sends the arduino the command to move forward or backward a certain distance
# in metes.
def command (register, value):
    message = list(bytearray(0))
    if(register <= 0x03 or register == 0xFE):
        message = list(bytearray(struct.pack("f",value))) # Float to byte array
    else:
        message = list(bytearray(value))
    try:
        smb.write_block_data(MOTOR_ADDR, register, message)
        machState = 0
        timeCnt = 0
        while(timeCnt < TIMEOUT):
            if machState is CMD_STAT or machState is CMD_VELOC:
                break
            sleep(1)
            machState = smb.read_byte_data(MOTOR_ADDR, register)
            #print("%i"%machState)
            timeCnt = timeCnt+1
        if(timeCnt >= TIMEOUT):
            print("Failed to confirm command success.");
    except IOError:
        print("Failed to communicate with the motor controller.")

def main():
    while(True):
        while(True):
            try: # Prompt user input
                register = int(input("Command Type: 0x"),16)
                cmdVal = float(input("Command Value: "))
                break
            except ValueError: # The user did not enter a float!
                print ("Invalid floating-point value!")
                continue
        command(register, cmdVal)

if __name__ == "__main__":
        main()
