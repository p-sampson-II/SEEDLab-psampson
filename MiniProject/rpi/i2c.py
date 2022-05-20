#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import board
import struct
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

from smbus import SMBus
from time import sleep

LCD_COLS = 16
LCD_ROWS = 2
MOTOR_ADDR = 0x04

CMD_WAIT = 0x00
CMD_POS = 0x01

smb = SMBus(1) # Initialize SMBus

def command (register, value):
    message = list(bytearray(0))
    if(register == CMD_POS):
        message = list(bytearray(struct.pack("f",value))) # Float to byte array
    else:
        message = list(bytearray(value))
    try:
        smb.write_block_data(MOTOR_ADDR, register, message)
        op_success = 0
        timeout = 0
        while(not op_success and timeout < 10):
            sleep(0.1)
            op_success = smb.read_byte_data(MOTOR_ADDR, register)
            timeout = timeout+1
        if(timeout >= 10):
            print("Failed to confirm successful data transfer.");
    except IOError:
        print("Failed to communicate with the motor controller.")

def demo(pos):
    print("Sending command to moving to %f radians: " % pos)
    command(CMD_POS, pos)

pos = 0

def main():
    if(len(sys.argv) > 1):
        try:
            pos = float(sys.argv[1])
        except ValueError:
            print("Invalid input arguments!")
        if(pos):
            demo(pos)
        time.sleep(1)
        sys.exit()
    while(True):
        while(True):
            try: # Prompt user input
                pos = float(input("Position: "))
                break
            except ValueError: # The user did not enter a float!
                print ("Invalid floating-point value!")
                continue
        demo(pos)

if __name__ == "__main__":
        main()
