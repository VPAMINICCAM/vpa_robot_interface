# -*- coding: utf-8 -*-

#!/usr/bin/python3

import serial
import time
import binascii

class ToF400F:

    def __init__(self) -> None:
        self.serial = serial.Serial('/dev/ttyAMA0',115200)
        if self.serial.isOpen() :
            print("tof usart: open success")
        else :
            print("tof usart: open failed")

    def cl(a):

        # This is a questionanle function that comes with the example code
        # mayworth take a look if it is really needed

        dat1 = a[0:1]
        if dat1 == 'a':
            dat1 = 10
        elif dat1 == 'b':
            dat1 = 11
        elif dat1 == 'c':
            dat1 = 12
        elif dat1 == 'd':
            dat1 = 13
        elif dat1 == 'e':
            dat1 = 14
        elif dat1 == 'f':
            dat1 = 15
            
        return dat1
    
    def get_distance(self) -> int:

        num = self.serial.inWaiting()

        if num:
            try: 
                interface_data = self.serial.read(num)
                data = str(binascii.b2a_hex(interface_data))
                if(len(data)>8):
                    byte1_low = data[9:10]
                    byte2_high = data[10:11]
                    byte2_low = data[11:12]
                    distance_value = 0.0 # needs to be float from beginning, otherwise runtime-conversion will take too long
                    distance_value = int(self.cl(byte2_low)) + int(self.cl(byte2_high))*16 + int(self.cl(byte1_low))*256
                    # print("distance:", distance_value, "mm")
                    return distance_value
                else:
                    return -1
            except:
                return -1