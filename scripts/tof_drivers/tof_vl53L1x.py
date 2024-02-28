import os
import sys
import time
import RPi.GPIO as GPIO
import VL53L1X

class ToFVL53L1X:

    def __init__(self, address, xshut=17):
        self.address = address

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.xshut = xshut
        GPIO.setup(self.xshut, GPIO.IN)

        # 1 = Short Range, 2 = Medium Range, 3 = Long Range 
        self.range = 1

        if(self.address == 0x29):
            self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address = 0x29)
        else:
            self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address = self.address)
            self.tof.open()

    def start_sensor(self, pin):
        # The XSHUT pin is set HIGH to activate the sensor.
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, True)
        time.sleep(0.2)

        self.tof.open()
        self.tof.start_ranging(self.range)

    def stop_sensor(self, pin):
        self.tof.stop_ranging()
        GPIO.output(pin, False)

    def set_range(self, range):
        if range == "short":
            self.range = 1
        elif range == "medium":
            self.range = 2
        else:
            self.range = 3

        self.tof.stop_ranging()
        self.tof.start_ranging(self.range)

    def get_range(self):
        if self.range == 1:
            currentRange = "short"
        elif self.range == 2:
            currentRange = "medium"
        else:
            currentRange = "long"

        return currentRange

    def get_distance(self):
        distance = 0.0
        distance = self.tof.get_distance() * 0.001 # mm to m conversion

        if distance >= 0:
            return distance
        else:
            return -1

    def get_speed(self):
        start_distance = self.get_distance() * 0.01     # to m conversion

        time.sleep(1)

        end_distance = self.get_distance() * 0.01       # to m conversion

        speed = (end_distance - start_distance) / 1.0   # m/s

        return speed