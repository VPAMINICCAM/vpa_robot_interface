"""
actuators.py
Classes to control the motors and servos.

This is a reduced package from the original donkeycar library

https://github.com/autorope/donkeycar/

"""
import time

import RPi.GPIO as GPIO
import Adafruit_PCA9685
from Adafruit_GPIO import I2C
#
# pwm/duty-cycle/pulse
# - Standard RC servo pulses range from 1 millisecond (full reverse)
#   to 2 milliseconds (full forward) with 1.5 milliseconds being neutral (stopped).
# - These pulses are typically send at 50 hertz (every 20 milliseconds).
# - This means that, using the standard 50hz frequency, a 1 ms pulse
#   represents a 5% duty cycle and a 2 ms pulse represents a 10% duty cycle.
# - The important part is the length of the pulse;
#   it must be in the range of 1 ms to 2ms.
# - So this means that if a different frequency is used, then the duty cycle
#   must be adjusted in order to get the 1ms to 2ms pulse.
# - For instance, if a 60hz frequency is used, then a 1 ms pulse requires
#   a duty cycle of 0.05 * 60 / 50 = 0.06 (6%) duty cycle
# - We default the frequency of our PCA9685 to 60 hz, so pulses in
#   config are generally based on 60hz frequency and 12 bit values.
#   We use 12 bit values because the PCA9685 has 12 bit resolution.
#   So a 1 ms pulse is 0.06 * 4096 ~= 246, a neutral pulse of 0.09 duty cycle
#   is 0.09 * 4096 ~= 367 and full forward pulse of 0.12 duty cycles
#   is 0.12 * 4096 ~= 492
# - These are generalizations that are useful for understanding the underlying
#   api call arguments.  The final choice of duty-cycle/pulse length depends
#   on your hardware and perhaps your strategy (you may not want to go too fast,
#   and so you may choose is low max throttle pwm)
#

def clamp(n, min, max):
    if min > max:
        return clamp(n, max, min)

    if n < min:
        return min
    if n > max:
        return max
    return n

class PCA9685:
    ''' 
    PWM motor controler using PCA9685 boards. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=1, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        I2C.get_default_bus = busnum
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_high(self):
        self.pwm.set_pwm(self.channel, 4096, 0)

    def set_low(self):
        self.pwm.set_pwm(self.channel, 0, 4096)

    def set_duty_cycle(self, duty_cycle):
        if duty_cycle < 0 or duty_cycle > 1:

            duty_cycle = clamp(duty_cycle, 0, 1)
            
        if duty_cycle == 1:
            self.set_high()
        elif duty_cycle == 0:
            self.set_low()
        else:
            # duty cycle is fraction of the 12 bits
            pulse = int(4096 * duty_cycle)
            try:
                self.pwm.set_pwm(self.channel, 0, pulse)
            except:
                self.pwm.set_pwm(self.channel, 0, pulse)

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)
