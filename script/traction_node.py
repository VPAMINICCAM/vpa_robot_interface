#!/usr/bin/python3

import rospy
import socket
from math import fabs, floor
from adafruit_drivers.Adafruit_PWM_Servo_Driver import PWM

from vpa_robot_interface.msg import DirectCmd

''' 
    Reference comments from donkeycar lib

'''
STEERING_LEFT_PWM       = 460
STEERING_RIGHT_PWM      = 290

THROTTLE_FORWARD_PWM    = 500
THROTTLE_STOPPED_PWM    = 370
THROTTLE_REVERSE_PWM    = 220

THROTTLE_CHN = 1
STEERING_CNN = 0

class PiRacerActutaor:

    def __init__(self) -> None:
        self.robot_name = socket.gethostname()
        self.pwm = PWM()             # call this modified version of PWM
        self.pwm.setPWMFreq(60)      # set oerating PWM frequency

        self.sub_cmd = rospy.rospy.Subscriber("actuator_cmd",DirectCmd,self.actuator_cb)
        rospy.loginfo('%s: traction node ready',self.robot_name)

    def actuator_cb(self,msg:DirectCmd):

        throttle_ratio = msg.throttle
        steer_ratio    = msg.steering

       
        steer_pwm      = int((STEERING_LEFT_PWM+STEERING_RIGHT_PWM)/2)

        if throttle_ratio == 0:
            throttle_pwm   = THROTTLE_STOPPED_PWM
        elif throttle_ratio > 0:
            throttle_pwm = int((THROTTLE_FORWARD_PWM - THROTTLE_STOPPED_PWM) * throttle_ratio) + THROTTLE_STOPPED_PWM
        else:
            throttle_pwm = int((THROTTLE_STOPPED_PWM - THROTTLE_REVERSE_PWM) * throttle_ratio) + THROTTLE_REVERSE_PWM

        self.pwm.setPWM(THROTTLE_CHN,throttle_pwm)

        steer_gain = (STEERING_RIGHT_PWM - STEERING_LEFT_PWM)/2
        steer_pwm  = int(steer_pwm + (steer_gain * steer_ratio))

        self.pwm.setPWM(STEERING_CNN,steer_pwm)
        
if __name__ == "__main__":
    rospy.init_node('Actutaor')
    T = PiRacerActutaor()
    rospy.spin()