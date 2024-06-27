#!/usr/bin/python3

import rospy
import socket

from donkey.actutaor import PCA9685

from donkey import STEERING_CHN,THROTTLE_CHN,BUSNUM,I2C_ADDR
from vpa_robot_interface.msg import DirectCmd

STEERING_LEFT_PWM       = 460
STEERING_RIGHT_PWM      = 290

THROTTLE_FORWARD_PWM    = 500
THROTTLE_STOPPED_PWM    = 370
THROTTLE_REVERSE_PWM    = 220

class PiRacerActutaor:

    def __init__(self) -> None:

        self.robot_name = socket.gethostname()
        self.steer_controller    = PCA9685(STEERING_CHN,I2C_ADDR)
        # steering 
        
        self.throttle_controller = PCA9685(THROTTLE_CHN,I2C_ADDR)
        # throttle

        self.cmd_sub = rospy.Subscriber('movement',DirectCmd,self.move_cb)
        rospy.loginfo('%s: actutaors node activated',self.robot_name)

    def move_cb(self,data:DirectCmd):

        throttle_cmd = data.throttle
        steer_cmd    = data.steering

        # both of these values will be a value [-1,1]
        dutycycle_throttle = self.ratio2throttleDutyCycle(throttle_cmd)
        dutycycle_steer    = self.ratio2steerDutyCycle(steer_cmd)

        # convert information to wheels
        self.steer_controller.set_duty_cycle(dutycycle_steer)
        self.throttle_controller.set_duty_cycle(dutycycle_throttle)

    def ratio2steerDutyCycle(self,ratio:float) -> int:

        # define positive: left
        k = (STEERING_RIGHT_PWM - STEERING_LEFT_PWM)/2
        return (ratio + 1) * k + STEERING_LEFT_PWM
    
    def ratio2throttleDutyCycle(self,ratio:float) -> int:
        if ratio == 0:
            return THROTTLE_STOPPED_PWM
        elif ratio > 0:
            k = THROTTLE_FORWARD_PWM - THROTTLE_STOPPED_PWM
            return THROTTLE_STOPPED_PWM + k * ratio
        else:
            k = THROTTLE_STOPPED_PWM - THROTTLE_REVERSE_PWM
            return THROTTLE_REVERSE_PWM + k * ratio

if __name__ == "__main__":
    rospy.init_node('Actutaor')
    T = PiRacerActutaor()
    rospy.spin()