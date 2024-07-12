#!/usr/bin/python

import rospy
import socket

from adafruits_driver.adafruit_servoe_kit import PWM

from vpa_robot_interface.msg import JetServo
from std_msgs.msg import Bool
''' 
    Reference comments from donkeycar lib

'''

STEERING_LEFT_PWM       = 450
STEERING_RIGHT_PWM      = 350

THROTTLE_FORWARD_PWM    = 500
THROTTLE_STOPPED_PWM    = 370
THROTTLE_REVERSE_PWM    = 220

THROTTLE_CHN = 1
STEERING_CNN = 0


class JetRacerActuator:

    def __init__(self):
        self.robot_name = socket.gethostname()
        self.pwm = PWM()             # call this modified version of PWM

        self.pwm.setPWMFreq(60)      # set operating PWM frequency
        rospy.on_shutdown(self.shut_hook)
        
        self.local_brake    = True
        self.global_brake   = True
        
        self.sub_cmd = rospy.Subscriber("actuator_cmd",JetServo,self.actuator_cb)
        self.sub_local_brk  = rospy.Subscriber("local_brake",Bool,self.local_brk_cb)
        self.sub_global_brk = rospy.Subscriber("/global_brake",Bool,self.global_brk_cb)
        rospy.loginfo('%s: traction node ready',self.robot_name)

        self.set_idle()

        rospy.loginfo('%s: steering set forward and throttle set idle',self.robot_name)

    def local_brk_cb(self,msg):
        
        if not self.local_brake == msg.data:
            self.local_brake = msg.data
            rospy.loginfo('%s: local brake status %s',self.robot_name,self.local_brake)
        
    def global_brk_cb(self,msg):
        
        if not self.global_brake == msg.data:
            self.global_brake = msg.data
            rospy.loginfo('%s: global brake status %s',self.robot_name,self.global_brake)
        
    def set_idle(self):
        self.pwm.setPWM(STEERING_CNN,0,int((STEERING_LEFT_PWM+STEERING_RIGHT_PWM)/2))
        self.pwm.setPWM(THROTTLE_CHN,0,THROTTLE_STOPPED_PWM)

    def actuator_cb(self,msg):

        throttle_ratio = msg.throttle
        steer_ratio    = msg.steering_servo

        if self.local_brake or self.global_brake:
            self.set_idle()
            return None
       
        steer_pwm      = int((STEERING_LEFT_PWM+STEERING_RIGHT_PWM)/2)

        if throttle_ratio == 0:
            throttle_pwm   = THROTTLE_STOPPED_PWM
        elif throttle_ratio > 0:
            throttle_pwm = int((THROTTLE_FORWARD_PWM - THROTTLE_STOPPED_PWM) * throttle_ratio) + THROTTLE_STOPPED_PWM
        else:
            throttle_pwm = int((THROTTLE_STOPPED_PWM - THROTTLE_REVERSE_PWM) * throttle_ratio) + THROTTLE_REVERSE_PWM

        self.pwm.setPWM(THROTTLE_CHN,0,throttle_pwm)

        steer_gain = (STEERING_RIGHT_PWM - STEERING_LEFT_PWM)/2
        steer_pwm  = int(steer_pwm + (steer_gain * steer_ratio))

        self.pwm.setPWM(STEERING_CNN,0,steer_pwm)
    
    def shut_hook(self):

        self.set_idle()
        rospy.loginfo('%s: shutdown, reset actuators',self.robot_name)

if __name__ == "__main__":
    rospy.init_node('Actutaor')
    T = JetRacerActuator()
    rospy.spin()