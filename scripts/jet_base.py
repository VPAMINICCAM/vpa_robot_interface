#!/usr/bin/python3

import rospy

from ackermann_msgs.msg import AckermannDrive
from vpa_robot_interface.msg import jetracer_act
from jetracer.nvidia_racecar import NvidiaRacecar

class jetracer_driver:
    
    def __init__(self) -> None:
        self.car = NvidiaRacecar()
        
        self.car.steering_offset = 0.12
        self.car.steering = 0
        self.car.throttle = 0
        
        self.cmd_sub = rospy.Subscriber('/cmd_jetact',jetracer_act,self._cb,queue_size=1)
        
    def _cb(self,msg:jetracer_act):
        self.car.steering = msg.steering_servo
        self.car.throttle = msg.throttle
        # # positive to the left
        
        # servo_output = -(_steering_angle * 57.2958)/15
        # if servo_output > 1:
        #     servo_output = 1
        # elif servo_output < -1:
        #     servo_output = -1
        
        # self.car.steering = servo_output
        # _acc = msg.acceleration # m/s^2
        
        # # but this is rough with not close loop
        
        # if _acc > 1:
        #     _acc = 1
        # elif _acc < -1:
        #     _acc = -1
        # self.car.throttle = _acc
        
if __name__ == "__main__":
    try:
        rospy.init_node('jet_base')
        T = jetracer_driver()
        rospy.spin()
    except KeyboardInterrupt:
        print('shutdown')