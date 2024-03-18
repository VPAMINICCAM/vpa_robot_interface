#!/usr/bin/python3

import rospy

from ackermann_msgs.msg import AckermannDrive

from jetracer.nvidia_racecar import NvidiaRacecar

class jetracer_driver:
    
    def __init__(self) -> None:
        self.car = NvidiaRacecar()
        
        self.car.steering_offset = 0.12
        self.car.steering = 0
        self.car.throttle = 0
        
        self.cmd_sub = rospy.Subscriber('/cmd_ackermann',AckermannDrive,self._cb)
        
    def _cb(self,msg:AckermannDrive):
        
        _steering_angle = msg.steering_angle #(rad)
        
        # positive to the left
        
        servo_output = -(_steering_angle * 57.2958)/15
        if servo_output > 1:
            servo_output = 1
        elif servo_output < -1:
            servo_output = -1
            
        self.car.steering = servo_output
        
        _acc = msg.acceleration # m/s^2
        
        # but this is rough with not close loop
        
        if _acc > 1:
            _acc = 1
        elif _acc < -1:
            _acc = -1
        self.car.throttle = _acc
        
if __name__ == "__main__":
    try:
        rospy.init_node('jet_base')
        T = jetracer_driver()
    except KeyboardInterrupt:
        print('shutdown')