#!/usr/bin/env python3
import rospy
import socket
from sensor_msgs.msg import Joy
from vpa_robot_interface.msg import WheelsCmd

class JOY2TWIST:
    
    def __init__(self) -> None:
        
        self.robot_name = socket.gethostname()
        self.sub_joy = rospy.Subscriber('joy',Joy,self.joy_cb)
        self.pub_cmd = rospy.Publisher('throttle',WheelsCmd,queue_size=1)
        rospy.loginfo("%s: joystick ready",self.robot_name)
        
    def joy_cb(self,data:Joy):
        axes = data.axes
        
        left_UD     = axes[1]
        right_LR    = axes[2]
        
        msg = WheelsCmd()
        k = 0.5
        msg.throttle_left   = left_UD * (1 - k * right_LR)
        msg.throttle_right  = left_UD * (1 + k * right_LR)
        
        self.pub_cmd.publish(msg)

if __name__ == "__main__":
    
    rospy.init_node('joystick_translator') 
    T = JOY2TWIST()
    rospy.spin()       