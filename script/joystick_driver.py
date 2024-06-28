#!/usr/bin/env python3
import rospy
import socket
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from vpa_robot_interface.msg import DirectCmd

class JOYDRIVER:
    
    def __init__(self) -> None:
        
        self.robot_name = socket.gethostname()
        self.local_unlock_flag = False
        self.latch_flag        = False
        self.timer = None
        self.sub_joy = rospy.Subscriber('joy',Joy,self.joy_cb)
        self.pub_brk = rospy.Publisher('local_brake',Bool,queue_size=1)
        self.pub_cmd = rospy.Publisher('actuator_cmd',DirectCmd,queue_size=1)
        self.throttle_upper = rospy.get_param('~gas_max',0.3)
        rospy.loginfo("%s: joystick ready",self.robot_name)
    
    def joy_cb(self,data:Joy):
        axes = data.axes
        buts = data.buttons
                
        LB = buts[6]
        RB = buts[7]
        
        if LB == 1 and RB == 1:
            msg = Bool()
            if not self.latch_flag:
                self.latch_flag = True
                self.timer = rospy.Timer(rospy.Duration(0.5), self.latch_lock, oneshot=True)
                if self.local_unlock_flag:
                    # has been unlock
                    msg.data = True # lock the wheels
                    self.local_unlock_flag = False
                else:
                    msg.data = False
                    self.local_unlock_flag = True
                self.pub_brk.publish(msg)
                
        left_UD     = axes[1]
        right_LR    = axes[2]
        
        msg = DirectCmd()
        msg.steering = -right_LR
        msg.throttle = self.bound_output(left_UD)
        
        self.pub_cmd.publish(msg)
        
    def bound_output(self,input) -> float:
        if input > self.throttle_upper:
            return self.throttle_upper
        elif input < -0.5:
            return -0.5
        return input
    
    def latch_lock(self,event):
        self.latch_flag = False

if __name__ == "__main__":
    
    rospy.init_node('joystick_translator') 
    T = JOYDRIVER()
    rospy.spin()