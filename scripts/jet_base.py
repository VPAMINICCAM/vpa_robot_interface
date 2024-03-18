#!/usr/bin/python3

import rospy

# from ackermann_msgs.msg import AckermannDrive
from vpa_robot_interface.msg import JetServo
#from std_msgs.msg import Float32
from jetracer.nvidia_racecar import NvidiaRacecar

car = NvidiaRacecar()
car.steering_offset = 0.12

def callback(data):
    car.steering = data.steering_servo
    car.throttle = data.throttle
    # print('steering:',car.steering,'throttle:',car.throttle)
    
def listener():
    rospy.init_node('tutorials_msg_sub', anonymous=False)
    rospy.Subscriber("cmd", JetServo, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
