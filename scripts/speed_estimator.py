#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

'''
This node is supposed to perform an estimation on speed based on the IMU readings.

Currently, this IMU is a N200 high-end IMU with internal filtering, which delivers quite good static results.

However, there is a fan and a rotating lidar on the robot, causing constant vibrations, thus filtering is required.
'''

class VelocityEst:

    def __init__(self):
        # Initialize the node
        rospy.init_node('velocity_estimation', anonymous=True)
        
        # Subscriber to IMU data
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        
        # Publisher for estimated velocity
        self.velocity_pub = rospy.Publisher('estimated_velocity', Float32, queue_size=10)
        
        # Placeholder for storing velocity
        self.velocity = 0.0
        
        # Placeholder for previous time
        self.prev_time = None
        
        # Placeholder for previous linear acceleration
        self.prev_linear_acceleration = None

        # Low-pass filter parameter (between 0 and 1)
        self.alpha = 0.1  # Adjust this value to change the filtering level

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_linear_acceleration = data.linear_acceleration
            return
        
        dt = (current_time - self.prev_time).to_sec()
        linear_acceleration = data.linear_acceleration
        
        # Simple velocity estimation using trapezoidal integration
        raw_velocity = self.velocity + 0.5 * (linear_acceleration.x + self.prev_linear_acceleration.x) * dt
        
        # Apply low-pass filter
        self.velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.velocity
        
        # Publish the estimated velocity
        self.velocity_pub.publish(self.velocity)
        
        # Update previous values
        self.prev_time = current_time
        self.prev_linear_acceleration = linear_acceleration

if __name__ == '__main__':
    try:
        ve = VelocityEst()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
