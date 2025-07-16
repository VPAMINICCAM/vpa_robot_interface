#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, Twist
from vpa_robot_interface.msg import WheelsEncoder
from math import pi, cos, sin, atan2, sqrt
import socket
class PoseToTwistNode:

    def __init__(self):
        rospy.init_node('pose_to_twist_node')
        robot_name = socket.gethostname()
        self.vf_ref = rospy.get_param('~vf_ref', 0.3)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
         
        self.target_pose = Pose2D()
        rospy.Subscriber("target_pose", Pose2D, self.target_callback)
        
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        self.ticks_per_rev  = 135
        self.wheel_radius   = 0.0318  # meters
        self.wheel_base     = 0.1       # meters (adjust as needed)
        self.tick_to_meter  = 2 * pi * self.wheel_radius / self.ticks_per_rev
        self.pose_pub = rospy.Publisher("dead_reckoned_pose", Pose2D, queue_size=1)

        rospy.Subscriber("wheel_omega", WheelsEncoder, self.encoder_cb)

        rospy.loginfo("%s: Pose to Twist node initialized", robot_name)

    def target_callback(self, msg: Pose2D):
        self.target_pose = msg

    def encoder_cb(self, msg: WheelsEncoder):
        curr_left_ticks = msg.left_ticks
        curr_right_ticks = msg.right_ticks

        if self.last_left_ticks is None:
            self.last_left_ticks = curr_left_ticks
            self.last_right_ticks = curr_right_ticks
            return
        
        delta_left = (curr_left_ticks - self.last_left_ticks) * self.tick_to_meter
        delta_right = (curr_right_ticks - self.last_right_ticks) * self.tick_to_meter

        self.last_left_ticks = curr_left_ticks
        self.last_right_ticks = curr_right_ticks

        d_center = 0.5 * (delta_left + delta_right)
        d_theta = (delta_right - delta_left) / self.wheel_base

        self.x += d_center * cos(self.theta + d_theta / 2.0)
        self.y += d_center * sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)

        v_ref, w_ref, _ = self.compute_twist(self.x, self.y, self.theta,self.target_pose.x, self.target_pose.y, self.vf_ref)

        cmd_msg = Twist()
        cmd_msg.linear.x = v_ref
        cmd_msg.angular.z = w_ref
        self.cmd_pub.publish(cmd_msg)

    def compute_twist(x_meas, y_meas, theta_meas, x_ref, y_ref, v_cmd):
        # Parameters
        Kw = 1.5           # angular gain
        stop_radius = 0.05 # meters
        w_max = 3.0        # rad/s clamp

        dx = x_ref - x_meas
        dy = y_ref - y_meas
        distance = sqrt(dx**2 + dy**2)

        angle_to_target = atan2(dy, dx)
        angle_error = angle_to_target - theta_meas
        angle_error = atan2(sin(angle_error), cos(angle_error))  # normalize to [-pi, pi]

        if distance < stop_radius:
            v_ref = 0.0
            w_ref = 0.0
        else:
            v_ref = v_cmd
            w_ref = Kw * angle_error
            w_ref = max(-w_max, min(w_max, w_ref))

        return v_ref, w_ref, distance

if __name__ == '__main__':
    try:
        node = PoseToTwistNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass