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
        # Params
        self.vf_ref      = rospy.get_param('~vf_ref', 0.30)
        self.Kw          = rospy.get_param('~Kw', 1.5)
        self.stop_radius = rospy.get_param('~stop_radius', 0.05)
        self.w_max       = rospy.get_param('~w_max', 3.0)
        self.ticks_per_rev = rospy.get_param('~ticks_per_rev', 135)
        self.wheel_radius  = rospy.get_param('~wheel_radius', 0.0318)
        self.wheel_base    = rospy.get_param('~wheel_base', 0.10)

        # State
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.target_pose = Pose2D()

        # Derived
        self.tick_to_meter = 2 * pi * self.wheel_radius / self.ticks_per_rev

        # IO
        rospy.Subscriber("target_pose", Pose2D, self.target_callback, queue_size=1)
        rospy.Subscriber("wheels_encoder", WheelsEncoder, self.encoder_cb, queue_size=50)  # <-- ensure topic/msg match
        self.cmd_pub  = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("dead_reckoned_pose", Pose2D, queue_size=1)

        rospy.loginfo("%s: PoseToTwist initialized", robot_name)

    def target_callback(self, msg: Pose2D):
        self.target_pose = msg

    def encoder_cb(self, msg: WheelsEncoder):
        curr_left_ticks  = msg.left_ticks
        curr_right_ticks = msg.right_ticks

        if self.last_left_ticks is None:
            self.last_left_ticks = curr_left_ticks
            self.last_right_ticks = curr_right_ticks
            return

        dlt = (curr_left_ticks  - self.last_left_ticks)  * self.tick_to_meter
        drt = (curr_right_ticks - self.last_right_ticks) * self.tick_to_meter
        self.last_left_ticks  = curr_left_ticks
        self.last_right_ticks = curr_right_ticks

        d_center = 0.5 * (dlt + drt)
        d_theta  = (drt - dlt) / self.wheel_base

        # Update pose (first-order exact)
        self.x     += d_center * cos(self.theta + 0.5 * d_theta)
        self.y     += d_center * sin(self.theta + 0.5 * d_theta)
        self.theta += d_theta
        # normalize heading
        self.theta = atan2(sin(self.theta), cos(self.theta))

        # publish pose
        pose_msg = Pose2D(x=self.x, y=self.y, theta=self.theta)
        self.pose_pub.publish(pose_msg)

        # compute and publish cmd
        v_ref, w_ref, _ = self.compute_twist(self.x, self.y, self.theta,
                                             self.target_pose.x, self.target_pose.y)
        cmd = Twist()
        cmd.linear.x  = v_ref
        cmd.angular.z = w_ref
        self.cmd_pub.publish(cmd)

    def compute_twist(self, x_meas, y_meas, theta_meas, x_ref, y_ref):
        dx = x_ref - x_meas
        dy = y_ref - y_meas
        distance = sqrt(dx*dx + dy*dy)

        angle_to_target = atan2(dy, dx)
        angle_error = atan2(sin(angle_to_target - theta_meas), cos(angle_to_target - theta_meas))

        if distance < self.stop_radius:
            return 0.0, 0.0, distance

        # Optional: taper linear speed near goal for smoother stop
        v = self.vf_ref * min(1.0, distance / (3.0 * self.stop_radius))
        w = max(-self.w_max, min(self.w_max, self.Kw * angle_error))
        return v, w, distance

if __name__ == '__main__':
    try:
        PoseToTwistNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
