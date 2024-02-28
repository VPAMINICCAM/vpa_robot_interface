#!/usr/bin/python

# To the best of my knowledge, the ROS must only access the same I2C bus within the same node
# This is the wraper file for I2C devices including servo drivers and ToF Sensors

import rospy

from sensor_msgs.msg import Range
from adafruit_drivers.Adafruit_PWM_Servo_Driver import PWM
from tof_drivers.tof_vl53L1x import ToFVL53L1X

from typing import Dict

from dt_config.dt_hardware_settings import HATv2,MotorDirection
from math import fabs, floor

from vpa_robot_interface.msg import WheelsCmd

class wheel_drivers(object):

    # The VPA maintainer try reducing the layer of referencing here

    LEFT_MOTOR_MIN_PWM  = 60        #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM  = 255       #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_PWM = 60        #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255       #: Maximum speed for right motor
    SPEED_TOLERANCE     = 1.0e-2    #: Speed tolerance level
    
    def __init__(self) -> None:
        self.hat = HATv2()
        self.leftMotor  = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")
        
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self._pwm_update()

    def set_wheels_speed(self, left: float, right: float):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1

        """
        self.leftSpeed = left
        self.rightSpeed = right
        self._pwm_update()

    def _pwm_value(self, v, min_pwm, max_pwm):
        """Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            min_pwm (:obj:`int8`): minimum speed as int8
            max_pwm (:obj:`int8`): maximum speed as int8
        """
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)
    
    def _pwm_update(self):
        """Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to the
        linear velocities of the motors. The requested speed gets
        tresholded.
        """
        vl = self.leftSpeed
        vr = self.rightSpeed

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        leftMotorMode = 0
        rightMotorMode = 0

        if fabs(vl) < self.SPEED_TOLERANCE:
            pwml = 0
        elif vl > 0:
            leftMotorMode = MotorDirection.FORWARD
        elif vl < 0:
            leftMotorMode = MotorDirection.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            pwmr = 0
        elif vr > 0:
            rightMotorMode = MotorDirection.FORWARD
        elif vr < 0:
            rightMotorMode = MotorDirection.BACKWARD

        self.leftMotor.set(leftMotorMode, pwml)
        self.rightMotor.set(rightMotorMode, pwmr)

    def __del__(self):
        """Destructor method.

        Releases the motors and deletes tho object.
        """
        self.leftMotor.set(MotorDirection.RELEASE)
        self.rightMotor.set(MotorDirection.RELEASE)
        del self.hat

class dt_I2C_node:

    def __init__(self) -> None:

        self.driver         = wheel_drivers()
        self.tof_sensor     = ToFVL53L1X()

        rospy.on_shutdown(self.shut_hook)

        # Subscribers
        self.sub_topic = rospy.Subscriber("wheels_cmd", WheelsCmd, self.wheels_cmd_cb, queue_size=1)
        # Publishers
        self.rate   = rospy.Rate(10) # 10hz
        self.tofPub = rospy.Publisher('/tof/distance', Range, queue_size=10)
        rospy.Timer(self.rate, self.publish_current_distance)

        rospy.loginfo("Wheel Driver Initialized")
        rospy.loginfo("ToF Sensor Initialized")

    def wheels_cmd_cb(self, msg:WheelsCmd):
        self.driver.set_wheels_speed(left=msg.vel_left,right=msg.vel_right)

    def shut_hook(self):
        self.driver.set_wheels_speed(left=0,right=0)
        del self.driver
        rospy.loginfo('Wheels Released on shutdown')
    
    def publish_current_distance(self):
        distance = self.tof_sensor.distance()

        # message_str = "Distance: %s cm" % distance
        # rospy.loginfo(message_str)

        #for distance in ranges:
        r = Range()

        r.header.stamp      = rospy.Time.now()
        r.header.frame_id   = "/tof_sensor"
        r.radiation_type    = Range.INFRARED
        r.field_of_view     = 3
        r.min_range         = 130
        r.max_range         = 27 * 0.0174532925199433

        r.range = distance

        self.tofPub.publish(r)

if __name__ == '__main__':

    rospy.init_node("dt_I2C_interface", anonymous=False)

    T = dt_I2C_node()

    rospy.spin()