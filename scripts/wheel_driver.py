#!/usr/bin/python3

import rospy

from math import fabs, floor

from dt_config.dt_hardware_settings import MotorDirection, HATv2

from vpa_robot_interface.msg import WheelsCmd,WheelsEncoder
from vpa_robot_interface.cfg import omegaConfig

from pid_controller.pi_format import PI_controller

from dynamic_reconfigure.server import Server

class WheelDriver:

    LEFT_MOTOR_MIN_PWM  = 60        #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM  = 255       #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_PWM = 60        #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255       #: Maximum speed for right motor
    SPEED_TOLERANCE     = 1.0e-2    #: Speed tolerance level

    def __init__(self) -> None:
        self.hat        = HATv2
        self.leftMotor  = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")

        self.leftThrottle   = 0.0
        self.rightThrottle  = 0.0
        self._pwm_update()

    def set_wheels_throttle(self, left: float, right: float):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1
           is_test_cmd (:obj:`bool`): whether this is a command issue by the hardware test

        """
        self.leftThrottle  = left
        self.rightThrottle = right
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
        vl = self.leftThrottle
        vr = self.rightThrottle

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        leftMotorMode   = 0
        rightMotorMode  = 0

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
    
class WheelDriverNode:

    def __init__(self) -> None:

        rospy.on_shutdown(self.shut_hook)

        self.driver = WheelDriver()

        self.kp     = 0.15
        self.ki     = 0.005

        self.omega_controller_left  = PI_controller(ki=self.ki,kp=self.kp)
        self.omega_controller_right = PI_controller(ki=self.ki,kp=self.kp)

        self.omega_left_ref     = 0
        self.omega_right_ref    = 0

        self.omega_left_sig     = 0
        self.omega_right_sig    = 0

        self.throttle_left      = 0
        self.throttle_right     = 0

        # Subscribers
        self.sub_cmd       = rospy.Subscriber("wheels_cmd", WheelsCmd, self.wheels_cmd_cb, queue_size=1)
        self.sub_left_enc  = rospy.Subscriber("left_omega",WheelsEncoder,self.left_omega_cb,queue_size=1)
        self.sub_right_enc = rospy.Subscriber("right_omega",WheelsEncoder,self.right_omega_cb,queue_size=1)
        
        self.srv = Server(omegaConfig,self.dynamic_reconfigure_callback)

    def wheels_cmd_cb(self,msg:WheelsCmd) -> None:

        self.omega_left_ref     = msg.vel_right
        self.omega_right_ref    = msg.vel_right

    def shut_hook(self) -> None:
        self.driver.set_wheels_throttle(0)
        self.driver.set_wheels_throttle(0)
        del self.driver
        rospy.loginfo('Wheel Driver Shutdown')
    
    def left_omega_cb(self,msg:WheelsEncoder) -> None:
        self.omega_left_sig     = msg.omega
        self.throttle_left      = self.omega_controller_left.pi_control(self.omega_left_ref,self.omega_left_sig)
        if self.throttle_left > 1:
            self.throttle_left = 1
        elif self.throttle_left < -1:
            self.throttle_left = -1
        self.driver.set_wheels_throttle(left=self.throttle_left,right=self.throttle_right)

    def right_omega_cb(self,msg:WheelsEncoder) -> None:
        self.omega_right_sig    = msg.omega
        self.throttle_right     = self.omega_controller_right.pi_control(self.omega_right_ref,self.omega_right_sig)
        if self.throttle_right > 1:
            self.throttle_right = 1
        elif self.throttle_right < -1:
            self.throttle_right = -1
        self.driver.set_wheels_throttle(left=self.throttle_left,right=self.throttle_right)


    def dynamic_reconfigure_callback(self,config,level):
        self.kp = config.kp
        self.ki = config.ki
        self.omega_controller_left.update_controller_param(self.kp,self.ki)
        self.omega_controller_right.update_controller_param(self.kp,self.ki)
        return config
    
if __name__ == '__main__':

    try:
        rospy.init_node("wheel_driver")
        N = WheelDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')