#!/usr/bin/env python3

import socket
import rospy

from geometry_msgs.msg import Twist  # Importing Twist message type for cmd_vel
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
from vpa_robot_interface.msg import DirectCmd  # Import the custom message

from hardware.chassis import CHASSIS
from hardware.serial_com import SerialComm

from protocol.usart_to_hat import MCUcommProtocol
from controller.pid import PID
from dynamic_reconfigure.server import Server
from vpa_robot_interface.cfg import SpdCtrlConfig

class VPAHAT:

    def __init__(self):
        rospy.init_node('vpa_hat')
        
        # import parameter
        self.robot_name         = socket.gethostname()
        
        # set chassis 
        wheel_diameter          = rospy.get_param('~wheel_diameter', 0.045)
        self.chassis            = CHASSIS(wheel_diameter)

        self.debug_mode         = rospy.get_param('~debug_mode', False)
        self.direct_throttle    = rospy.get_param('~direct_throttle', False)

        self.start_imu          = rospy.get_param('~start_imu', True)

        self.global_stop_flag   = True
        self.local_stop_flag    = True

        # controller parameters for lower level controller
        self.kp         = 0.05
        self.ki         = 0.004
        self.kd         = 0

        self.kff = 0.01
        self.bff = 0.14

        # setting communication
        self.usart_com = MCUcommProtocol(SerialComm('/dev/ttyAMA0', 115200, self.debug_mode))
        
        # shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        
        start_ack = self.usart_com.send_start_message()
        
        if not start_ack:
            rospy.signal_shutdown('ROS node shutting down')

        self.usart_com.send_speed_reading_message()

        # Subscribers and Publishers
        self.pub_real_wheel_speeds = rospy.Publisher('wheel_speed', Float32, queue_size=10)
        self.pub_real_throttle = rospy.Publisher('throttle_set', Float32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / 50.0), self.timer_callback)

        rospy.Subscriber("/global_brake", Bool, self.estop_cb)
        rospy.Subscriber("local_brake", Bool, self.estop_local_cb)

        if self.direct_throttle:
            rospy.loginfo('%s: direct cmd input mode',self.robot_name)
            rospy.Subscriber("direct_cmd", DirectCmd, self.direct_cmd_callback)
        else:
            rospy.loginfo('%s: twist cmd input mode',self.robot_name)
            rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        
        # Initialize IMU data if IMU is enabled
        self.angular_velocity_z = 0.0
        if self.start_imu:
            self.steer_pid = PID(
                Kp=0.3,Ki=0.1,integral_limits=(-5,5),output_limits=(-0.05,0.05),smoothing_factor=0.5
            )
            rospy.Subscriber('imu', Imu, self.imu_callback)


        self.dynamic_params = Server(SpdCtrlConfig, self.dynamic_reconf_callback)
        # self.usart_com.serial_comm.send_message(cmd_id=0x01)
        rospy.loginfo("%s,actuator node initialized successfully.",self.robot_name)

    def imu_callback(self, msg:Imu):
        self.angular_velocity_z = msg.angular_velocity.z

    def dynamic_reconf_callback(self, config, level):
        rospy.loginfo(f"Dynamic Reconfigure speed pid:kp={config.kp}, ki={config.ki}, kd={config.kd}, kff={config.kff}, bff={config.bff}")

        # Send PID parameters update only if any changes
        if self.kp != config.kp or self.ki != config.ki or self.kd != config.kd:
            self.kp, self.ki, self.kd = config.kp, config.ki, config.kd
            self.usart_com.send_message(self.usart_com.pid_id, self.kp, self.ki, self.kd)

        if self.kff != config.kff or self.bff != config.bff:
            self.kff,self.bff = config.kff,config.bff
            self.usart_com.send_message(self.usart_com.pid_ff_id,self.kff,self.bff)

        return config
    def timer_callback(self, event):
        # Create and populate the Float32MultiArray message

        # Publish the message
        self.publish_wheel_speed(self.usart_com.speed)
        self.publish_throttle(self.usart_com.throttle_set)


    def estop_cb(self, msg: Bool):
        """
        Callback for global brake messages.
        """
        self.global_stop_flag = msg.data
        rospy.loginfo(f"Global brake: {msg.data}")

    def estop_local_cb(self, msg: Bool):
        """
        Callback for local brake messages.
        """
        self.local_stop_flag = msg.data
        rospy.loginfo(f"Local brake: {msg.data}")
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel messages.
        """
        if self.global_stop_flag or self.local_stop_flag:
            self.usart_com.send_message(self.usart_com.omega_id, 0)  # Stop the vehicle
        else:
            # Calculate wheel speed and send it to the actuator
            linear_velocity = msg.linear.x
            yaw = msg.angular.z
            omega = self.chassis.calculate_wheel_speeds(linear_velocity)
            self.usart_com.send_message(self.usart_com.omega_id, omega)
        
            str_value = self.chassis.yaw2steerratio(linear_velocity,yaw)

            if self.start_imu:
                pid_output = self.steer_pid.compute(setpoint=yaw,measurement=self.angular_velocity_z) # negative for right turn
                if linear_velocity == 0:
                    self.steer_pid.reset_pid()
                    pid_output = 0
                str_value += pid_output

            self.usart_com.send_message(self.usart_com.steer_id,str_value)

    def direct_cmd_callback(self, msg: DirectCmd):
        """
        Callback for direct_cmd messages.
        """
        if self.global_stop_flag or self.local_stop_flag:
            self.usart_com.send_message(self.usart_com.throttle_id, 0)  # Stop throttle
        else:
            # Send throttle and steering commands directly
            self.usart_com.send_message(self.usart_com.throttle_id, msg.throttle)
            self.usart_com.send_message(self.usart_com.steer_id, msg.steering)

    def publish_wheel_speed(self, speed):
        """
        Publish the received wheel speed to the 'real_wheel_speeds' topic.
        """
        speed_msg = Float32()
        speed_msg.data = speed
        self.pub_real_wheel_speeds.publish(speed_msg)

        if self.debug_mode:
            rospy.loginfo(f"Published wheel speed: {speed:.2f}")

    def publish_throttle(self, throttle):
        """
        Publish the received throttle.
        """
        throttle_msg = Float32()
        throttle_msg.data = throttle
        self.pub_real_throttle.publish(throttle_msg)

        if self.debug_mode:
            rospy.loginfo(f"Published throttle: {throttle:.2f}")

    def shutdown_hook(self):
        """
        Perform cleanup actions on node shutdown, such as cleaning up GPIO and closing the serial connection.
        """
        rospy.loginfo("Shutting down VPAHAT and cleaning up resources.")

        # Clean up GPIO resources
        try:
            rospy.loginfo("GPIO resources cleaned up.")
        except Exception as e:
            rospy.logerr(f"Error cleaning up GPIO: {e}")

        # Close the serial connection
        try:
            if self.usart_com.serial_comm.serial_conn.is_open:
                self.usart_com.send_message(cmd_id=self.usart_com.shutdown_id)
                rospy.sleep(0.2)
                self.usart_com.serial_comm.serial_conn.close()
                rospy.loginfo("Serial connection closed.")
        except Exception as e:
            rospy.logerr(f"Error closing serial connection: {e}")

    
if __name__ == "__main__":
    try:
        # Create an instance of the VPAHAT class
        vpa_hat = VPAHAT()
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error in the traction node: {e}")
