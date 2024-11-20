#!/usr/bin/env python3

import os
import socket
import rospy

import struct
import RPi.GPIO as GPIO  # Importing GPIO for controlling pins
from geometry_msgs.msg import Twist  # Importing Twist message type for cmd_vel
from std_msgs.msg import Float32, Bool
from vpa_robot_interface.msg import DirectCmd  # Import the custom message

from traction.chassis import CHASSIS
from traction.serial_com import SerialComm

from dynamic_reconfigure.server import Server
from vpa_robot_interface.cfg import SpdCtrlConfig

class VPAHAT:

    def __init__(self):
        rospy.init_node('vpa_hat')
        
        self.robot_name         = socket.gethostname()

        wheel_diameter          = rospy.get_param('~wheel_diameter', 0.045)
        self.chassis            = CHASSIS(wheel_diameter)

        self.debug_mode         = rospy.get_param('~debug_mode', False)

        self.direct_throttle    = rospy.get_param('~direct_throttle', False)

        self.global_stop_flag   = True
        self.local_stop_flag    = True

        # Initialize the serial communication
        self.serial_comm = SerialComm('/dev/ttyAMA0', 115200, self.debug_mode)
        self.serial_comm.set_read_callback(self.process_usart_message)
        
        rospy.on_shutdown(self.shutdown_hook)

        # reset MCU
        self._send_reset_message()
        rospy.sleep(2)

        # Enable GPIO 23 for communication
        self._enable_communication_gpio()

        # Publishers
        self.pub_real_wheel_speeds = rospy.Publisher('wheel_speed', Float32, queue_size=10)

        rospy.Subscriber("/global_brake", Bool, self.estop_cb)
        rospy.Subscriber("local_brake", Bool, self.estop_local_cb)

        if self.direct_throttle:
            rospy.loginfo('%s: direct cmd input mode',self.robot_name)
            rospy.Subscriber("direct_cmd", DirectCmd, self.direct_cmd_callback)
        else:
            rospy.loginfo('%s: twist cmd input mode',self.robot_name)
            rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.dynamic_params = Server(SpdCtrlConfig, self.dynamic_reconf_callback)

        rospy.loginfo("%s,actuator node initialized successfully.",self.robot_name)
    
    def _send_reset_message(self):
        """
        Send the reset message with cmd_id 0x15 and validate the reply.
        """
        try:
            # Send reset message
            self.serial_comm.send_message(cmd_id=0x15)
            rospy.loginfo("Reset message (cmd_id=0x15) sent. Waiting for reply...")

            # Wait for the reply
            reply = self.serial_comm.serial_conn.read(4)  # Expected reply length: 4 bytes
            if reply == bytearray([0x02, 0x01, 0x16, 0x03]):
                rospy.loginfo("MCU reset acknowledged (cmd_id=0x16).")
            else:
                rospy.logerr("No valid reset acknowledgment received. Please manually reset the MCU.")

        except Exception as e:
            rospy.logerr(f"Error sending reset message: {e}")

    def dynamic_reconf_callback(self, config, level):
        rospy.loginfo(f"Dynamic Reconfigure: deadzone={config.deadzone}, kp={config.kp}, ki={config.ki}, kd={config.kd}")

        # Send deadzone update only if it changes
        if self.deadzone != config.deadzone:
            self.deadzone = config.deadzone
            self.serial_comm.send_message(0x11, self.deadzone)

        # Send PID parameters update only if any changes
        if self.kp != config.kp or self.ki != config.ki or self.kd != config.kd:
            self.kp, self.ki, self.kd = config.kp, config.ki, config.kd
            self.serial_comm.send_message(0x13, self.kp, self.ki, self.kd)

        return config

    def _enable_communication_gpio(self):
        """
        Configure GPIO 23 to enable communication by setting it high.
        """
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        self.enable_pin = 23  # GPIO pin number
        GPIO.setup(self.enable_pin, GPIO.OUT)  # Set pin as output
        GPIO.output(self.enable_pin, GPIO.HIGH)  # Set the pin high to enable communication

        if self.debug_mode:
            rospy.loginfo("GPIO 23 set high to enable communication.")

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
            self.serial_comm.send_message(0x07, 0)  # Stop the vehicle
        else:
            # Calculate wheel speed and send it to the actuator
            linear_velocity = msg.linear.x
            omega = self.chassis.calculate_wheel_speeds(linear_velocity)
            self.serial_comm.send_message(0x07, omega)    

    def direct_cmd_callback(self, msg: DirectCmd):
        """
        Callback for direct_cmd messages.
        """
        if self.global_stop_flag or self.local_stop_flag:
            self.serial_comm.send_message(0x09, 0)  # Stop throttle
        else:
            # Send throttle and steering commands directly
            self.serial_comm.send_message(0x09, msg.throttle)
            self.serial_comm.send_message(0x03, msg.steering)

    def process_usart_message(self, message):
        """
        Process a received USART message.
        """
        try:
            cmd_id = message[2]

            # Define a dictionary mapping cmd_id to their handler methods
            cmd_handlers = {
                0x02: self.handle_speed_message,  # Speed message
            }

            # Get the handler for the received cmd_id
            handler = cmd_handlers.get(cmd_id, self.handle_unknown_message)

            # Call the handler with the message
            handler(message)

        except Exception as e:
            rospy.logerr(f"Error processing USART message: {e}")

    def handle_speed_message(self, message):
        """
        Handle speed update messages (cmd_id = 0x02).
        """
        speed = struct.unpack('<f', message[3:7])[0]
        self.publish_wheel_speed(speed)

        if self.debug_mode:
            rospy.loginfo(f"Received speed: {speed:.2f}")

    def handle_unknown_message(self, message):
        """
        Handle unknown or unsupported messages.
        """
        cmd_id = message[2]
        rospy.logwarn(f"Unknown cmd_id received: {cmd_id}")


    def publish_wheel_speed(self, speed):
        """
        Publish the received wheel speed to the 'real_wheel_speeds' topic.
        """
        speed_msg = Float32()
        speed_msg.data = speed
        self.pub_real_wheel_speeds.publish(speed_msg)

        if self.debug_mode:
            rospy.loginfo(f"Published wheel speed: {speed:.2f}")

    def shutdown_hook(self):
        """
        Perform cleanup actions on node shutdown, such as cleaning up GPIO and closing the serial connection.
        """
        rospy.loginfo("Shutting down VPAHAT and cleaning up resources.")

        # Clean up GPIO resources
        try:
            GPIO.cleanup()
            rospy.loginfo("GPIO resources cleaned up.")
        except Exception as e:
            rospy.logerr(f"Error cleaning up GPIO: {e}")

        # Close the serial connection
        try:
            if self.serial_comm.serial_conn.is_open:
                self.serial_comm.serial_conn.close()
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
