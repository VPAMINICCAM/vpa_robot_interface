#!/usr/bin/env python3

import os
import socket
import rospy
import time
import serial
import struct  # For packing and unpacking data
import RPi.GPIO as GPIO  # Importing GPIO for controlling pins
from geometry_msgs.msg import Twist  # Importing Twist message type for cmd_vel
from std_msgs.msg import Float, Bool

from vpa_robot_interface.msg import DirectCmd  # Import the custom message

class CHASSIS:
    def __init__(self, wheel_diameter: float):
        """Initialize the chassis with wheel diameter and wheelbase."""
        self.wheel_diameter = wheel_diameter  # Diameter of the wheels


    def calculate_wheel_speeds(self, linear_x: float):
        """Calculate  wheel speeds in revolution per second (rps) based on the linear and angular velocity from cmd_vel."""
        if not linear_x == 0: 
            omega = linear_x / (2*3.14*self.wheel_diameter)
        else:
            omega = 0
        return omega

class VPAHAT:

    def __init__(self):
        rospy.init_node('vpa_hat')
        self._enable_STBY_pin()

        self._enable_USART()

        time.sleep(0.5)

        self.enable_actuator()

        self.veh_name               = socket.gethostname()
        # Chassis parameters
        self.wheel_diameter         = rospy.get_param('~wheel_diameter', 0.045)  # Example: 45 mm
        self.debug_mode             = rospy.get_param('~debug_mode', False)  # Debug mode flag
        self.direct_throttle        = rospy.get_param('~direct_throttle', False)  # Direct throttle mode flag
        self.speed = 0
        self.chassis = CHASSIS(self.wheel_diameter)
        self.global_stop_flag   = True
        rospy.loginfo("%s: global brake activated",self.veh_name)
        self.local_stop_flag    = True
        rospy.loginfo("%s: local brake activated",self.veh_name)

        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)

        # Publishers
        self.pub_real_wheel_speeds = rospy.Publisher('real_wheel_speeds', Float, queue_size=10)
        if self.debug_mode:
            self.pub_setpoints_debug = rospy.Publisher('setpoints_debug', Float, queue_size=10)

        if self.direct_throttle:
            # DirectCmd subscriber (for direct throttle)
            self.sub_direct_cmd = rospy.Subscriber("direct_cmd", DirectCmd, self.direct_cmd_callback, queue_size=1)
        else:
            self.sub_cmd_vel    = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

        import threading
        threading.Thread(target=self.read_usart_messages, daemon=True).start()

    def _enable_STBY_pin(self) -> None:
        GPIO.setmode(GPIO.BCM)
        self.enable_pin = 23  # GPIO23
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.output(self.enable_pin, GPIO.HIGH)  # Set GPIO23 high to enable hardware

    
    def _enable_USART(self) -> None:
        self.port = rospy.get_param('~port', '/dev/ttyAMA0')  # Default serial port
        self.baudrate = rospy.get_param('~baudrate', 115200)  # Default baud rate

        try:
            self.serial_conn = serial.Serial(
                port=self.port,               
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,  
                parity=serial.PARITY_NONE,   
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            rospy.loginfo(f"Initialized serial connection on {self.port} with baud rate {self.baudrate}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial connection: {e}")
            rospy.signal_shutdown("Serial initialization failed")

    def enable_actuator(self):
        """
        Sends a command to enable the actuator via USART (cmd_id = 0x01).
        """
        try:
            # Use send_usart_message with cmd_id = 0x01 and no additional data
            self.send_usart_message(0x01)

            if self.debug_mode:
                rospy.loginfo(f"{self.veh_name}: Actuator enable command sent (cmd_id=0x01)")
        except serial.SerialException as e:
            rospy.logerr(f"{self.veh_name}: Failed to send actuator enable command: {e}")


    def estop_cb(self,msg:Bool) -> None:
        self.global_stop_flag = msg.data
        rospy.loginfo_once('%s: global brake: %s',self.veh_name,str(msg.data))


    def estop_local_cb(self,msg:Bool) -> None:
        self.local_stop_flag = msg.data
        rospy.loginfo_once('%s: local brake: %s',self.veh_name,str(msg.data))

    def cmd_vel_callback(self, msg: Twist) -> None:

        """Callback function for /cmd_vel topic. This is called whenever a new cmd_vel message is received."""

        linear_velocity = msg.linear.x  # Forward/backward velocity

        if self.global_stop_flag or self.local_stop_flag:
            self.send_usart_message(0x07, 0)
        else:
            omega = self.chassis.calculate_wheel_speeds(linear_velocity)

            # this so far is only about 

            # Send omega to the lower controller via USART
            self.send_usart_message(0x07, omega)


    def send_usart_message(self, cmd_id: int, *data: float) -> None:
        """
        Send a message over USART to the lower controller with dynamic payload length.

        Args:
            cmd_id (int): Command identifier for the message.
            *data (float): Variable number of float values to send as the payload.
        """
        try:
            # Protocol format: [START_MARKER][LENGTH][CMD_ID][DATA...][END_MARKER]
            start_marker = 0x02
            end_marker = 0x03

            # Convert all float data to little-endian format
            payload = bytearray()
            for value in data:
                payload.extend(struct.pack('<f', value))  # Pack each float

            # Calculate length dynamically (1 for CMD_ID + size of payload)
            length = 1 + len(payload)

            # Build the message
            message = bytearray([start_marker, length, cmd_id])
            message.extend(payload)
            message.append(end_marker)

            # Send the message over USART
            self.serial_conn.write(message)

            if self.debug_mode:
                rospy.loginfo(f"{self.veh_name}: Sent cmd_id {cmd_id}, data: {data} (Raw: {message.hex()})")

        except serial.SerialException as e:
            rospy.logerr(f"{self.veh_name}: Failed to send message over serial: {e}")

    def direct_cmd_callback(self, msg: DirectCmd) -> None:
        """
        Callback for /direct_cmd topic. Handles throttle commands in direct throttle mode.
        """
        if self.global_stop_flag or self.local_stop_flag:
            self.send_usart_message(0x09, 0)
            self.send_usart_message(0x03, 0)
            
        else:
            # Send throttle value directly to the lower controller
            throttle = msg.throttle
            self.send_usart_message(0x09, throttle)
            steering = msg.steering
            # Send steering value via USART with cmd_id = 0x03
            self.send_usart_message(0x03, steering)

            if self.debug_mode:
                rospy.loginfo(f"{self.veh_name}: Sent throttle: {throttle:.2f} in direct throttle mode")


    def read_usart_messages(self):
        """
        Continuously read and process messages from the STM32 over USART.
        """
        # rospy.loginfo("Starting USART read loop...")
        try:
            while not rospy.is_shutdown():
                # Read a full message
                message = self._read_message()
                if message:
                    self._process_usart_message(message)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down USART read loop.")
        except Exception as e:
            rospy.logerr(f"Error in USART read loop: {e}")

    def _read_message(self):
        """
        Read a full message from USART based on the protocol.
        Returns the raw message as a bytearray or None if no valid message is received.
        """
        try:
            # Wait for the start marker
            byte = self.serial_conn.read(1)
            if not byte or byte[0] != 0x02:  # Start marker
                return None

            # Read the length byte
            length_byte = self.serial_conn.read(1)
            if not length_byte:
                return None
            length = length_byte[0]

            # Read the remaining bytes (length + end marker)
            message = self.serial_conn.read(length + 1)
            if len(message) != length + 1 or message[-1] != 0x03:  # End marker
                return None

            # Return the full message
            return bytearray([0x02]) + bytearray([length]) + message
        except Exception as e:
            rospy.logerr(f"Error reading USART message: {e}")
            return None

    def _process_usart_message(self, message):
        """
        Process a received USART message.
        """
        try:
            cmd_id = message[2]

            # Check if the message is a speed message (cmd_id = 0x04)
            if cmd_id == 0x02:
                speed = struct.unpack('<f', message[3:7])[0]

                # Publish the speeds
                speeds_msg = Float()
                speeds_msg.data = speed
                self.pub_real_wheel_speeds.publish(speeds_msg)

                # Debug logging
                if self.debug_mode:
                    rospy.loginfo(f"{self.veh_name}: Received Speeds -  {speed:.2f}")
            else:
                rospy.logwarn(f"{self.veh_name}: Received unknown cmd_id: {cmd_id}")
        except Exception as e:
            rospy.logerr(f"Error processing USART message: {e}")

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
