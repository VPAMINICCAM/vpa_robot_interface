#!/usr/bin/env python3

import os
import socket
import rospy
import serial
import struct  # For packing and unpacking data
import RPi.GPIO as GPIO  # Importing GPIO for controlling pins
from geometry_msgs.msg import Twist  # Importing Twist message type for cmd_vel
from std_msgs.msg import Float, Bool


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

        self.veh_name       = socket.gethostname()
        # Chassis parameters
        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.045)  # Example: 45 mm
        self.debug_mode = rospy.get_param('~debug_mode', False)  # Debug mode flag
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

    def estop_cb(self,msg:Bool) -> None:
        self.global_stop_flag = msg.data
        rospy.loginfo_once('%s: global brake: %s',self.veh_name,str(msg.data))


    def estop_local_cb(self,msg:Bool) -> None:
        self.local_stop_flag = msg.data
        rospy.loginfo_once('%s: local brake: %s',self.veh_name,str(msg.data))

    def cmd_vel_callback(self, msg: Twist) -> None:

        """Callback function for /cmd_vel topic. This is called whenever a new cmd_vel message is received."""
        linear_velocity = msg.linear.x  # Forward/backward velocity

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

