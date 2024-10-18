#!/usr/bin/env python3

import os
import rospy
import serial
import struct  # For packing and unpacking data
import RPi.GPIO as GPIO  # Importing GPIO for controlling pins
from geometry_msgs.msg import Twist  # Importing Twist message type for cmd_vel
from std_msgs.msg import Float32MultiArray, Bool  # For publishing setpoints and real wheel speeds

class CHASSIS:
    def __init__(self, wheel_diameter: float, wheelbase: float):
        """Initialize the chassis with wheel diameter and wheelbase."""
        self.wheel_diameter = wheel_diameter  # Diameter of the wheels
        self.wheelbase = wheelbase  # Distance between the two wheels

        self.trim = 0

    def calculate_wheel_speeds(self, linear_x: float, angular_z: float):
        """Calculate left (A) and right (B) wheel speeds in radians per second (rps) based on the linear and angular velocity from cmd_vel."""
        if not linear_x == 0: 
            r_left = linear_x - (self.wheelbase * angular_z) 
            r_right = linear_x + (self.wheelbase * angular_z)

            omega_left = ((r_left) / (self.wheel_diameter * 3.14)) * (1 - self.trim)
            omega_right = (r_right) / (self.wheel_diameter * 3.14) * (1 + self.trim)
        else:
            omega_left  = 0
            omega_right = 0
        return omega_left, omega_right
        

class VPAHAT:

    def __init__(self):
        rospy.init_node('vpa_hat')
        
        self._enable_STBY_pin()
        self._enable_USART()
        
        # Chassis parameters
        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.065)  # Example: 65 mm
        self.wheelbase = rospy.get_param('~wheelbase', 0.105)  # Example: 105 mm
        self.debug_mode = rospy.get_param('~debug_mode', False)  # Debug mode flag

        self.left_speed = 0
        self.right_speed = 0

        self.global_stop_flag   = True
        self.local_stop_flag    = True
        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)

        self.chassis = CHASSIS(self.wheel_diameter, self.wheelbase)
        self.chassis.trim = self._read_trim_from_file()

        # Publishers
        self.pub_real_wheel_speeds = rospy.Publisher('real_wheel_speeds', Float32MultiArray, queue_size=10)
        if self.debug_mode:
            self.pub_setpoints_debug = rospy.Publisher('setpoints_debug', Float32MultiArray, queue_size=10)

        # Subscribe to the cmd_vel topic
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
    
    def estop_cb(self,msg:Bool) -> None:
        self.global_stop_flag = msg.data

    def estop_local_cb(self,msg:Bool) -> None:
        self.local_stop_flag = msg.data

    def _enable_STBY_pin(self) -> None:
        GPIO.setmode(GPIO.BCM)
        self.enable_pin = 23  # GPIO23
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.output(self.enable_pin, GPIO.HIGH)  # Set GPIO23 high to enable hardware

        rospy.loginfo("STBY PIN set to HIGH, hardware enabled")

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

    def reverse_bytes(self, data: bytes) -> bytes:
        """Reverse the byte order of the packed float."""
        return data[::-1]

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback function for /cmd_vel topic. This is called whenever a new cmd_vel message is received."""
        linear_velocity = msg.linear.x  # Forward/backward velocity
        angular_velocity = msg.angular.z  # Angular velocity (rotation)

        # Calculate left and right wheel speeds in radians per second
        omega_left, omega_right = self.chassis.calculate_wheel_speeds(linear_velocity, angular_velocity)

        # Publish the setpoints if debug mode is enabled
        if self.debug_mode:
            # Log the calculated wheel speeds
            rospy.loginfo(f"Left wheel (A) speed: {omega_left:.2f} rps, Right wheel (B) speed: {omega_right:.2f} rps")
            setpoints_msg = Float32MultiArray()
            setpoints_msg.data = [omega_left, omega_right]
            self.pub_setpoints_debug.publish(setpoints_msg)
            rospy.loginfo(f"Published setpoints in debug mode: {omega_left:.2f} rps, {omega_right:.2f} rps")

        # Send the setpoints over USART
        if self.local_stop_flag or self.global_stop_flag:
            self.send_wheel_setpoints(0, 0)
        else:
            self.send_wheel_setpoints(omega_left, omega_right)

    def _read_trim_from_file(self):
        """Read the trim value from a file in the package."""
        package_path = os.path.dirname(os.path.realpath(__file__))  # Get current package directory
        trim_file_path = os.path.join(package_path, 'config', 'trim.txt')

        try:
            if os.path.exists(trim_file_path):
                with open(trim_file_path, 'r') as file:
                    trim_value = float(file.readline().strip())
                    rospy.loginfo(f"Loaded trim value: {trim_value}")
                    return trim_value
            else:
                return 0.0
        except (FileNotFoundError, ValueError) as e:
            rospy.logwarn(f"Failed to read trim value from file: {e}, using default trim = 0.0")
            return 0.0  # Default trim value

    def send_wheel_setpoints(self, omega_left: float, omega_right: float) -> None:
        """
        Send the wheel setpoints (left and right) over USART.
        - 1 byte identifier: 0x03
        - 4 bytes for left wheel speed (A) in reversed float
        - 4 bytes for right wheel speed (B) in reversed float
        """


        try:
            identifier = struct.pack('<B', 0x03)
            omega_left_packed = struct.pack('<f', omega_left)
            omega_right_packed = struct.pack('<f', omega_right)

            omega_left_reversed = self.reverse_bytes(omega_left_packed)
            omega_right_reversed = self.reverse_bytes(omega_right_packed)

            message = identifier + omega_left_reversed + omega_right_reversed

            self.serial_conn.write(message)
            # rospy.loginfo(f"Sent wheel setpoints in hex: {message.hex()}")
        except Exception as e:
            rospy.logerr(f"Error sending wheel setpoints: {e}")

    def read_wheel_speeds(self) -> None:
        """
        Read the real wheel speeds from the MCU at 20Hz.
        - 1 byte identifier: 0x04
        - 4 bytes for left wheel speed (A) in float
        - 4 bytes for right wheel speed (B) in float
        """
        try:
            data = self.serial_conn.read(9)
            if len(data) == 9:
                identifier = data[0]
                if identifier == 0x04:
                    left_speed_raw = struct.unpack('<f', data[1:5])[0]
                    right_speed_raw = struct.unpack('<f', data[5:9])[0]
                    
                    if left_speed_raw > -100:
                        left_speed = left_speed_raw
                        self.left_speed = left_speed
                    else:
                        left_speed = self.left_speed
                    
                    if right_speed_raw > -100:
                        right_speed = right_speed_raw
                        self.right_speed = right_speed
                    else:
                        right_speed = self.right_speed

                    # rospy.loginfo(f"Received real wheel speeds: Left = {left_speed:.2f} rps, Right = {right_speed:.2f} rps")

                    real_speeds_msg = Float32MultiArray()
                    real_speeds_msg.data = [left_speed, right_speed]
                    self.pub_real_wheel_speeds.publish(real_speeds_msg)
                else:
                    rospy.logwarn(f"Unexpected identifier: {identifier}")
            else:
                rospy.logwarn("Incomplete data received for wheel speeds")
        except Exception as e:
            rospy.logerr(f"Error reading wheel speeds: {e}")

    def run(self) -> None:
        """Main loop where the ROS node executes."""
        rate = rospy.Rate(20)  # Run at 20Hz to match the wheel speed update rate
        try:
            while not rospy.is_shutdown():
                self.read_wheel_speeds()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Cleanup function to close resources."""
        if self.serial_conn.is_open:
            self.serial_conn.close()
        GPIO.cleanup()
        rospy.loginfo("Cleaned up GPIO and closed serial connection")

if __name__ == '__main__':
    node = VPAHAT()
    node.run()
