#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import struct

class VPAHATDRIVER:

    def __init__(self) -> None:
        # Initialize the UART connection on /dev/ttyAMA0
        self.ser = serial.Serial(
            port='/dev/ttyAMA0',    # Use ttyAMA0 for communication
            baudrate=115200,        # Set the baud rate
            bytesize=serial.EIGHTBITS,  # 8 data bits
            parity=serial.PARITY_NONE,  # No parity
            stopbits=serial.STOPBITS_ONE,  # 1 stop bit
            timeout=1               # Timeout for reading
        )

        self.current_wheelspeed = 0  # Current wheel speed in rad/s
        self.current_veh_speed  = 0  # Current vehicle speed in m/s

        self.wheel_speed_setpoint = 0  # Desired wheel speed setpoint in rad/s
        self.servo_ratio = 0  # Desired steering ratio
        self.wheelbase = 125e-3  # Wheelbase length in meters (adjust as needed)
        self.wheel_dia = 45e-3  # Diameter of the wheels (in meters)
        self.axis_gap = 174.18e-3  # Axis gap (distance between axles) in meters
        self.max_inner_angle = 22 * (math.pi / 180)  # 22 degrees in radians
        self.max_outer_angle = 18 * (math.pi / 180)  # 18 degrees in radians
        
        self.max_steer_angle = 0.5 * (self.max_inner_angle + self.max_outer_angle)

        # Publisher for the current vehicle speed
        self.veh_speed_pub = rospy.Publisher('current_vehicle_speed', Float32, queue_size=10)

        # Send enable command and wait for response
        self.initialized = self.send_enable_command()    
        if self.initialized:
            rospy.loginfo("Initialization successful. Starting node...")
            # ROS subscriber for cmd_vel
            self.sub_cmd = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

            # ROS Timer to call send_setpoint every 100 ms
            self.timer = rospy.Timer(rospy.Duration(0.1), self.send_setpoint)
        else:
            rospy.loginfo("Initialization failed. No response received.")

    def send_enable_command(self) -> bool:
        # Enable command 5-byte message, starting with 0x3D, repeated 6 times
        enable_command = bytes([0x3D, 0x00, 0x00, 0x00, 0x00])
        for _ in range(6):
            self.ser.write(enable_command)
            # rospy.loginfo(f"Sent enable command: {enable_command.hex()}")
            rospy.sleep(0.5)  # Small delay between sends

        # Wait for response starting with 0x3E
        rospy.loginfo("Waiting for response...")
        response = self.ser.read(5)  # Read 5 bytes from UART
        if len(response) == 5 and response[0] == 0x3E:
            return True
        else:
            return False

    def send_setpoint(self, event):
        # Send wheel speed and steering ratio setpoints every 100 ms
        self.send_wheel_speed_setpoint(self.wheel_speed_setpoint)
        self.send_steering_point(self.servo_ratio)

        # Publish the current vehicle speed
        self.publish_current_veh_speed()

    def send_steering_point(self, servo_ratio: float):
        # Format and send the steering point to UART
        command = self._format_steering_command(servo_ratio)
        self.ser.write(command)

    def send_wheel_speed_setpoint(self, wheel_speed_setpoint: float):
        # Format and send the wheel speed setpoint to UART
        command = self._format_wheel_speed_command(wheel_speed_setpoint)
        self.ser.write(command)

        # Wait for the response containing the current speed
        response = self.ser.read(5)  # Read 5 bytes from UART
        if len(response) == 5 and response[0] == 0x04:
            # Extract the float value from the response
            self.current_wheelspeed = struct.unpack('<f', response[1:])[0]
            # rospy.loginfo(f"Received current wheel speed: {self.current_wheelspeed} rad/s")
            # Calculate current vehicle speed
            self._wheelspeed_to_vehspeed()
        else:
            rospy.loginfo(f"Invalid or no response received for speed: {response.hex() if response else 'None'}")

    def _format_steering_command(self, servo_ratio: float) -> bytes:
        # Format the steering point command, starting with 0x01 followed by the float value
        steering_float_bytes = struct.pack('<f', servo_ratio)  # Convert float to 4 bytes in little-endian format
        return bytes([0x01]) + steering_float_bytes  # Combine the command header with the float value

    def _format_wheel_speed_command(self, wheel_speed_setpoint: float) -> bytes:
        # Format the wheel speed setpoint command, starting with 0x03 followed by the float value
        speed_float_bytes = struct.pack('<f', wheel_speed_setpoint)  # Convert float to 4 bytes in little-endian format
        return bytes([0x03]) + speed_float_bytes  # Combine the command header with the float value

    def cmd_vel_callback(self, msg: Twist) -> None:
        # Extract linear and angular speeds from the Twist message
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Calculate wheel speed setpoint and steering ratio based on linear speed and yaw rate
        [self.wheel_speed_setpoint, self.servo_ratio] = self._kinematics_calculate(linear_speed, angular_speed)

    def _kinematics_calculate(self, linear_speed: float, yaw_rate: float) -> list:
        # Calculate wheel speed in radians per second
        wheel_speed_rad_sec = 2 * linear_speed / self.wheel_dia

        # Calculate the desired turning radius based on the yaw rate
        if yaw_rate != 0:
            desired_radius = linear_speed / yaw_rate  # Turning radius = linear_speed / yaw_rate
        else:
            desired_radius = float('inf')  # Straight line if no yaw rate

        # Calculate steering angle based on desired turning radius
        if desired_radius != float('inf'):
            # Using the wheelbase and desired radius to find the steering angle
            steering_angle = math.atan(self.wheelbase / desired_radius)
        else:
            steering_angle = 0.0  # No steering needed for straight line

        # Calculate the steering ratio
        steer_ratio = steering_angle / self.max_steer_angle

        # Invert steering ratio for right turn as negative convention
        if yaw_rate > 0:
            steer_ratio = -steer_ratio

        return [wheel_speed_rad_sec, steer_ratio]
    
    def _wheelspeed_to_vehspeed(self):
        # Convert current wheel speed to vehicle speed (m/s)
        self.current_veh_speed = (self.current_wheelspeed / 2) * self.wheel_dia

    def publish_current_veh_speed(self):
        # Publish the current vehicle speed
        veh_speed_msg = Float32()
        veh_speed_msg.data = self.current_veh_speed
        self.veh_speed_pub.publish(veh_speed_msg)
        # rospy.loginfo(f"Published current vehicle speed: {self.current_veh_speed} m/s")

    def close_serial(self):
        # Close the serial port and stop the timer when done
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial connection closed.")

if __name__ == '__main__':
    rospy.init_node('vpahat_driver')
    driver = VPAHATDRIVER()

    try:
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    finally:
        driver.close_serial()  # Ensure serial port is closed on exit
