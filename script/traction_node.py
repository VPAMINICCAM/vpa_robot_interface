#!/usr/bin/env python3

import os
import socket
import rospy

from geometry_msgs.msg import Twist  # Importing Twist message type for cmd_vel
from std_msgs.msg import Float32MultiArray, Bool  # For publishing setpoints and real wheel speeds
from sensor_msgs.msg import Imu  # For subscribing to the IMU data
from hardware.chassis import CHASSIS
from hardware.usart import SerialComm
from protocol.usart_to_hat import MCUcommProtocol

from sensor_signal_process.filter import ComplementaryFilter

class VPAHAT:

    def __init__(self):
        rospy.init_node('actuator')
        self.veh_name       = socket.gethostname()
        # Chassis parameters
        wheel_diameter = rospy.get_param('~wheel_diameter', 0.065)  # Example: 65 mm
        wheelbase      = rospy.get_param('~wheelbase', 0.105)  # Example: 105 mm
        self.chassis   = CHASSIS(wheel_diameter,wheelbase)

        self.debug_mode = rospy.get_param('~debug_mode', False)  # Debug mode flag

        self.usart_com = MCUcommProtocol(SerialComm('/dev/ttyAMA0', 115200, self.debug_mode))
        # shutdown hook
        rospy.on_shutdown(self.shutdown)
        self.usart_com.send_start_message()

        self.global_stop_flag   = True
        rospy.loginfo("%s: global brake activated",self.veh_name)
        self.local_stop_flag    = True
        rospy.loginfo("%s: local brake activated",self.veh_name)

        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)

        self.yaw_rate_imu            = 0
        self.sub_imu                 = rospy.Subscriber("imu", Imu, self.imu_cb, queue_size=1)

        self.chassis = CHASSIS(wheel_diameter,wheelbase)
        self.chassis.trim = self._read_trim_from_file()

        self.filter = ComplementaryFilter(alpha=0.7)  # Create a complementary filter with alpha = 0.7

        # Publishers
        self.pub_real_wheel_speeds = rospy.Publisher('real_wheel_speeds', Float32MultiArray, queue_size=10)
        self.pub_encoders = rospy.Publisher('encoder_count',Float32MultiArray,queue_size=1)
        if self.debug_mode:
            self.pub_setpoints_debug = rospy.Publisher('setpoints_debug', Float32MultiArray, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / 50.0), self.timer_callback)

        # Subscribe to the cmd_vel topic
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
    
    def estop_cb(self,msg:Bool) -> None:
        self.global_stop_flag = msg.data
        rospy.loginfo_once('%s: global brake: %s',self.veh_name,str(msg.data))

    def estop_local_cb(self,msg:Bool) -> None:
        self.local_stop_flag = msg.data
        rospy.loginfo_once('%s: local brake: %s',self.veh_name,str(msg.data))

    def imu_cb(self,msg:Imu) -> None:
        self.yaw_rate_imu = msg.angular_velocity.z

    def timer_callback(self, event):
        # Publish the message
        message = Float32MultiArray()
        self.yaw_rate_model = self.chassis.calculate_yaw_rate_from_wheelspd(self.usart_com.left_speed, self.usart_com.right_speed)
        left_spd_linear = self.usart_com.left_speed * self.chassis.wheel_diameter * 3.14
        right_spd_linear = self.usart_com.right_speed * self.chassis.wheel_diameter * 3.14
        self.yaw_rate = self.filter.update(self.yaw_rate_imu, self.yaw_rate_model)
        message.data = [left_spd_linear, right_spd_linear,self.yaw_rate]
        self.pub_real_wheel_speeds.publish(message)
        if self.debug_mode:
            rospy.loginfo("Published message: %s",message)
        self.publish_encoders_count()

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback function for /cmd_vel topic. This is called whenever a new cmd_vel message is received."""
        linear_velocity = msg.linear.x  # Forward/backward velocity
        angular_velocity = msg.angular.z  # Angular velocity (rotation)

        # Calculate left and right wheel speeds in radians per second
        # omega_left, omega_right = self.chassis.calculate_wheel_speeds(linear_velocity, angular_velocity)
        omega_left, omega_right = self.chassis.caculate_wheel_speeds_rps(linear_velocity,angular_velocity)
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
            self.usart_com.send_wheel_setpoints(0,0)
        else:
            self.usart_com.send_wheel_setpoints(omega_left,omega_right)

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

    def publish_encoders_count(self):
        message = Float32MultiArray()
        message.data = [self.usart_com.left_enc_count,self.usart_com.right_enc_count]
        self.pub_encoders.publish(message)

    def read_ack_msg(self):
        try:
            data = self.serial_conn.read(4)
            if len(data) == 4:
                identifier = data[0]
                if identifier == 0xa2:
                    self.ack_flag = True
        except Exception as e:
                rospy.logerr(f"Error reading ack messages: {e}")        

    def shutdown(self):

        rospy.loginfo("Shutting down VPAHAT and cleaning up resources.")
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