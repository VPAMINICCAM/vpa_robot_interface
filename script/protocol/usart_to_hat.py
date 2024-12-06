import rospy
import struct

from hardware.serial_com import SerialComm

class MCUcommProtocol:

    def __init__(self,usart_com:SerialComm,debug_mode=False):
        self.serial_comm = usart_com
        self.debug_mode  = debug_mode
        self.ack_flag = False
        # assign protocol level read 
        # this function will strip the format part and pass the main part to the process part
        self.serial_comm.set_read_message(custom_read_message=self.pro_read_message)

        self.omega_id       = 0x07
        self.throttle_id    = 0x09
        self.steer_id       = 0x19

        self.deadzone_id    = 0x11
        self.pid_id         = 0x13
        self.shutdown_id    = 0x17

        self.speed = 0

        self.serial_comm.set_read_callback(self.process_usart_message)

    def send_start_message(self) -> bool:
        """
        Send the reset message with cmd_id 0x15 and validate the reply.
        """
        try:
            # Send reset message
            
            rospy.loginfo("start message (cmd_id=0x01) sent. Waiting for reply...")

            for i in range(100):
                self.send_message(cmd_id=0x01)
                rospy.sleep(0.2)
                # Wait for the reply)
                if self.ack_flag:
                    rospy.loginfo("MCU start acknowledged (cmd_id=0x04).")
                    return True
            if i > 99:
                rospy.signal_shutdown('Unable to start communication, please try manual reset')
                return False

        except Exception as e:
            rospy.logerr(f"Error sending reset message: {e}") 
            return False
    
    def send_speed_reading_message(self):
        try:
            rospy.loginfo("speed feedback start message (cmd_id=0x0a) sent.")
            self.send_message(cmd_id=0x0a)
        except Exception as e:
            rospy.logerr(f"Error sending reset message: {e}")

    def send_message(self, cmd_id, *data):
        """
        Send a message over USART following the protocol.

        Args:
            cmd_id (int): Command identifier for the message.
            *data (float): Variable number of float values to send as the payload.
        """
        try:
            # Protocol format: [START_MARKER][LENGTH][CMD_ID][DATA...][END_MARKER]
            start_marker    = 0x02
            end_marker      = 0x03

            # Convert all float data to little-endian format
            payload = bytearray()
            for value in data:
                payload.extend(struct.pack('<f', value))  # Pack each float as IEEE 754

            # Calculate length dynamically (1 byte for CMD_ID + size of payload)
            length = 1 + len(payload)

            # Build the message
            message = bytearray([start_marker, length, cmd_id]) + payload + bytearray([end_marker])

            # Send the message over USART
            self.serial_comm.serial_conn.write(message)

            if self.debug_mode:
                print(f"Sent cmd_id {cmd_id}, data: {data} (Raw: {message.hex()})")

        except Exception as e:
            if self.debug_mode:
                print(f"Failed to send message: {e}")

    def pro_read_message(self):
        """
        Read a full message based on the protocol.

        Returns:
            bytearray: The complete message if valid, otherwise None.
        """
        try:
            # Wait for the start marker
            start = self.serial_comm.serial_conn.read(1)
            if not start or start[0] != 0x02:  # Start marker check
                return None

            # Read the length byte
            length_byte = self.serial_comm.serial_conn.read(1)
            if not length_byte:
                return None
            length = length_byte[0]
            # Read the remaining bytes (length + end marker)
            message = self.serial_comm.serial_conn.read(length + 1)
            if len(message) != length + 1 or message[-1] != 0x03:  # End marker check
                return None

            # Return the full message
            return bytearray([0x02]) + bytearray([length]) + message
        except Exception as e:
            if self.debug_mode:
                print(f"Error reading message: {e}")
            return None
        
    def process_usart_message(self, message):
        """
        Process a received USART message.
        """
        try:
            cmd_id = message[2]
            
            # Define a dictionary mapping cmd_id to their handler methods
            cmd_handlers = {
                self.omega_id: self.handle_speed_message,  # Speed message
                0x04: self.handle_ack_start,
            }

            # Get the handler for the received cmd_id
            handler = cmd_handlers.get(cmd_id, self.handle_unknown_message)

            # Call the handler with the message
            handler(message)

        except Exception as e:
            rospy.logerr(f"Error processing USART message: {e}")

    def handle_ack_start(self,message):        
        self.ack_flag = True

    def handle_speed_message(self, message):
        """
        Handle speed update messages (cmd_id = 0x02).
        """
        speed = struct.unpack('<f', message[3:7])[0]
        self.speed = speed
        if self.debug_mode:
            rospy.loginfo(f"Received speed: {speed:.2f}")

    def handle_unknown_message(self, message):
        """
        Handle unknown or unsupported messages.
        """
        cmd_id = message[2]
        rospy.logwarn(f"Unknown cmd_id received: {cmd_id}")