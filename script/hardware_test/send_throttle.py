#!/usr/bin/env python3

import serial
import struct
import time


class DirectMCUCommand:
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
        """
        Initialize the class to control the GPIO and serial communication.
        """
        self.port = port
        self.baudrate = baudrate

        # Initialize serial connection
        self.serial_conn = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )

    def enable_actuator(self):
        """
        High GPIO23 and send actuator enable command.
        """
        time.sleep(0.1)  # Allow some time for the system to stabilize

        # Enable actuator command: 02 01 01 03
        message = bytearray([0x02, 0x01, 0x01, 0x03])
        self.serial_conn.write(message)
        print("Sent enable actuator command: 02 01 01 03")

    def send_throttle(self, throttle_value):
        """
        Send throttle command to the MCU.

        Args:
            throttle_value (float): The throttle value to send (e.g., 0.2).
        """
        # Throttle command: 02 05 09 [THROTTLE_FLOAT] 03
        start_marker = 0x02
        cmd_id = 0x09
        end_marker = 0x03
        length = 0x05  # cmd_id (1 byte) + throttle (4 bytes)

        # Convert the throttle value to little-endian float
        throttle_bytes = struct.pack('<f', throttle_value)

        # Build the message
        message = bytearray([start_marker, length, cmd_id]) + throttle_bytes + bytearray([end_marker])
        self.serial_conn.write(message)
        print(f"Sent throttle command: {message.hex()}")

    def cleanup(self):
        """
        Clean up resources (GPIO and serial connection).
        """
        if self.serial_conn.is_open:
            self.serial_conn.close()
        print("Cleaned up resources.")


if __name__ == "__main__":
    try:
        mcu_commander = DirectMCUCommand()

        # Enable actuator
        mcu_commander.enable_actuator()

        # Send throttle value 0.2
        throttle_value = 0.2
        mcu_commander.send_throttle(throttle_value)
        time.sleep(20)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Cleanup resources
        mcu_commander.cleanup()
