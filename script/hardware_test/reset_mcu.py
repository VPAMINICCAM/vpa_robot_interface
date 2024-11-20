#!/usr/bin/env python3

import serial
import time

def send_reset_command(port, baudrate):
    try:
        # Initialize the serial connection
        ser = serial.Serial(port, baudrate, timeout=1)

        # Reset command: 02 01 15 03
        reset_command = bytes([0x02, 0x01, 0x15, 0x03])

        # Send the reset command
        ser.write(reset_command)
        print("Reset command sent:", reset_command.hex())

        # Wait for acknowledgment
        time.sleep(0.1)  # Allow some time for the MCU to respond
        ack = ser.read(4)  # Read the acknowledgment (4 bytes expected)

        if ack == bytes([0x02, 0x05, 0x16, 0x03]):
            print("Acknowledgment received:", ack.hex())
        else:
            print("No valid acknowledgment received. Received:", ack.hex() if ack else "None")

        # Optional: Wait to ensure reset happens
        time.sleep(2)

        # Cleanup
        ser.close()

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Port and baudrate configuration
    SERIAL_PORT = "/dev/ttyAMA0"  # Replace with your serial port
    BAUDRATE = 115200             # Ensure this matches the STM32 configuration

    send_reset_command(SERIAL_PORT, BAUDRATE)
