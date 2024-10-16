import serial
import struct

def send_wheel_setpoints(omega_left: float, omega_right: float, port='/dev/ttyAMA0', baudrate=115200):
    """
    Pack the identifier and the wheel speeds into a binary message, reverse the bytes,
    and send it via USART.
    """
    try:
        # Initialize the serial connection
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        # Pack the data with identifier 0x03 and wheel speeds as 4-byte floats
        message = struct.pack('<Bff', 0x03, omega_left, omega_right)

        # Reverse the byte order of the entire message
        reversed_message = message[::-1]

        # Send the reversed message over the serial connection
        ser.write(reversed_message)

        # Print the reversed packed message in hexadecimal for debugging
        print("Sent reversed message in hex:", reversed_message.hex())

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Test with wheel speeds of 1.0 rps for both wheels
    send_wheel_setpoints(1.0, 1.0)
