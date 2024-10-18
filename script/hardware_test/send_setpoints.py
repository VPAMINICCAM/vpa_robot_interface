import serial
import struct
import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM (Broadcom chip pin numbering)
GPIO.setmode(GPIO.BCM)

# Set GPIO23 as an output pin
GPIO.setup(23, GPIO.OUT)

def reverse_bytes(data: bytes) -> bytes:
    """Reverse the byte order of the packed float."""
    return data[::-1]

def send_wheel_setpoints(omega_left: float, omega_right: float, port='/dev/ttyAMA0', baudrate=115200):
    """
    Pack the identifier and the wheel speeds into a binary message, reverse the byte order
    of the float values, and send it via USART.
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

        # Pack the identifier
        identifier = struct.pack('<B', 0x03)

        # Pack the float values separately and reverse their byte order
        omega_left_packed = struct.pack('<f', omega_left)  # Little-endian float
        omega_right_packed = struct.pack('<f', omega_right)  # Little-endian float

        # Reverse the bytes of each float
        omega_left_reversed = reverse_bytes(omega_left_packed)
        omega_right_reversed = reverse_bytes(omega_right_packed)

        # Combine the identifier and the reversed float bytes
        message = identifier + omega_left_reversed + omega_right_reversed

        # Send the message via USART
        ser.write(message)

        # Print the reversed packed message in hexadecimal for debugging
        print("Sent message in hex:", message.hex())

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Test with wheel speeds of 1.0 rps for both wheels
    GPIO.output(23, GPIO.HIGH)
    print("GPIO23 is set to HIGH")
    send_wheel_setpoints(1.0, 1.0)
    time.sleep(10)
    GPIO.cleanup()