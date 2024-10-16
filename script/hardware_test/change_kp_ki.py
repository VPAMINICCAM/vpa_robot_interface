import serial
import struct

def reverse_bytes(data: bytes) -> bytes:
    """Reverse the byte order of the packed float."""
    return data[::-1]

def send_pid_gains(kp: float, ki: float, port='/dev/ttyAMA0', baudrate=115200):
    """
    Send the PID gains (Kp, Ki) to the MCU. The message format is:
    - 1 byte identifier: 0x05
    - 4 bytes for Kp in reversed float
    - 4 bytes for Ki in reversed float
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
        identifier = struct.pack('<B', 0x05)

        # Pack the Kp and Ki values as floats
        kp_packed = struct.pack('<f', kp)
        ki_packed = struct.pack('<f', ki)

        # Reverse the bytes of each float
        kp_reversed = reverse_bytes(kp_packed)
        ki_reversed = reverse_bytes(ki_packed)

        # Combine the identifier and the reversed floats
        message = identifier + kp_reversed + ki_reversed

        # Send the message via USART
        ser.write(message)

        # Print the reversed packed message in hexadecimal for debugging
        print(f"Sent message in hex: {message.hex()}")

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Example Kp and Ki values
    kp_value = 1.0
    ki_value = 0.5

    # Send the PID gains to the MCU
    send_pid_gains(kp_value, ki_value)
