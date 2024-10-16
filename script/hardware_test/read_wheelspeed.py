import serial
import struct

def read_wheel_speeds(port='/dev/ttyAMA0', baudrate=115200):
    """
    Test script to read real wheel speeds from the MCU. 
    The message format is:
    - 1 byte identifier: 0x04
    - 4 bytes for left wheel speed (A) in float
    - 4 bytes for right wheel speed (B) in float
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

        print("Waiting for data from the MCU...")

        while True:
            # Read 9 bytes (1 identifier byte + 4 bytes for each float)
            data = ser.read(9)

            if len(data) == 9:
                identifier = data[0]
                if identifier == 0x04:
                    # Unpack the next 8 bytes as two floats
                    left_speed = struct.unpack('<f', data[1:5])[0]
                    right_speed = struct.unpack('<f', data[5:9])[0]

                    # Print the received wheel speeds
                    print(f"Received real wheel speeds: Left = {left_speed:.2f} rps, Right = {right_speed:.2f} rps")
                else:
                    print(f"Unexpected identifier: {identifier}")
            else:
                print("Incomplete data received")

    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        # Close the serial connection
        ser.close()

if __name__ == "__main__":
    # Run the test function
    read_wheel_speeds()
