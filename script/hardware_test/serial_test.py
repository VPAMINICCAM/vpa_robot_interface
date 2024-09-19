import serial
import time

# Initialize the UART connection on /dev/ttyAMA0
ser = serial.Serial(
    port='/dev/ttyAMA0',    # Use ttyAMA0 for communication
    baudrate=115200,        # Set the baud rate
    bytesize=serial.EIGHTBITS,  # 8 data bits
    parity=serial.PARITY_NONE,  # No parity
    stopbits=serial.STOPBITS_ONE,  # 1 stop bit
    timeout=1               # Timeout for reading
)

# The hex code to send: 0x3D 00 00 00 00
hex_code = bytes([0x3D, 0x00, 0x00, 0x00, 0x00])

try:
    # Send the hex code 6 times
    for _ in range(6):
        ser.write(hex_code)  # Send the hex code
        print(f"Sent: {hex_code.hex()}")  # Print sent data in hex format
        time.sleep(0.5)  # Wait for 500 milliseconds between sends
    
    # After sending 6 times, check for a 5-byte response
    print("Checking for response...")
    
    time.sleep(1)  # Wait briefly to allow time for the response
    
    if ser.in_waiting >= 5:  # Check if at least 5 bytes are available
        response = ser.read(5)  # Read 5 bytes from the serial port
        if response[0] == 0x3E:  # Check if the first byte is 0x3E
            print(f"Received valid response: {response.hex()}")
        else:
            print(f"Received invalid response: {response.hex()}")
    else:
        print("No response received")
        
finally:
    ser.close()  # Close the serial connection
    print("Serial connection closed.")
