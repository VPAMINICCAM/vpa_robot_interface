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

def main():
    try:
        while True:
            # Send 'Hello World' message
            ser.write(b'Hello World\n')  # Send bytes over the serial connection
            print("Sent: Hello World")
            
            # Wait for 1 second before sending again
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping script...")

    finally:
        # Ensure the serial connection is closed on exit
        ser.close()

if __name__ == '__main__':
    main()