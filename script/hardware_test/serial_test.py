import serial
import time

# Configure the UART serial connection
ser = serial.Serial(
    port='/dev/serial0',    # Use the Pi's onboard UART
    baudrate=115200,        # Baud rate of 115200
    bytesize=serial.EIGHTBITS,  # 8 data bits
    parity=serial.PARITY_NONE,  # No parity
    stopbits=serial.STOPBITS_ONE,  # 1 stop bit
    timeout=1               # Timeout for reading
)

def send_data(data):
    """
    Function to send data over serial.
    """
    ser.write(data.encode('utf-8'))  # Send data encoded as utf-8
    print(f"Sent: {data}")

def receive_data():
    """
    Function to receive data from serial.
    """
    if ser.in_waiting > 0:  # Check if there is data waiting
        data = ser.readline().decode('utf-8').rstrip()
        print(f"Received: {data}")
        return data
    return None

if __name__ == "__main__":
    try:
        while True:
            # Send test data
            send_data("Hello from Raspberry Pi!")
            
            # Wait a second
            time.sleep(1)
            
            # Check if any data is received
            received = receive_data()
            if received:
                print(f"Echo: {received}")
            
            # Pause for a moment before sending again
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting program")

    finally:
        # Close the serial connection when done
        ser.close()
        print("Serial connection closed.")
