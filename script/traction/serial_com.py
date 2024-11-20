import struct
import threading


class SerialComm:
    """
    A class to handle serial communication with a USART device, including message reading, validation,
    and continuous reading with callbacks for message processing.
    """

    def __init__(self, port, baudrate, debug_mode=False):
        """
        Initialize the serial communication.

        Args:
            port (str): The serial port to use (e.g., '/dev/ttyAMA0').
            baudrate (int): The baud rate for the connection (e.g., 115200).
            debug_mode (bool): If True, enables debug logging.
        """
        self.port = port
        self.baudrate = baudrate
        self.debug_mode = debug_mode
        self.serial_conn = None
        self.read_callback = None  # Callback function to process messages

        self._initialize_serial()

        # Start the continuous reading thread
        self._reading_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reading_thread.start()

    def _initialize_serial(self):
        """
        Initialize the serial connection.
        """
        import serial
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            if self.debug_mode:
                print(f"Initialized serial connection on {self.port} with baudrate {self.baudrate}")
        except serial.SerialException as e:
            raise Exception(f"Failed to initialize serial: {e}")

    def set_read_callback(self, callback):
        """
        Set the callback function to process received messages.

        Args:
            callback (function): A function to process messages. It should accept a `bytearray` as its argument.
        """
        self.read_callback = callback

    def send_message(self, cmd_id, *data):
        """
        Send a message over USART following the protocol.

        Args:
            cmd_id (int): Command identifier for the message.
            *data (float): Variable number of float values to send as the payload.
        """
        try:
            # Protocol format: [START_MARKER][LENGTH][CMD_ID][DATA...][END_MARKER]
            start_marker = 0x02
            end_marker = 0x03

            # Convert all float data to little-endian format
            payload = bytearray()
            for value in data:
                payload.extend(struct.pack('<f', value))  # Pack each float as IEEE 754

            # Calculate length dynamically (1 byte for CMD_ID + size of payload)
            length = 1 + len(payload)

            # Build the message
            message = bytearray([start_marker, length, cmd_id]) + payload + bytearray([end_marker])

            # Send the message over USART
            self.serial_conn.write(message)

            if self.debug_mode:
                print(f"Sent cmd_id {cmd_id}, data: {data} (Raw: {message.hex()})")

        except Exception as e:
            if self.debug_mode:
                print(f"Failed to send message: {e}")

    def _read_loop(self):
        """
        Continuously read messages from USART and process them with the read_callback.
        """
        while True:
            try:
                message = self._read_message()
                if message:
                    if self.read_callback:
                        self.read_callback(message)
                    elif self.debug_mode:
                        print("Warning: No read_callback set. Message received but not processed.")
            except Exception as e:
                if self.debug_mode:
                    print(f"Error in read loop: {e}")

    def _read_message(self):
        """
        Read a full message based on the protocol.

        Returns:
            bytearray: The complete message if valid, otherwise None.
        """
        try:
            # Wait for the start marker
            start = self.serial_conn.read(1)
            if not start or start[0] != 0x02:  # Start marker check
                return None

            # Read the length byte
            length_byte = self.serial_conn.read(1)
            if not length_byte:
                return None
            length = length_byte[0]

            # Read the remaining bytes (length + end marker)
            message = self.serial_conn.read(length + 1)
            if len(message) != length + 1 or message[-1] != 0x03:  # End marker check
                return None

            # Return the full message
            return bytearray([0x02]) + bytearray([length]) + message
        except Exception as e:
            if self.debug_mode:
                print(f"Error reading message: {e}")
            return None
