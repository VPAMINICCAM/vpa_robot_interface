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

    def set_read_message(self, custom_read_message):
        """
        Sets the custom `_read_message` function dynamically.
        
        :param custom_read_message: Function to use as the `_read_message` logic.
        """
        self._read_message = custom_read_message
