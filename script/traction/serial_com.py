import struct

class SerialComm:
    def __init__(self, port, baudrate, debug_mode=False):
        self.port = port
        self.baudrate = baudrate
        self.debug_mode = debug_mode
        self.serial_conn = None
        self._initialize_serial()

    def _initialize_serial(self):
        import serial
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
        except serial.SerialException as e:
            raise Exception(f"Failed to initialize serial: {e}")

    def send_message(self, cmd_id, *data):
        start_marker = 0x02
        end_marker = 0x03
        payload = bytearray()
        for value in data:
            payload.extend(struct.pack('<f', value))
        length = 1 + len(payload)
        message = bytearray([start_marker, length, cmd_id]) + payload + bytearray([end_marker])
        self.serial_conn.write(message)
        if self.debug_mode:
            print(f"Sent cmd_id {cmd_id}, data: {data} (Raw: {message.hex()})")

    def read_message(self):
        try:
            start = self.serial_conn.read(1)
            if not start or start[0] != 0x02:
                return None
            length = self.serial_conn.read(1)[0]
            message = self.serial_conn.read(length + 1)
            if message[-1] != 0x03:
                return None
            return bytearray([0x02, length]) + message
        except Exception as e:
            raise Exception(f"Failed to read message: {e}")
