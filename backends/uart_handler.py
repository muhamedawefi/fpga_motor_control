import serial
import threading, queue
from utils import parse_telemetry

class UARTHandler:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.queue = queue.Queue()
        self.running = True

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buffer = bytearray()

        while self.running:
            try:
                 data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                 break

            buffer += data

            while len(buffer) >= 8:
                if buffer[0] != 0xFF:
                    buffer.pop(0)
                    continue

                frame = buffer[:8]
                buffer = buffer[8:]

                parsed = parse_telemetry(frame)
                if parsed:
                    self.queue.put(parsed)

    def send(self, frame: bytes):
        self.ser.write(frame)

    def get_latest(self):
        latest = None
        while not self.queue.empty():
            latest = self.queue.get()
        return latest

    def stop(self):
        self.running = False