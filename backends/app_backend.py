import asyncio
import threading

class AppBackend:
    def __init__(self, uart):
        self.uart = uart
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

    def _run(self, coro):
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=5.0)

    def set_speed(self, value):
        self._run(self.uart.write(0x00, int(value * 128)))

    def enable_stream(self):
        self._run(self.uart.enable_streaming(1))

    def get_data(self):
        return self._run(self.uart.read_telemetry())
