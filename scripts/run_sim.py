from backends.sim_backend import SimBackend
import time

backend = SimBackend()

backend.start_stream()

backend.set_speed(100)

for _ in range(200):
    data = backend.get_data()

    if data:
        print(
            f"{data['speed']:.2f}, {data['current']:.2f}, {data['vcmd']:.2f}"
        )

    time.sleep(0.01)