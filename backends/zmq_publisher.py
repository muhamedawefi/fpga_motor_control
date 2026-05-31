import zmq
import json

class ZMQPublisher:
    def __init__(self, port=5555):
        ctx = zmq.Context()
        self.sock = ctx.socket(zmq.PUB)
        self.sock.bind(f"tcp://*:{port}")

    def publish(self, data):
        self.sock.send_string(json.dumps(data))

