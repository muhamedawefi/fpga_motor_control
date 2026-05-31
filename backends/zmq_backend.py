import zmq
import json
from backends.backend_interface import BackendInterface

_zmq_context = None
_cmd_socket  = None
_sub_socket  = None

def _get_sockets(port, cmd_port):
    global _zmq_context, _cmd_socket, _sub_socket
    if _zmq_context is None:
        _zmq_context = zmq.Context()
        _sub_socket  = _zmq_context.socket(zmq.SUB)
        _sub_socket.setsockopt(zmq.CONFLATE, 1)
        _sub_socket.connect(f"tcp://localhost:{port}")
        _sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        _sub_socket.setsockopt(zmq.RCVTIMEO, 100)
        _cmd_socket  = _zmq_context.socket(zmq.PUB)
        _cmd_socket.bind(f"tcp://*:{cmd_port}")
    return _sub_socket, _cmd_socket

class ZMQBackend(BackendInterface):
    def __init__(self, port=5555, cmd_port=5556):
        self.sock, self.cmd = _get_sockets(port, cmd_port)

    def set_speed(self, value):
        msg = json.dumps({"setpoint": value})
        self.cmd.send_string(msg)
        print(f"ZMQ SENT: {msg}")

    def start_stream(self):
        pass

    def stop_stream(self):
        pass

    def get_data(self):
        try:
            msg = self.sock.recv_string()
            print(f"ZMQ RECV: {msg[:50]}")
            return json.loads(msg)
        except zmq.Again:
            return None
