class BackendInterface:
    def set_speed(self, value):
        raise NotImplementedError

    def start_stream(self):
        raise NotImplementedError

    def stop_stream(self):
        raise NotImplementedError

    def get_data(self):
        raise NotImplementedError