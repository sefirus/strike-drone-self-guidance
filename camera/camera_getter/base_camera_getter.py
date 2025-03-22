class BaseCameraGetter:
    def start(self):
        raise NotImplementedError

    def get_frame(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError