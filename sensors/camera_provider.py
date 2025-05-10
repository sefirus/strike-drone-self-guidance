import threading
import time
import cv2

from sensors.interfaces.camera_provider import CameraProvider
from sensors.data_structures import CameraFrame

class ThreadedCameraProvider(CameraProvider):
    def __init__(self, buffer_flush: bool = True):
        """
        device_index: index of the webcam (0 for default USB camera).
        buffer_flush: if True, calls cap.read() twice on start to discard old frames.
        """
        self.device_index = 0
        self.buffer_flush = buffer_flush
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_ANY)
        if not self.cap.isOpened():
            raise RuntimeError(f"Unable to open camera #{self.device_index}")
        # Optionally flush the buffer to get a fresh frame
        if self.buffer_flush:
            self.cap.read()
            self.cap.read()
        self.latest_frame = None
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                # camera read failed; wait a bit and retry
                time.sleep(0.01)
                continue
            # Wrap in your CameraFrame: (frame: np.ndarray, timestamp: float)
            self.latest_frame = CameraFrame(frame, time.time())
        # Clean up when loop exits
        self.cap.release()

    def get_latest_camera_frame(self) -> CameraFrame:
        """
        Returns the most recent CameraFrame, or None if no frame has been read yet.
        """
        return self.latest_frame

    def stop(self):
        """
        Signals the capture thread to exit and waits for it to finish.
        """
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
