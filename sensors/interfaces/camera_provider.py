from abc import ABC, abstractmethod
from sensors.data_structures import CameraFrame

class CameraProvider(ABC):
    @abstractmethod
    def get_latest_camera_frame(self) -> CameraFrame:
        pass
