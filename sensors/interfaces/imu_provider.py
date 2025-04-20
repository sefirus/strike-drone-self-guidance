from abc import ABC, abstractmethod
from sensors.data_structures import IMUData

class IMUProvider(ABC):
    @abstractmethod
    def get_latest_imu_data(self) -> IMUData:
        pass
