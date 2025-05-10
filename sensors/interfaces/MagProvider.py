from abc import ABC, abstractmethod
from sensors.data_structures import MagData


class MagProvider(ABC):
    @abstractmethod
    def get_latest_mag_data(self) -> MagData:
        pass
