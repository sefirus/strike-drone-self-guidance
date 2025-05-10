from dataclasses import dataclass
import numpy as np

@dataclass
class CameraFrame:
    frame: np.ndarray
    timestamp: float

@dataclass
class IMUData:
    orientation_q: tuple
    linear_acceleration: tuple
    angular_velocity: tuple
    timestamp: float

@dataclass
class MagData:
    x: float
    y: float
    z: float
