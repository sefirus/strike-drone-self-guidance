import time
from abc import ABC, abstractmethod

import board
import busio
import adafruit_bno055

from sensors.interfaces.imu_provider import IMUProvider
from sensors.interfaces.MagProvider import MagProvider
from sensors.data_structures import IMUData, MagData

class BNO055Provider(IMUProvider, MagProvider):
    """
    Uses the Adafruit BNO055 CircuitPython driver to implement
    both IMUProvider.get_latest_imu_data() and MagProvider.get_latest_mag_data().
    """

    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_bno055.BNO055_I2C(i2c)

    def get_latest_imu_data(self) -> IMUData:
        """
        Returns:
            IMUData containing:
            - orientation: 4-tuple quaternion (w, x, y, z)
            - linear_acceleration: 3-tuple m/s² (gravity subtracted)
            - angular_velocity: 3-tuple °/s (gyroscope)
            - timestamp: float seconds since epoch
        """
        quat = self._sensor.quaternion           # (w, x, y, z) :contentReference[oaicite:6]{index=6}
        lin_acc = self._sensor.linear_acceleration  # (x, y, z) in m/s² :contentReference[oaicite:7]{index=7}
        gyro   = self._sensor.gyro               # (x, y, z) in °/s :contentReference[oaicite:8]{index=8}

        return IMUData(
            orientation_q=quat,
            linear_acceleration=lin_acc,
            angular_velocity=gyro,
            timestamp=time.time()
        )

    def get_latest_mag_data(self) -> MagData:
        """
        Returns:
            MagData containing:
            - x, y, z: magnetometer in microteslas
        """
        mag = self._sensor.magnetic             # (x, y, z) in µT :contentReference[oaicite:9]{index=9}
        return MagData(x=mag[0], y=mag[1], z=mag[2])
