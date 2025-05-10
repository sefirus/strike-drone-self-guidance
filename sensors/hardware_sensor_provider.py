# sensors/hardware_sensor_provider.py
import threading

from sensors.camera_provider import ThreadedCameraProvider
from sensors.interfaces.camera_provider import CameraProvider
from sensors.interfaces.imu_provider import IMUProvider
from sensors.interfaces.MagProvider import MagProvider
from sensors.data_structures import CameraFrame, IMUData, MagData

from sensors.bno055_provider import BNO055Provider                       # the class we built earlier


class HardwareSensorProvider(CameraProvider, IMUProvider, MagProvider):
    """
    Thin wrapper that bundles a USB webcam (ThreadedCameraProvider)
    and a BNO055 IMU/magnetometer (BNO055Provider) so it looks exactly
    like your SimSensorProvider to the rest of the system.
    """

    def __init__(self, config):
        self.camera = ThreadedCameraProvider()
        self.imu_mag = BNO055Provider()

    # --- lifecycle ---------------------------------------------------------
    def start(self):
        """ThreadedCameraProvider already starts itself, nothing extra needed."""
        pass

    def stop(self):
        self.camera.stop()

    # --- interface implementations -----------------------------------------
    def get_latest_camera_frame(self) -> CameraFrame:
        return self.camera.get_latest_camera_frame()

    def get_latest_imu_data(self) -> IMUData:
        return self.imu_mag.get_latest_imu_data()

    def get_latest_mag_data(self) -> MagData:
        return self.imu_mag.get_latest_mag_data()
