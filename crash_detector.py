import math
import time

from sensors.data_structures import IMUData


class CrashDetector:
    """Detects a probable crash or hard impact from IMU data.

    Logic (simple but effective):
      • If linear‑acceleration magnitude exceeds ACC_THRESHOLD  _or_
      • If angular‑rate magnitude exceeds GYRO_THRESHOLD
      for at least *impact_window* seconds, flag crash.
    """
    def __init__(self, config):
        self.acc_threshold  = getattr(config, "CRASH_ACC_THRESHOLD",  30.0)   # m/s²
        self.gyro_threshold = getattr(config, "CRASH_GYRO_THRESHOLD", 5.0)    # rad/s
        self.window         = getattr(config, "CRASH_IMPACT_WINDOW", 0.05)   # seconds
        self.tripped_time   = None
        self.crashed        = False

    def update(self, imu_data: IMUData):
        if self.crashed:
            return True

        # magnitude of acceleration (already body frame)
        ax, ay, az = imu_data.linear_acceleration
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)

        # magnitude of gyro (rad/s)  note: ros IMU typically in rad/s
        gx, gy, gz = imu_data.angular_velocity
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)

        trigger = (acc_mag > self.acc_threshold) or (gyro_mag > self.gyro_threshold)

        now = time.time()
        if trigger:
            if self.tripped_time is None:
                self.tripped_time = now
            elif (now - self.tripped_time) >= self.window:
                self.crashed = True
        else:
            self.tripped_time = None  # reset

        return self.crashed
