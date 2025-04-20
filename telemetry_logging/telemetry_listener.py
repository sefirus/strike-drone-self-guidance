# telemetry_listener.py

import time
import logging
import threading
from collections import deque
from pymavlink import mavutil

class TelemetryListener:
    """
    Listens for MAVLink ATTITUDE and SCALED_IMU messages,
    logs raw data, smooths accelerations, and publishes
    the latest smoothed accel for external polling.
    """
    def __init__(self, mav_conn, shutdown_event: threading.Event,
                 blackbox_logger, telemetry_frequency=200,
                 smoothing_window=10):
        self.mav_conn           = mav_conn
        self.shutdown_event     = shutdown_event
        self.blackbox_logger    = blackbox_logger
        self.telemetry_frequency= telemetry_frequency
        self.window_size        = smoothing_window

        # buffers for moving average
        self._x_win = deque(maxlen=self.window_size)
        self._y_win = deque(maxlen=self.window_size)
        self._z_win = deque(maxlen=self.window_size)

        # latest published smoothed value
        self.latest_smoothed = None  # {'t':…, 'x':…, 'y':…, 'z':…}

        # lock for both attitude and smoothed accel
        self.lock = threading.Lock()
        self.latest_attitude = None

        self._start_time = time.time()

        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            100000,  # Interval in microseconds (10 Hz)
            0, 0, 0, 0, 0)

    def listen(self):
        interval = 1.0 / self.telemetry_frequency
        while not self.shutdown_event.is_set():
            # print('try_message')
            try:
                # ATTITUDE
                att = self.mav_conn.recv_match(type="ATTITUDE", blocking=False)
                if att:
                    d = att.to_dict()
                    #with self.lock:
                    self.latest_attitude = d
                    self.blackbox_logger.log_message(d)

                # SCALED_IMU
                imu = self.mav_conn.recv_match(type="RAW_IMU", blocking=False)
                if imu:
                    d = imu.to_dict()
                    self.blackbox_logger.log_message(d)

                    # update smoothing buffers
                    self._x_win.append(imu.xacc)
                    self._y_win.append(imu.yacc)
                    self._z_win.append(imu.zacc)

                    # once full, compute moving average and publish
                    if len(self._x_win) == self.window_size:
                        t = time.time() - self._start_time
                        x_avg = sum(self._x_win) / self.window_size
                        y_avg = sum(self._y_win) / self.window_size
                        z_avg = sum(self._z_win) / self.window_size
                        sm = {'t': t, 'x': x_avg, 'y': y_avg, 'z': z_avg}

                        # with self.lock:
                        self.latest_smoothed = sm

            except Exception as e:
                logging.error(f"Error processing MAVLink message: {e}")

            time.sleep(0.1)

    def get_latest_attitude(self):
        with self.lock:
            return self.latest_attitude

    def get_latest_smoothed(self):
        """
        Returns the most recent smoothed accel dict:
        { 't': seconds_since_start,
          'x': x_accel_mG,
          'y': y_accel_mG,
          'z': z_accel_mG }
        or None if not yet available.
        """
        with self.lock:
            return self.latest_smoothed
