import time
import logging
import threading

class TelemetryListener:
    """
    Listens for MAVLink telemetry messages and caches the latest ATTITUDE data.
    Also forwards each message to the BlackboxLogger.
    """
    def __init__(self, mav_conn, shutdown_event: threading.Event, blackbox_logger, telemetry_frequency=10):
        """
        Args:
            mav_conn: The pymavlink connection object.
            shutdown_event: threading.Event for graceful shutdown.
            blackbox_logger: Instance of BlackboxLogger to log messages.
            telemetry_frequency: Frequency (in Hz) at which to poll messages.
        """
        self.mav_conn = mav_conn
        self.shutdown_event = shutdown_event
        self.blackbox_logger = blackbox_logger
        self.telemetry_frequency = telemetry_frequency  # Hz
        self.latest_attitude = None  # Cached ATTITUDE data.
        self.lock = threading.Lock()

    def listen(self):
        """
        Continuously retrieves non-blocking ATTITUDE messages, caches the latest,
        and logs them via BlackboxLogger.
        """
        interval = 1.0 / self.telemetry_frequency
        while not self.shutdown_event.is_set():
            try:
                # Listen for ATTITUDE messages (angles, not quaternions)
                msg = self.mav_conn.recv_match(type="ATTITUDE", blocking=False)
                if msg is not None:
                    msg_dict = msg.to_dict()
                    with self.lock:
                        self.latest_attitude = msg_dict
                    self.blackbox_logger.log_message(msg_dict)
            except Exception as e:
                logging.error(f"Error processing MAVLink message: {e}")
            time.sleep(interval)

    def get_latest_attitude(self) -> dict:
        """
        Returns:
            dict: Latest cached ATTITUDE data, or None if unavailable.
        """
        with self.lock:
            return self.latest_attitude
