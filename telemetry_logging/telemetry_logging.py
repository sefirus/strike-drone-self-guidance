import threading
import logging
from telemetry_logging.telemetry_listener import TelemetryListener
from telemetry_logging.blackbox_logger import BlackboxLogger

class TelemetryLoggingThread(threading.Thread):
    """
    Runs the telemetry listener in a dedicated thread.
    This thread is responsible for receiving MAVLink telemetry, caching the latest data,
    and logging all messages via the BlackboxLogger.
    """
    def __init__(self, mav_conn, shutdown_event: threading.Event, config):
        """
        Args:
            mav_conn: The pymavlink connection object.
            shutdown_event: A threading.Event for graceful shutdown.
            config: Configuration object (expects attribute BLACKBOX_LOG_FILE for the log file path).
        """
        super().__init__(name="TelemetryLoggingThread")
        self.mav_conn = mav_conn
        self.shutdown_event = shutdown_event
        self.config = config
        log_path = getattr(config, "BLACKBOX_LOG_FILE", "blackbox.log")
        self.blackbox_logger = BlackboxLogger(log_file_path=log_path)
        self.telemetry_listener = TelemetryListener(mav_conn, shutdown_event, self.blackbox_logger)

    def run(self):
        logging.info("Telemetry Logging Thread started.")
        self.telemetry_listener.listen()
        logging.info("Telemetry Logging Thread exiting.")
