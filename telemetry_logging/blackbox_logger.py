import logging

class BlackboxLogger:
    """
    Writes MAVLink telemetry messages to a dedicated log file for post-flight analysis.
    """
    def __init__(self, log_file_path="blackbox.log"):
        self.log_file_path = log_file_path
        self.logger = logging.getLogger("BlackboxLogger")
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False  # Prevent log messages from propagating to the root logger

        # Remove any existing handlers
        for handler in self.logger.handlers[:]:
            self.logger.removeHandler(handler)

        # Set up file handler only
        file_handler = logging.FileHandler(self.log_file_path)
        formatter = logging.Formatter('%(asctime)s %(message)s')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    def log_message(self, msg: dict):
        """
        Logs a MAVLink message (as a dict) to the blackbox file.
        """
        self.logger.info(msg)
