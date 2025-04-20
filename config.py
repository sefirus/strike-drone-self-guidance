import yaml
import os


class Config:
    """
    Configuration object that holds all constants.
    Values are loaded from a YAML file and can be overridden via CLI.
    """

    def __init__(self, config_dict: dict):
        self.CONNECTION = config_dict.get("connection", "udp:127.0.0.1:14550")
        self.BLACKBOX_LOG_FILE = config_dict.get("blackbox_log_file", "blackbox.log")
        self.TELEMETRY_FREQUENCY = config_dict.get("telemetry_frequency", 200)  # in Hz
        self.MAVLINK_BAUD = config_dict.get("mavlink_baud", 57600)
        self.LOG_LEVEL = config_dict.get("log_level", "INFO")

        # Camera parameters
        self.CAMERA_SOURCE = config_dict.get("camera_source", "sim")  # Options: "sim", "real"
        self.SHOW_CAMERA_WINDOW = config_dict.get("show_camera_window", True)  # Default: True

    def __repr__(self):
        """String representation for debugging."""
        return f"Config(CONNECTION={self.CONNECTION}, BLACKBOX_LOG_FILE={self.BLACKBOX_LOG_FILE}, " \
               f"TELEMETRY_FREQUENCY={self.TELEMETRY_FREQUENCY}, MAVLINK_BAUD={self.MAVLINK_BAUD}, " \
               f"LOG_LEVEL={self.LOG_LEVEL}, CAMERA_SOURCE={self.CAMERA_SOURCE}, " \
               f"SHOW_CAMERA_WINDOW={self.SHOW_CAMERA_WINDOW})"


def load_config(config_path: str, cli_args=None) -> Config:
    """
    Loads configuration from a YAML file and applies any CLI overrides.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file {config_path} not found.")
    with open(config_path, 'r') as f:
        config_dict = yaml.safe_load(f) or {}

    config = Config(config_dict)

    # Override with CLI args if provided
    if cli_args:
        if hasattr(cli_args, "connection") and cli_args.connection:
            config.CONNECTION = cli_args.connection
        if hasattr(cli_args, "blackbox_log_file") and cli_args.blackbox_log_file:
            config.BLACKBOX_LOG_FILE = cli_args.blackbox_log_file
        if hasattr(cli_args, "telemetry_frequency") and cli_args.telemetry_frequency:
            config.TELEMETRY_FREQUENCY = cli_args.telemetry_frequency
        if hasattr(cli_args, "mavlink_baud") and cli_args.mavlink_baud:
            config.MAVLINK_BAUD = cli_args.mavlink_baud
        if hasattr(cli_args, "log_level") and cli_args.log_level:
            config.LOG_LEVEL = cli_args.log_level
        if hasattr(cli_args, "camera_source") and cli_args.camera_source:
            config.CAMERA_SOURCE = cli_args.camera_source
        if hasattr(cli_args, "show_camera_window"):
            config.SHOW_CAMERA_WINDOW = cli_args.show_camera_window

    return config
