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
        # In Config.__init__
        self.NAV_CONSTANT = config_dict.get("nav_constant", 3.0)
        self.GRAVITY = config_dict.get("gravity", 9.80665)
        self.CAMERA_H_FOV = config_dict.get("camera_h_fov", 120.0)
        self.CAMERA_V_FOV = config_dict.get("camera_v_fov", 105.0)
        self.max_tilt_angle_deg = config_dict.get("max_tilt_angle_deg", 60.0)
        self.GUIDANCE_MODE = config_dict.get("guidance_mode", "los").lower()
        self.LOS_GAIN = config_dict.get("los_gain", 3.0)
        self.ACC_SMOOTH_METHOD = config_dict.get("ACC_SMOOTH_METHOD", "window")
        self.ACC_WINDOW_SIZE = config_dict.get("ACC_WINDOW_SIZE", 4)
        self.ACC_EMA_ALPHA = config_dict.get("ACC_EMA_ALPHA", 0.7)
        self.INITIAL_TILT_DEG = config_dict.get("INITIAL_TILT_DEG", 10)
        self.TILT_RAMP_RATE_DEG_S = config_dict.get("TILT_RAMP_RATE_DEG_S", 20)
        self.MAX_TILT_DEG = config_dict.get("MAX_TILT_DEG", 60)
        self.GRAV_KOEF = config_dict.get("GRAV_KOEF", 0.1)
        self.MAG_OFFSET_YAW_RAD = config_dict.get("MAG_OFFSET_YAW_RAD", 2.9520)

        self.H_GUARD_M = config_dict.get("H_GUARD_M", 3.0)
        self.SINK_RATE_LIMIT_MS = config_dict.get("SINK_RATE_LIMIT_MS", 1.0)
        self.GRAVITY = config_dict.get("GRAVITY", 9.81)
        self.RAV_KOEF = config_dict.get("RAV_KOEF", 1.0)
        self.HOVER_THROTTLE = config_dict.get("HOVER_THROTTLE", 0.55)
        self.ALT_P = config_dict.get("ALT_P", 0.12)
        self.ALT_D = config_dict.get("ALT_D", 0.05)
        self.REL_ALT_MAX = config_dict.get("REL_ALT_MAX", 10.0)
        self.CAMERA_H_FOV = config_dict.get("CAMERA_H_FOV", 78)
        self.CAMERA_V_FOV = config_dict.get("CAMERA_V_FOV", 60)
        self.LOS_KOEF = config_dict.get("LOS_KOEF", 1.4)

        self.BORDER_FRAC = config_dict.get("BORDER_FRAC", 0.80)
        self.GAIN_XY = config_dict.get("GAIN_XY", 0.40)
        self.GAIN_Z = config_dict.get("GAIN_Z", 0.60)
        self.ACCEL_GAIN = config_dict.get("ACCEL_GAIN", 0.15)


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
