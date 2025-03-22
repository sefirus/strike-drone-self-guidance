import argparse

def parse_cli_args():
    parser = argparse.ArgumentParser(description="Kamikaze Drone System CLI")
    parser.add_argument("--connection", type=str, default="udp:127.0.0.1:14550",
                        help="MAVLink connection string (default: udp:127.0.0.1:14550)")
    parser.add_argument("--config", type=str, default="config.yaml",
                        help="Path to the configuration YAML file (default: config.yaml)")
    parser.add_argument("--blackbox-log-file", type=str,
                        help="Override the default blackbox log file path")
    parser.add_argument("--telemetry-frequency", type=int,
                        help="Override telemetry listener frequency in Hz")
    parser.add_argument("--mavlink-baud", type=int,
                        help="Override MAVLink baud rate (default defined in config)")
    parser.add_argument("--log-level", type=str, default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        help="Logging level (default: INFO)")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in simulation mode (no actual flight controller)")
    # New camera-related CLI options:
    parser.add_argument("--camera-source", type=str, default="sim",
                        choices=["sim", "real"],
                        help="Select camera source type (sim or real), default: sim")
    parser.add_argument("--show-camera-window", dest="show_camera_window", action="store_true",
                        help="Show camera output window (default: True)")
    parser.add_argument("--no-show-camera-window", dest="show_camera_window", action="store_false",
                        help="Do not show camera output window")
    parser.set_defaults(show_camera_window=True)
    return parser.parse_args()
