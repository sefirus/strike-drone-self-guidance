#!/usr/bin/env python3
import threading
import signal
import time
import logging
import sys

from cli import parse_cli_args
from config import load_config
from mavlink_connection import establish_mavlink_connection
from flight_control import FlightControlThread
from telemetry_logging.telemetry_logging import TelemetryLoggingThread

def main():
    # Parse CLI arguments (all CLI-related defaults and options are in cli.py)
    args = parse_cli_args()

    # Load configuration (all default values and parameters are in config.py)
    # config = load_config(args.config)
    config = load_config('/home/sefirus/uavs/strike-drone-self-guidance/config.yaml')
    # Configure logging using configuration parameters.
    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(threadName)s: %(message)s"
    )
    logging.info("Starting Kamikaze Drone System")

    # Establish the MAVLink connection (placeholder function)
    try:
        mav_conn, connection_lock = establish_mavlink_connection(config, args)
    except Exception as e:
        logging.error(f"Unable to establish MAVLink connection: {e}")
        sys.exit(1)

    # Global shutdown event for all threads
    shutdown_event = threading.Event()

    # Instantiate threads with placeholder implementations
    flight_thread = FlightControlThread(mav_conn, connection_lock, shutdown_event, config)
    telemetry_thread = TelemetryLoggingThread(mav_conn, shutdown_event, config)

    # Start threads
    flight_thread.start()
    telemetry_thread.start()

    # Graceful shutdown on SIGINT/SIGTERM
    def handle_signal(sig, frame):
        logging.info("Shutdown signal received. Terminating threads...")
        shutdown_event.set()
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Wait for threads to finish
    try:
        while flight_thread.is_alive() or telemetry_thread.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown_event.set()

    flight_thread.join()
    telemetry_thread.join()
    logging.info("Kamikaze Drone System shutdown complete.")

if __name__ == "__main__":
    main()
