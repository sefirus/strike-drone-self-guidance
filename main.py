#!/usr/bin/env python3
import threading
import signal
import time
import logging
import sys

import matplotlib.pyplot as plt

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

    # plt.ion()
    # fig, ax = plt.subplots(figsize=(10, 6))
    # line_x, = ax.plot([], [], label='X accel (mG)')
    # line_y, = ax.plot([], [], label='Y accel (mG)')
    # line_z, = ax.plot([], [], label='Z accel (mG)')
    # ax.set_xlabel('Time since start (s)')
    # ax.set_ylabel('Acceleration (mG)')
    # ax.set_title('Real‑Time Smoothed Accelerations')
    # ax.legend(loc='upper right')
    # ax.grid(True)
    #
    # times, xs, ys, zs = [], [], [], []

    # --- main loop: poll smoothed data and update plot ---
    try:
        a = 5
        # while not shutdown_event.is_set():
            # sm = telemetry_thread.telemetry_listener.get_latest_smoothed()
            # if sm:
            #     times.append(sm['t'])
            #     xs.append(sm['x'])
            #     ys.append(sm['y'])
            #     zs.append(sm['z'])
            #
            #     line_x.set_data(times, xs)
            #     line_y.set_data(times, ys)
            #     line_z.set_data(times, zs)
            #
            #     ax.relim()
            #     ax.autoscale_view()
            #
            #     plt.draw()
            #     plt.pause(0.01)

            # small sleep to avoid busy‑spin
            # time.sleep(0.01)

    except Exception as e:
        logging.error(f"Plotting loop error: {e}")

    finally:
        # turn interactive mode off and show final frame
        plt.ioff()
        plt.show()

        # wait for threads to finish
        flight_thread.join()
        telemetry_thread.join()
        logging.info("Kamikaze Drone System shutdown complete.")

    flight_thread.join()
    telemetry_thread.join()
    logging.info("Kamikaze Drone System shutdown complete.")

if __name__ == "__main__":
    main()
