import threading
import logging
import cv2

from mavlink_command_sender import MavlinkCommandSender
from sensors.hardware_sensor_provider import HardwareSensorProvider
from sensors.sim_sensor_provider import SimSensorProvider
from camera.target_selector.sim_target_selector import SimTargetSelector
from camera.target_selector.rc_target_selector import RealTargetSelector
from camera.target_tracker import TargetTracker
from trajectory_planning.trajectory_planning import TrajectoryPlanner


class FlightControlThread(threading.Thread):
    def __init__(self, mav_conn, connection_lock, shutdown_event, config):
        super().__init__(name="FlightControlThread")
        self.mav_conn = mav_conn
        self.connection_lock = connection_lock
        self.shutdown_event = shutdown_event
        self.planner = TrajectoryPlanner(config)
        self.command_sender = MavlinkCommandSender(mav_conn, connection_lock)
        self.config = config

        # Unified sensor provider for both camera and IMU
        if config.CAMERA_SOURCE.lower() == "sim":
            self.sensor_provider = SimSensorProvider(config)
            self.target_selector = SimTargetSelector(config)
        else:
            self.sensor_provider = HardwareSensorProvider(config)
            self.target_selector = RealTargetSelector()
        # Target selection based on simulation or real
        self.target_selector = (
            SimTargetSelector(config) if config.CAMERA_SOURCE == "sim" else RealTargetSelector()
        )

        self.tracker = TargetTracker(config)

    def run(self):
        logging.info("Flight Control Thread started.")
        tracker_started = False

        self.sensor_provider.start()

        while not self.shutdown_event.is_set():
            camera_frame = self.sensor_provider.get_latest_camera_frame()
            if camera_frame is None or camera_frame.frame is None:
                continue

            frame = camera_frame.frame

            # Optional window display
            if self.config.SHOW_CAMERA_WINDOW:
                cv2.imshow("Simulated Camera", frame)
                cv2.waitKey(1)

            # Target acquisition
            bbox = self.target_selector.get_target_bbox(frame)
            if bbox is None:
                continue

            if not tracker_started:
                self.tracker.start_tracking(frame, bbox)
                tracker_started = True
            else:
                bbox = self.tracker.track(frame)
                if self.config.CAMERA_SOURCE == "sim" and bbox:
                    x, y, w, h = map(int, bbox)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 1)
                    cv2.imshow("Simulated Camera", frame)
                    cv2.waitKey(1)

            if tracker_started:
                imu = self.sensor_provider.get_latest_imu_data()
                mag = self.sensor_provider.get_latest_mag_data()
                cmd_quat, throttle = self.planner.compute_command(bbox, camera_frame, imu, mag)
                # Send the command:
                logging.info(f"Sent attitude command: {cmd_quat}, throttle: {throttle:.2f}")
                self.command_sender.send_attitude_target(cmd_quat, throttle)

            self.shutdown_event.wait(0.01)

        self.sensor_provider.stop()
        cv2.destroyAllWindows()
        logging.info("Flight Control Thread exiting.")
