import threading
import logging
import cv2

from sensors.sim_sensor_provider import SimSensorProvider
from camera.target_selector.sim_target_selector import SimTargetSelector
from camera.target_selector.rc_target_selector import RealTargetSelector
from camera.target_tracker import TargetTracker

class FlightControlThread(threading.Thread):
    def __init__(self, mav_conn, connection_lock, shutdown_event, config):
        super().__init__(name="FlightControlThread")
        self.mav_conn = mav_conn
        self.connection_lock = connection_lock
        self.shutdown_event = shutdown_event
        self.config = config

        # Unified sensor provider for both camera and IMU
        self.sensor_provider = SimSensorProvider(config)

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
                cv2.imshow("Camera Feed", frame)
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
                    cv2.imshow("Camera Feed", frame)
                    cv2.waitKey(1)

            self.shutdown_event.wait(0.05)

        self.sensor_provider.stop()
        cv2.destroyAllWindows()
        logging.info("Flight Control Thread exiting.")
