import threading
import logging
import cv2

from camera.camera_getter.sim_camera_getter import SimCameraGetter
from camera.camera_getter.usb_camera_getter import RealCameraGetter
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

        # Initialize Camera Module
        if self.config.CAMERA_SOURCE == "sim":
            self.camera_getter = SimCameraGetter(self.config)
            self.target_selector = SimTargetSelector(self.config)
        else:
            self.camera_getter = RealCameraGetter(self.config)
            self.target_selector = RealTargetSelector()

        self.tracker = TargetTracker()

    def run(self):
        logging.info("Flight Control Thread started.")

        # Start the camera feed
        self.camera_getter.start()

        while not self.shutdown_event.is_set():
            frame = self.camera_getter.get_frame()
            if frame is None:
                continue  # Skip iteration if no frame is available

            if self.camera_getter.config.SHOW_CAMERA_WINDOW:
                cv2.imshow("Simulated Camera", frame)
                cv2.waitKey(1)

            # Process the frame
            bbox = self.target_selector.get_target_bbox(frame)

            # Pass to tracker (placeholder)
            tracked_target = self.tracker.track(frame, bbox)

            # Show the camera window if enabled
            if self.config.SHOW_CAMERA_WINDOW:
                display_frame = frame.copy()
                if bbox:
                    x, y, w, h = bbox
                    cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.imshow("Simulated Camera", display_frame)
                cv2.waitKey(1)

            # Ensure loop runs at a reasonable rate
            self.shutdown_event.wait(0.05)

        # Cleanup
        self.camera_getter.stop()
        cv2.destroyAllWindows()
        logging.info("Flight Control Thread exiting.")
