import os
import cv2
import logging
import threading

from camera.yolo_detector import YOLODetector


class TargetTracker:
    """
    Implements target tracking using a CSRT tracker.
    Runs YOLO detection asynchronously once on the initial region and reinitializes the tracker if detection is found.
    """

    def __init__(self, config):
        self.config = config
        self.tracker = None
        self.current_bbox = None
        self.latest_frame = None

        self.lock = threading.Lock()
        self.yolo_thread = None
        self.yolo_done = False

        model_path = os.path.join(os.path.dirname(__file__), "models", "best.pt")
        self.yolo_detector = YOLODetector(model_path)

    def start_tracking(self, initial_frame, initial_bbox):
        logging.info("TargetTracker: Initializing tracker.")
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(initial_frame, initial_bbox)

        with self.lock:
            self.current_bbox = initial_bbox
            self.latest_frame = initial_frame.copy()

        # Start YOLO detection asynchronously
        self.yolo_thread = threading.Thread(target=self._run_yolo_async, daemon=True)
        self.yolo_thread.start()

    def _run_yolo_async(self):
        with self.lock:
            bbox = self.current_bbox
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        new_bbox = self.yolo_detector.detect(frame, bbox)

        if new_bbox:
            with self.lock:
                self.current_bbox = new_bbox
                self.latest_frame = frame.copy()
                self.tracker.init(frame, new_bbox)


            logging.info("TargetTracker: Tracker reinitialized with new YOLO bbox.")
        else:
            logging.info("TargetTracker: YOLO did not find a better bbox.")

        self.yolo_done = True

    def track(self, frame, fallback_bbox=None):
        success, bbox = self.tracker.update(frame)
        with self.lock:
            self.latest_frame = frame.copy()

        if success:
            with self.lock:
                self.current_bbox = bbox
            return bbox

        logging.warning("TargetTracker: CSRT tracker lost target.")
        return fallback_bbox

    def stop(self):
        if self.yolo_thread is not None:
            self.yolo_thread.join()
        logging.info("TargetTracker: Tracking stopped.")
