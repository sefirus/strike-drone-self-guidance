import os
import cv2
import logging
import threading

from camera.yolo_detector import YOLODetector


class TargetTracker:
    """
    Implements target tracking using a CSRT tracker.
    Runs YOLO detection asynchronously when (and only when) the CSRT tracker loses the target,
    and returns the last known bbox until YOLO finishes.
    """

    def __init__(self, config):
        self.config = config
        self.tracker = None
        self.current_bbox = None
        self.lock = threading.Lock()

        # Async-YOLO state
        self.yolo_thread = None
        self.yolo_running = False

        model_path = os.path.join(os.path.dirname(__file__), "models", "best.pt")
        self.yolo_detector = YOLODetector(model_path)

    def start_tracking(self, initial_frame, initial_bbox):
        logging.info("TargetTracker: Initializing tracker.")
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(initial_frame, initial_bbox)

        with self.lock:
            self.current_bbox = initial_bbox

    def _run_yolo_async(self, frame, last_bbox):
        """Worker thread: run YOLO once, then reinit CSRT if found."""
        try:
            new_bbox = self.yolo_detector.detect(frame, last_bbox)
            if new_bbox:
                logging.info("TargetTracker: YOLO reacquired target, reinitializing CSRT.")
                with self.lock:
                    self.current_bbox = new_bbox
                # reinit CSRT on the new box
                tracker = cv2.TrackerCSRT_create()
                tracker.init(frame, new_bbox)
                with self.lock:
                    self.tracker = tracker
            else:
                logging.info("TargetTracker: YOLO failed to find target.")
        finally:
            # always clear the running flag
            self.yolo_running = False

    def track(self, frame, fallback_bbox=None):
        """
        Call every frame. If CSRT loses the target, spawn a YOLO thread exactly once,
        and meanwhile return the last known bbox.
        """
        try:
            success, bbox = self.tracker.update(frame)
            if success and bbox is not None:
                with self.lock:
                    self.current_bbox = bbox
                return bbox

            # lost the target
            logging.warning("TargetTracker: CSRT lost target.")
            # if YOLO isn't already running, start it
            if not self.yolo_running:
                with self.lock:
                    last = self.current_bbox
                self.yolo_running = True
                # pass frame copy and last bbox
                thread = threading.Thread(
                    target=self._run_yolo_async,
                    args=(frame.copy(), last),
                    daemon=True
                )
                thread.start()
                self.yolo_thread = thread

            # while YOLO is running (or if it later fails), return last known
            return fallback_bbox or self.current_bbox

        except Exception as e:
            logging.exception("TargetTracker: Exception during track()")
            return fallback_bbox or self.current_bbox

    def stop(self):
        if self.yolo_thread is not None:
            self.yolo_thread.join()
        logging.info("TargetTracker: Tracking stopped.")
