import cv2
import threading
import time
import logging
import os
from ultralytics import YOLO


class TargetTracker:
    """
    Implements target tracking by initializing a CSRT tracker on the initially selected target.
    Simultaneously, it runs YOLO on the target region in a background thread (only once).
    When YOLO returns a refined bounding box, the CSRT tracker is reinitialized with it.
    In simulation mode, the updated target can be outlined with a red rectangle.
    """

    def __init__(self, config):
        self.config = config
        self.tracker = None  # OpenCV CSRT tracker instance
        self.tracker_initialized = False  # Flag to check if tracker is initialized
        self.lock = threading.Lock()  # Synchronizes bbox and frame updates
        self.current_bbox = None  # Current bounding box from tracker
        self.latest_frame = None  # Latest frame received from main loop
        self.yolo_thread = None  # Background thread for YOLO detection
        self.yolo_done = False  # Flag indicating YOLO detection has finished

        # Initialize YOLO model using a relative path.
        model_path = os.path.join(os.path.dirname(__file__), "models", "best.pt")
        self.yolo = YOLO(model_path)
        # Log the loaded class names
        class_names = self.yolo.model.names if hasattr(self.yolo.model, "names") else {}
        logging.info("YOLO model loaded with classes: %s", class_names)

    def start_tracking(self, initial_frame, initial_bbox):
        """
        Immediately starts the CSRT tracker on the target and spawns a background thread to run YOLO once.

        Args:
            initial_frame: The frame in which the target is selected.
            initial_bbox: Tuple (x, y, w, h) representing the initial target area (e.g., 160x160 pixels).
        """
        # Initialize CSRT tracker
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker_initialized = self.tracker.init(initial_frame, initial_bbox)
        with self.lock:
            self.current_bbox = initial_bbox
            self.latest_frame = initial_frame.copy()

        # Start YOLO detection in the background (run once)
        self.yolo_thread = threading.Thread(target=self._run_yolo_once, daemon=True)
        self.yolo_thread.start()
        logging.info("TargetTracker: Tracking started with initial bbox %s", initial_bbox)

    def _run_yolo_once(self):
        """
        Runs YOLO detection on the ROI (region defined by the current bbox) once.
        When the detection is done, the CSRT tracker is reinitialized with the new bounding box.
        """
        # Small delay to ensure latest_frame is available.
        # time.sleep(0.1)
        with self.lock:
            bbox = self.current_bbox
            frame = self.latest_frame.copy() if self.latest_frame is not None else None
        if bbox is None or frame is None:
            logging.warning("YOLO detection: No frame or bbox available.")
            return

        x, y, w, h = bbox
        roi = frame[y:y + h, x:x + w]
        if roi.size == 0:
            logging.warning("YOLO detection: ROI is empty.")
            return

        # Run YOLO inference on the ROI.
        results = self.yolo(roi)
        new_bbox = bbox  # Default to original bbox if no detection.
        if results and len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes.xyxy.cpu().numpy() if hasattr(results[0].boxes, "xyxy") else None
            confs = results[0].boxes.conf.cpu().numpy() if hasattr(results[0].boxes, "conf") else None
            if boxes is not None and len(boxes) > 0:
                best_idx = int(confs.argmax()) if confs is not None else 0
                det_box = boxes[best_idx]  # [x1, y1, x2, y2] in ROI coordinates.

                # Map ROI coordinates to original frame coordinates.
                scale_x = w / float(roi.shape[1])
                scale_y = h / float(roi.shape[0])
                new_x = x + int(det_box[0] * scale_x)
                new_y = y + int(det_box[1] * scale_y)
                new_w = int((det_box[2] - det_box[0]) * scale_x)
                new_h = int((det_box[3] - det_box[1]) * scale_y)
                new_bbox = (new_x, new_y, new_w, new_h)

                # Save detected target image to file
                target_image = frame[new_y:new_y + new_h, new_x:new_x + new_w]
                if target_image.size > 0:
                    save_path = os.path.join(os.path.dirname(__file__), f"detected_target{time.time_ns()}.jpg")
                    cv2.imwrite(save_path, target_image)
                    logging.info("Saved detected target image to %s", save_path)
                else:
                    logging.warning("Detected target image has zero size; not saved.")

                logging.info("YOLO detection: Updated bbox to %s", new_bbox)
            else:
                logging.info("YOLO detection: No valid boxes found; using original bbox.")
        else:
            logging.info("YOLO detection: No results; using original bbox.")

        # Update tracker with new bbox.
        with self.lock:
            self.current_bbox = new_bbox
        self.tracker.init(frame, new_bbox)
        self.yolo_done = True

    def track(self, frame, fallback_bbox=None):
        """
        Updates the CSRT tracker on the current frame and returns the updated bounding box.
        Also stores the latest frame for YOLO (if needed).

        Args:
            frame: The current video frame.
            fallback_bbox: Fallback bounding box if tracking fails.

        Returns:
            Updated bounding box as (x, y, w, h) or fallback_bbox if tracking fails.
        """
        # if not self.tracker_initialized:
        #     return fallback_bbox

        success, bbox = self.tracker.update(frame)
        with self.lock:
            self.latest_frame = frame.copy()
        if success:
            with self.lock:
                self.current_bbox = bbox
            return bbox
        else:
            logging.warning("TargetTracker: CSRT tracker lost target.")
            return fallback_bbox

    def stop(self):
        """
        Waits for the YOLO thread to finish if it is running.
        """
        if self.yolo_thread is not None:
            self.yolo_thread.join()
        logging.info("TargetTracker: Tracking stopped.")
