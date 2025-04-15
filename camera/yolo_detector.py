import logging

from ultralytics import YOLO


class YOLODetector:
    """
    A separate class to run YOLO object detection synchronously.
    It processes the given frame and region and returns a new bounding box if detection is successful.
    """

    def __init__(self, model_path):
        self.yolo = YOLO(model_path)
        class_names = self.yolo.model.names if hasattr(self.yolo.model, "names") else {}
        logging.info("YOLODetector: YOLO model loaded with classes: %s", class_names)

    def detect(self, frame, bbox):
        """
        Runs YOLO detection on the region defined by bbox in the frame.
        Returns a new bounding box (x, y, w, h) if detection is found, otherwise returns None.
        """
        if bbox is None or frame is None:
            logging.warning("YOLODetector: No frame or bbox available.")
            return None

        x, y, w, h = bbox
        roi = frame[y:y + h, x:x + w]
        if roi.size == 0:
            logging.warning("YOLODetector: ROI is empty.")
            return None

        logging.info("YOLODetector: Running YOLO detection.")
        results = self.yolo(roi)
        if results and len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            all_boxes = results[0].boxes
            # extract numpy arrays
            cls_ids = all_boxes.cls.cpu().numpy() if hasattr(all_boxes, "cls") else None
            boxes = all_boxes.xyxy.cpu().numpy() if hasattr(all_boxes, "xyxy") else None
            confs = all_boxes.conf.cpu().numpy() if hasattr(all_boxes, "conf") else None

            # keep only detections of class 7
            if cls_ids is not None:
                mask = cls_ids == 7
                boxes = boxes[mask]
                confs = confs[mask]

            if boxes is not None and len(boxes) > 0:
                # pick the highest‑confidence detection among class 7
                best_idx = int(confs.argmax())
                det_box = boxes[best_idx]  # [x1, y1, x2, y2] in ROI coords.

                # Map ROI coords back to full‑frame
                scale_x = w / float(roi.shape[1])
                scale_y = h / float(roi.shape[0])
                new_x = x + int(det_box[0] * scale_x)
                new_y = y + int(det_box[1] * scale_y)
                new_w = int((det_box[2] - det_box[0]) * scale_x)
                new_h = int((det_box[3] - det_box[1]) * scale_y)
                new_bbox = (new_x, new_y, new_w, new_h)

                logging.info("YOLODetector: Detection (class 7) updated bbox to %s", new_bbox)
                return new_bbox

        logging.info("YOLODetector: No valid detection; returning None.")
        return None
