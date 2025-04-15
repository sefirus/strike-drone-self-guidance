import logging
import cv2

from camera.target_selector.base_target_selector import BaseTargetSelector

class SimTargetSelector(BaseTargetSelector):
    def __init__(self, config):
        self.config = config
        self.selected_bbox = None
        self.selection_in_progress = False

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            frame = param
            h = frame.shape[0]
            box_size = h // 3  # bounding box size is 1/3 of the height.
            x0 = max(x - box_size // 2, 0)
            y0 = max(y - box_size // 2, 0)
            self.selected_bbox = (x0, y0, box_size, box_size)
            logging.info(f"SimTargetSelector: Target selected at ({x}, {y}) with bbox {self.selected_bbox}")

    def get_target_bbox(self, frame):
        # Set up the mouse callback if not done already.
        if not self.selection_in_progress:
            cv2.namedWindow("Simulated Camera")
            cv2.setMouseCallback("Simulated Camera", self._mouse_callback, param=frame)
            self.selection_in_progress = True
        return self.selected_bbox
