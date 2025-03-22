import logging
from camera.target_selector.base_target_selector import BaseTargetSelector


class RealTargetSelector(BaseTargetSelector):
    def get_target_bbox(self, frame):
        logging.info("RealTargetSelector: Not implemented yet.")
        return None