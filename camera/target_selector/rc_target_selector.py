# camera/target_selector/real_target_selector.py
import math
from camera.target_selector.base_target_selector import BaseTargetSelector

class RealTargetSelector(BaseTargetSelector):
    """
    Tracking OFF / ON / ACTIVATE is driven by a single RC channel.
    No mouse click support (real drone).
    """

    def __init__(self, config, rc_pwm_reader):
        """
        rc_pwm_reader → callable returning latest PWM μs (int).
        """
        super().__init__()
        self.cfg       = config
        self.get_pwm   = rc_pwm_reader
        self.bbox      = None
        self.size_px   = 160

    def _update_state(self, frame_shape):
        pwm = self.get_pwm()
        if pwm < 1500 or pwm >= self.cfg.ACT_SW_TH:
            self.bbox = None           # tracking disabled
            return False
        if self.bbox is None:
            h, w = frame_shape[:2]
            self.bbox = (w//2 - self.size_px//2,
                          h//2 - self.size_px//2,
                          self.size_px, self.size_px)
        return True                    # tracking enabled

    # ------------------------------------------------------------------
    def get_target_bbox(self, frame):
        """
        Returns centre 160×160 bbox when RC switch in “track” range,
        otherwise returns None.
        """
        return self.bbox if self._update_state(frame.shape) else None
