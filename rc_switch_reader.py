import threading
from pymavlink import mavutil

class RCSwitchReader(threading.Thread):
    """
    Monitors MAVLink RC_CHANNELS (or RC_CHANNELS_RAW) messages and caches
    the latest channel values. Non-blocking read, so it won’t stall loops.
    """
    def __init__(self, mav_conn, shutdown_event):
        super().__init__(name="RCSwitchReader", daemon=True)
        self.mav = mav_conn
        self.shutdown = shutdown_event
        self.latest = {}  # ch_index: μs

    def run(self):
        while not self.shutdown.is_set():
            msg = self.mav.recv_match(type=["RC_CHANNELS", "RC_CHANNELS_RAW"], blocking=False)
            if msg:
                # RC_CHANNELS gives chan1_raw … chan18_raw, RC_CHANNELS_RAW gives chan1_raw … chan8_raw
                for i in range(1, 19):
                    field = f"chan{i}_raw"
                    if hasattr(msg, field):
                        self.latest[i] = getattr(msg, field)
