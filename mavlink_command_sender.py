import time


class MavlinkCommandSender:
    """
    Encapsulates MAVLink attitude+throttle command sending.
    """
    def __init__(self, mav_connection, connection_lock):
        self.mav_connection = mav_connection
        self.connection_lock = connection_lock

    def send_attitude_target(self, attitude_quaternion, throttle):
        """
        Send a SET_ATTITUDE_TARGET MAVLink message with the given quaternion and throttle.

        Args:
            attitude_quaternion: Tuple (w, x, y, z)
            throttle: Float in [0.0, 1.0]
        """
        # Unpack quaternion
        w, x, y, z = attitude_quaternion
        # Build and send MAVLink message
        with self.connection_lock:
            self.mav_connection.mav.set_attitude_target_send(
                time_boot_ms=int(time.monotonic() * 1000),
                target_system=self.mav_connection.target_system,
                target_component=self.mav_connection.target_component,
                type_mask=0,              # Use all fields
                q=(w, x, y, z),           # Desired attitude quaternion
                body_roll_rate=0.0,
                body_pitch_rate=0.0,
                body_yaw_rate=0.0,
                thrust=throttle
            )
