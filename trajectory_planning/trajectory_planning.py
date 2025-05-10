import collections
import logging
import math
from core.quaternion import Quaternion
from core.vector3 import Vector3
from sensors.data_structures import IMUData, MagData


def pixel_to_vector_sphere(
    cx: float,
    cy: float,
    image_width: int,
    image_height: int,
    h_fov_deg: float,
    v_fov_deg: float,
) -> Vector3:
    """Convert *pixel* centre to a unit LOS vector in camera frame (FRD)."""
    nx = (cx - image_width / 2) / (image_width / 2)
    ny = (image_height / 2 - cy) / (image_height / 2)
    half_h = math.radians(h_fov_deg / 2)
    half_v = math.radians(v_fov_deg / 2)
    yaw = nx * half_h
    pitch = ny * half_v
    cos_p = math.cos(pitch)
    x_cam = cos_p * math.sin(yaw)  # right
    y_cam = cos_p * math.cos(yaw)  # forward
    z_cam = math.sin(pitch)  # up
    # Camera (forward‑right‑down)
    return Vector3(y_cam, x_cam, -z_cam).normalized()


def ros_quat_to_frd(q_ros: tuple[float, float, float, float]) -> Quaternion:
    """Convert ROS (FLU) quaternion to Forward‑Right‑Down body frame."""
    q_flip = Quaternion(math.pi, 1.0, 0.0, 0.0)  # 180° about body‑X
    return Quaternion.from_ros(q_ros) * q_flip

def quat_to_frd(q: Quaternion) -> Quaternion:
    """Convert ROS (FLU) quaternion to Forward‑Right‑Down body frame."""
    q_flip = Quaternion(math.pi, 1.0, 0.0, 0.0)  # 180° about body‑X
    return q * q_flip


# ----------------------------------------------------------------------
# Trajectory planner (refactored)
# ----------------------------------------------------------------------

class TrajectoryPlanner:
    def __init__(self, config):
        self.cfg = config
        self.prev_t: float | None = None
        self.start_time: float | None = None
        if self.cfg.ACC_SMOOTH_METHOD == "window":
            self._acc_buf: collections.deque[Vector3] = collections.deque(maxlen=self.cfg.ACC_WINDOW_SIZE)
        elif self.cfg.ACC_SMOOTH_METHOD == "ema":
            self._acc_ema = Vector3(0.0, 0.0, 0.0)

    # --------------------------------------------------------------
    # Internal helpers
    # --------------------------------------------------------------
    def _ramped_max_tilt(self, now: float) -> float:
        init = self.cfg.INITIAL_TILT_DEG
        rate = self.cfg.TILT_RAMP_RATE_DEG_S
        hard = self.cfg.MAX_TILT_DEG
        if self.start_time is None:
            return math.radians(init)
        elapsed = now - self.start_time
        return math.radians(min(hard, init + rate * elapsed))


    def _angle_from_imu(self, imu: IMUData) -> float:
        imu_orientation_frd = ros_quat_to_frd(imu.orientation_q)
        global_vector = imu_orientation_frd.rotate(Vector3(0, 0, 1))
        return math.radians(global_vector.vertical_angle_deg())


    def _limit_tilt(self, thrust_dir_w: Vector3, max_tilt: float, imu: IMUData) -> Vector3:
        tilt = self._angle_from_imu(imu)
        if tilt <= max_tilt:
            return thrust_dir_w.normalized()
        horiz = math.hypot(thrust_dir_w.x, thrust_dir_w.y)
        if horiz == 0.0:
            return Vector3(0.0, 0.0, 1.0 if thrust_dir_w.z >= 0 else -1.0)
        z_target = horiz / math.tan(max_tilt)
        z_target = math.copysign(z_target, thrust_dir_w.z if thrust_dir_w.z != 0 else 1.0)
        return Vector3(thrust_dir_w.x, thrust_dir_w.y, z_target).normalized()

    # --------------------------------------------------------------
    # Public API
    # --------------------------------------------------------------
    def compute_command(self, bbox, cam_frame, imu: IMUData, mag: MagData):
        """Return attitude *Quaternion* and *throttle* based on latest sensors."""
        now = cam_frame.timestamp
        self.start_time = self.start_time or now
        self.prev_t = now

        logging.info(f"imu = {imu}")

        # ------------------------------------------------------------------
        # LOS in world frame
        # ------------------------------------------------------------------
        x, y, bw, bh = bbox
        cx, cy = x + bw / 2, y + bh / 2
        los_cam = pixel_to_vector_sphere(
            cx,
            cy,
            cam_frame.frame.shape[1],
            cam_frame.frame.shape[0],
            self.cfg.CAMERA_H_FOV,
            self.cfg.CAMERA_V_FOV,
        )
        logging.info(f"los_cam = {los_cam}")
        q_bw = Quaternion.from_axis_angle(Vector3(0, 0, 1), math.pi)
        los_world = (q_bw.rotate(los_cam)).normalized()
        logging.info(f"los_world = {los_world}")

        # Desired thrust: LOS guidance + gravity compensation
        g_vec = Vector3(0.0, 0.0, -self.cfg.GRAVITY * self.cfg.GRAV_KOEF)
        thrust_dir_w = (los_world * self.cfg.LOS_KOEF - g_vec).normalized()

        logging.info(f"thrust_dir_w = {thrust_dir_w}")

        max_tilt = self._ramped_max_tilt(now)
        if self._angle_from_imu(imu) > max_tilt:
            thrust_dir_w = self._limit_tilt(thrust_dir_w, max_tilt, imu)
            logging.info(f"adjusted thrust_dir_w= {thrust_dir_w}")

        # Local attitude command (world Z → thrust_dir_w)
        cmd_local = Quaternion.between(Vector3(0, 0, 1), thrust_dir_w)
        throttle = 0.6  # TODO: derive from controller
        logging.info(f"cmd_local= {cmd_local}")

        # ------------------------------------------------------------------
        # Heading correction via magnetometer
        # ------------------------------------------------------------------
        mx, my, mz = mag.y, mag.x, -mag.z  # NED axes
        psi = math.atan2(my, mx) - 2.9520  # site declination offset
        logging.info(f"psi= {psi}")
        q_h = Quaternion.from_axis_angle(Vector3(0, 0, 1), -psi)
        logging.info(f"q_h= {q_h}")

        cmd_quat = (q_h * cmd_local)
        return cmd_quat.to_tuple(), throttle
