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
    """Convert ROS (x,y,z,w) FLU quaternion into FRD (Forward-Right-Down) body frame."""
    # first convert from ROS order into your internal (w,x,y,z)
    q_body = Quaternion.from_ros(q_ros)
    # then flip 180° about the body-X axis using a proper unit quaternion
    q_flip = Quaternion.from_axis_angle(Vector3(1.0, 0.0, 0.0), math.pi)
    return q_body * q_flip

def quat_to_frd(q: Quaternion) -> Quaternion:
    """Re-express an existing (w,x,y,z) ROS-style quaternion into FRD body frame."""
    # same 180° about X flip
    q_flip = Quaternion.from_axis_angle(Vector3(1.0, 0.0, 0.0), math.pi)
    return q * q_flip

# ----------------------------------------------------------------------
# Trajectory planner (refactored)
# ----------------------------------------------------------------------

class TrajectoryPlanner:
    def __init__(self, config):
        self.cfg = config
        self.start_time: float | None = None
        self._prev_stamp = None
        self._prev_rate = Vector3(0.0, 0.0, 0.0)


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
        imu_orientation_frd = Quaternion(imu.orientation_q[3], imu.orientation_q[0], imu.orientation_q[1], imu.orientation_q[2])
        logging.info(f"imu_orientation_frd = {imu_orientation_frd}")
        global_vector = imu_orientation_frd.rotate(Vector3(0, 0, 1))
        logging.info(f"global_vector = {global_vector}")
        return 90 - global_vector.vertical_angle_deg()


    def _limit_tilt(self, thrust_dir_w: Vector3, max_tilt: float, imu: IMUData) -> Vector3:
        tilt = self._angle_from_imu(imu)
        if tilt <= max_tilt:
            return thrust_dir_w.normalized()
        z_target = math.copysign(thrust_dir_w.z * self.cfg.GAIN_Z, thrust_dir_w.z if thrust_dir_w.z != 0 else 1.0)
        return Vector3(thrust_dir_w.x, thrust_dir_w.y, z_target).normalized()

    def _apply_boundary_compensation(
            self,
            thrust_dir_frd: Vector3,
            bbox: tuple[int, int, int, int],
            frame_size: tuple[int, int],
            imu: IMUData,
    ) -> Vector3:
        # guarantee prev-stamp / prev-rate exist
        if self._prev_stamp is None:
            self._prev_stamp = imu.timestamp
        if self._prev_rate is None:
            self._prev_rate = Vector3(*imu.angular_velocity)

        x, y, w, h = bbox
        W, H = frame_size
        cx, cy = x + w * 0.5, y + h * 0.5

        # normalised offsets (centre → 0, bounds → ±1)
        off_x = (cx - W * 0.5) / (W * 0.5)  # +right  / −left
        off_y = (H * 0.5 - cy) / (H * 0.5)  # +up     / −down

        def edge_err(off: float) -> float:
            if abs(off) < self.cfg.BORDER_FRAC:
                return 0.0
            return (abs(off) - self.cfg.BORDER_FRAC) * (1.0 if off > 0 else -1.0)

        ex, ey = edge_err(off_x), edge_err(off_y)

        # body-rate acceleration (for optional feed-forward)
        curr_rate = Vector3(*imu.angular_velocity)
        dt = max(imu.timestamp - self._prev_stamp, 1e-3)
        dv = curr_rate - self._prev_rate
        ang_acc = Vector3(dv.x/dt, dv.y/dt, dv.z/dt)
        self._prev_rate, self._prev_stamp = curr_rate, imu.timestamp

        # initialise comps to zero ─ we’ll set only what each edge needs
        comp_x = comp_y = comp_z = 0.0

        # ── LEFT / RIGHT edge violated ────────────────────────────────
        if ex != 0.0:
            # roll toward the target ( +y rolls right-wing down )
            comp_x = -self.cfg.GAIN_XY * ex + self.cfg.ACCEL_GAIN * ang_acc.x

        # ── TOP / BOTTOM edge violated ────────────────────────────────
        if ey != 0.0:
            # pure vertical change: ascend if target is high, descend if low
            comp_z = -self.cfg.GAIN_Z * ey + self.cfg.ACCEL_GAIN * ang_acc.z
            #     ey > 0  (target above centre)  →  negative comp_z  (ascend)
            #     ey < 0  (target below centre)  →  positive comp_z  (descend)

        logging.info(f"edge ex={ex} ey={ey}  comps = ({comp_x}, {comp_y}, {comp_z})")

        # combine & renormalise
        new_vec = Vector3(
            thrust_dir_frd.x + comp_x,
            thrust_dir_frd.y + comp_y,
            thrust_dir_frd.z + comp_z,
        )
        return new_vec.normalized()

    # --------------------------------------------------------------
    # Public API
    # --------------------------------------------------------------
    def compute_command(self, bbox, cam_frame, imu: IMUData, mag: MagData):
        """Return attitude *Quaternion* and *throttle* based on latest sensors."""
        now = cam_frame.timestamp
        self.start_time = self.start_time or now

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
        tilt = self._angle_from_imu(imu)
        logging.info(f"tilt = {tilt}")
        if tilt > max_tilt:
            thrust_dir_w = self._limit_tilt(thrust_dir_w, max_tilt, imu)
            logging.info(f"adjusted thrust_dir_w= {thrust_dir_w}")

        thrust_dir_w = self._apply_boundary_compensation(thrust_dir_w, bbox, (640, 480), imu)

        # Local attitude command (world Z → thrust_dir_w)
        cmd_local = Quaternion.between(Vector3(0, 0, 1), thrust_dir_w)
        throttle = 0.6  # TODO: derive from controller
        # logging.info(f"cmd_local= {cmd_local}")

        # ------------------------------------------------------------------
        # Heading correction via magnetometer
        # ------------------------------------------------------------------
        mx, my, mz = mag.y, mag.x, -mag.z  # NED axes
        psi = math.atan2(my, mx) - self.cfg.MAG_OFFSET_YAW_RAD # site declination offset
        # logging.info(f"psi= {psi}")
        q_h = Quaternion.from_axis_angle(Vector3(0, 0, 1), -psi)
        # logging.info(f"q_h= {q_h}")

        cmd_quat = (q_h * cmd_local)
        return cmd_quat.to_tuple(), throttle
