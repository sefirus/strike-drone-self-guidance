
import collections
import logging
import math
from sensors.data_structures import IMUData, MagData


def clamp(val, lo, hi):
    return max(lo, min(val, hi))


def normalize(v):
    x, y, z = v
    mag = math.sqrt(x * x + y * y + z * z)
    return (0.0, 0.0, 0.0) if mag == 0 else (x / mag, y / mag, z / mag)


def q_inv(q):
    w, x, y, z = q
    n2 = w * w + x * x + y * y + z * z
    return (w / n2, -x / n2, -y / n2, -z / n2)


def q_rot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    rw = -x * vx - y * vy - z * vz
    rx = w * vx + y * vz - z * vy
    ry = w * vy - x * vz + z * vx
    rz = w * vz + x * vy - y * vx
    iw, ix, iy, iz = q_inv(q)
    fx = rw * ix + rx * iw + ry * iz - rz * iy
    fy = rw * iy - rx * iz + ry * iw + rz * ix
    fz = rw * iz + rx * iy - ry * ix + rz * iw
    return (fx, fy, fz)


def q_between(v_from, v_to):
    f = normalize(v_from)
    t = normalize(v_to)
    dot_ = clamp(sum(a * b for a, b in zip(f, t)), -1.0, 1.0)
    angle = math.acos(dot_)
    logging.info(f"f, t, angle = {f, t, angle}")
    axis = normalize((
        f[1] * t[2] - f[2] * t[1],
        f[2] * t[0] - f[0] * t[2],
        f[0] * t[1] - f[1] * t[0]))
    logging.info(f"axis = {axis}")
    s = math.sin(angle / 2.0)
    return (math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)

def pixel_to_vector_sphere(cx, cy, image_width, image_height, h_fov_deg, v_fov_deg):
    # normalized pixel coords
    nx = (cx - image_width / 2) / (image_width / 2)
    ny = (image_height / 2 - cy) / (image_height / 2)

    # half‐angles
    half_h = math.radians(h_fov_deg / 2)
    half_v = math.radians(v_fov_deg / 2)

    # yaw about vertical, pitch about lateral
    yaw = nx * half_h
    pitch = ny * half_v

    cos_p = math.cos(pitch)
    # original camera‐frame vector
    x_cam = cos_p * math.sin(yaw)   # right
    y_cam = cos_p * math.cos(yaw)   # forward
    z_cam = math.sin(pitch)         # up

    # convert to forward‐right‐down (FRD)
    x = y_cam        # forward
    y = x_cam        # right
    z = -z_cam       # down

    return normalize((x, y, z))


def ros_quat_to_frd(q_ros):
    """
    Convert a ROS‐style quaternion (X→forward, Y→left, Z→up)
    into an FRD‐style quaternion (X→forward, Y→right, Z→down).
    This is equivalent to a 180° rotation about the body X‐axis.
    """
    # 180° (π) about X → θ/2 = π/2 → cos=0, sin=1
    q_flip = (0.0, 1.0, 0.0, 0.0)  # (w, x, y, z)
    return quat_mul(q_ros, q_flip)


# helper to multiply quaternions (w,x,y,z)
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )

class TrajectoryPlanner_copy:
    def __init__(self, config):
        self.cfg = config
        self.prev_t = None
        self.start_time = None
        if self.cfg.ACC_SMOOTH_METHOD == "window":
            self._acc_buf = collections.deque(maxlen=self.cfg.ACC_WINDOW_SIZE)
        elif self.cfg.ACC_SMOOTH_METHOD == "ema":
            self._acc_ema = (0.0, 0.0, 0.0)

    def _ramped_max_tilt(self, now: float) -> float:
        init = getattr(self.cfg, "INITIAL_TILT_DEG", 10)
        rate = getattr(self.cfg, "TILT_RAMP_RATE_DEG_S", 15)
        hard = getattr(self.cfg, "MAX_TILT_DEG", getattr(self.cfg, "max_tilt_angle_deg", 45))
        if self.start_time is None:
            return math.radians(init)
        elapsed = now - self.start_time
        max_deg = min(hard, init + rate * elapsed)
        return math.radians(max_deg)

    def get_angles(self, global_los_cam):
        x, y, z = global_los_cam
        # horizontal angle (azimuth) from +X toward +Y
        hor = math.degrees(math.atan2(y, x))
        # vertical angle (elevation) from XY-plane up toward +Z
        ver = math.degrees(math.atan2(z, math.hypot(x, y)))
        return hor, ver

    def _angle_from_z(self, vector, imu: IMUData):
        imu_orientation_frd = ros_quat_to_frd(imu.orientation_q)
        logging.info(f"imu_orientation_frd = {imu_orientation_frd}")
        global_vector =q_rot(imu_orientation_frd, vector)
        logging.info(f"global_vector = {global_vector}")
        hor_angle, ver_angle = self.get_angles(global_vector)
        logging.info(f"hor_angle, ver_angle = {hor_angle, ver_angle}")

        return math.radians(ver_angle)

    def _angle_from_imu(self, imu: IMUData):
        imu_orientation_frd = ros_quat_to_frd(imu.orientation_q)
        global_vector = q_rot(imu_orientation_frd, (0, 0, 1))
        logging.info(f"global_vector = {global_vector}")
        hor_angle, ver_angle = self.get_angles(global_vector)
        logging.info(f"hor_angle, ver_angle = {hor_angle, ver_angle}")

        return math.radians(ver_angle)

    def _limit_tilt(self, thrust_dir_w, max_tilt, imu):
        """
        If the tilt (angle away from +Z) is larger than `max_tilt`, increase the Z
        component just enough so that the vector lies on the max-tilt cone.
        X and Y stay untouched; the vector is re-normalised at the end.

        Parameters
        ----------
        thrust_dir_w : tuple[float, float, float]
            Desired (possibly un-normalised) thrust direction in the **world** frame.
        max_tilt : float
            Maximum allowed tilt in **radians**.

        Returns
        -------
        tuple[float, float, float]
            A unit-length vector whose tilt ≤ `max_tilt`.
        """
        # Current tilt
        tilt = self._angle_from_imu(imu)
        if tilt <= max_tilt:
            return normalize(thrust_dir_w)

        x, y, z = thrust_dir_w
        horiz = math.hypot(x, y)  # |projection onto XY|

        if horiz == 0.0:  # already vertical -> nothing to do
            return (0.0, 0.0, 1.0) if z >= 0 else (0.0, 0.0, -1.0)

        # Required Z so that tan(tilt_new) == tan(max_tilt)
        # tilt_new  : tan = horiz / z_new
        # therefore z_new = horiz / tan(max_tilt)
        z_target = horiz / math.tan(max_tilt)

        # Keep the original sign (up vs down)
        z_target = math.copysign(z_target, z if z != 0 else 1.0)

        limited = (x, y, z_target)
        return normalize(limited)


    def compute_command(self, bbox, cam_frame, imu: IMUData, mag:MagData):
        now = cam_frame.timestamp
        if self.start_time is None:
            self.start_time = now
        self.prev_t = now

        ori = (0, 0, 1, 0)
        q_bw = (ori[3], ori[0], ori[1], ori[2])

        x, y, bw, bh = bbox
        cx, cy = x + bw / 2, y + bh / 2
        los_cam = pixel_to_vector_sphere(
            cx, cy,
            cam_frame.frame.shape[1],
            cam_frame.frame.shape[0],
            self.cfg.CAMERA_H_FOV,
            self.cfg.CAMERA_V_FOV,
        )
        los_world = normalize(q_rot(q_bw, los_cam))

        g_vec = (0.0, 0.0, -self.cfg.GRAVITY * self.cfg.GRAV_KOEF)
        thrust_dir_w = normalize((
            self.cfg.LOS_KOEF * los_world[0] - g_vec[0],
            self.cfg.LOS_KOEF * los_world[1] - g_vec[1],
            self.cfg.LOS_KOEF * los_world[2] - g_vec[2],
        ))

        logging.info(f"thrust_dir_w = {thrust_dir_w}")

        thrust_dir_b = q_rot(q_inv(q_bw), thrust_dir_w)
        max_tilt = self._ramped_max_tilt(now)
        tilt = self._angle_from_imu(imu)
        logging.info(f"thrust_dir_b = {thrust_dir_b}")

        if tilt > max_tilt:
            thrust_dir_b = self._limit_tilt(thrust_dir_b, max_tilt, imu)
            thrust_dir_w = q_rot(q_bw, thrust_dir_b)

            tilt_after = self._angle_from_imu(imu)
            logging.info(f"Tilt AFTER limit = {math.degrees(tilt_after):.2f}°")

        cmd_local = q_between((0, 0, 1), thrust_dir_w)
        throttle = 0.6
        logging.info(f"cmd_local= {cmd_local}")


        mx, my, mz = mag.y, mag.x, -mag.z
        psi = math.atan2(my, mx) - 2.9520
        # logging.info(f"Magnetic heading ψ = {psi}, vector: x = {mx}, y = {my}, z = {mz}")
        logging.info(f"psi= {psi}")

        # --- STEP 3: build “undo‐yaw” quaternion Q_H ---
        # rotate by –ψ about NED‐Z (down) axis = (0,0,1)
        half = -psi / 2.0
        q_h = (
            math.cos(half),
            0.0,
            0.0,
            math.sin(half)
        )
        logging.info(f"q_h= {q_h}")

        # --- STEP 4: prepend heading correction ---
        # Q_cmd = Q_H ⊗ Q_L
        cmd_quat = quat_mul(q_h, cmd_local)

        return cmd_quat, throttle