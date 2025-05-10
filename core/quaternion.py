import math
import logging

from core.helpers import clamp
from core.vector3 import Vector3


class Quaternion:
    """Minimal utility quaternion (w, x, y, z)."""

    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w: float, x: float, y: float, z: float):
        self.w, self.x, self.y, self.z = w, x, y, z

    # ------------------------------------------------------------------
    # Magic methods
    # ------------------------------------------------------------------
    def __iter__(self):
        return iter((self.w, self.x, self.y, self.z))

    def __repr__(self):
        return f"Quaternion({self.w:.3f}, {self.x:.3f}, {self.y:.3f}, {self.z:.3f})"

    def __mul__(self, other: "Quaternion | Vector3") -> "Quaternion | Vector3":
        """Quaternion ⊗ Quaternion, or rotate Vector3 by this quaternion."""
        if isinstance(other, Quaternion):
            w1, x1, y1, z1 = self.w, self.x, self.y, self.z
            w2, x2, y2, z2 = other.w, other.x, other.y, other.z
            return Quaternion(
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            )
        if isinstance(other, Vector3):  # rotate vector
            return self.rotate(other)
        raise TypeError("Unsupported multiplication")

    # ------------------------------------------------------------------
    # Core quaternion ops
    # ------------------------------------------------------------------
    def inverse(self) -> "Quaternion":
        n2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
        return Quaternion(self.w / n2, -self.x / n2, -self.y / n2, -self.z / n2)

    def rotate(self, v: Vector3) -> Vector3:
        """Rotate Vector3 *v* from *this* frame into the parent frame."""
        # Hamilton product q * (0,v) * q⁻¹ (expanded for speed)
        w, x, y, z = self.w, self.x, self.y, self.z
        if w == 0 or x == 0 and y == 0 and z == 0:
            return v

        vx, vy, vz = v.x, v.y, v.z
        rw = -x * vx - y * vy - z * vz
        rx = w * vx + y * vz - z * vy
        ry = w * vy - x * vz + z * vx
        rz = w * vz + x * vy - y * vx
        iw, ix, iy, iz = self.inverse()
        fx = rw * ix + rx * iw + ry * iz - rz * iy
        fy = rw * iy - rx * iz + ry * iw + rz * ix
        fz = rw * iz + rx * iy - ry * ix + rz * iw
        return Vector3(fx, fy, fz)

    # ------------------------------------------------------------------
    # Factories
    # ------------------------------------------------------------------
    @staticmethod
    def between(v_from: Vector3, v_to: Vector3) -> "Quaternion":
        f = v_from.normalized()
        t = v_to.normalized()
        dot_ = clamp(f.dot(t), -1.0, 1.0)
        angle = math.acos(dot_)
        logging.info(f"f, t, angle = {f, t, angle}")
        axis = f.cross(t).normalized()
        logging.info(f"axis = {axis}")
        s = math.sin(angle / 2.0)
        return Quaternion(math.cos(angle / 2.0), axis.x * s, axis.y * s, axis.z * s)

    @staticmethod
    def from_axis_angle(axis: Vector3, angle_rad: float) -> "Quaternion":
        axis_n = axis.normalized()
        s = math.sin(angle_rad / 2)
        return Quaternion(math.cos(angle_rad / 2), axis_n.x * s, axis_n.y * s, axis_n.z * s)

    @staticmethod
    def from_ros(q_ros: tuple[float, float, float, float]) -> "Quaternion":
        """Convert (x, y, z, w) to (w, x, y, z)."""
        x, y, z, w = q_ros
        return Quaternion(w, x, y, z)

    # ------------------------------------------------------------------
    def to_tuple(self) -> tuple[float, float, float, float]:
        return (self.w, self.x, self.y, self.z)
