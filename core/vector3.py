import math


class Vector3:
    """Light‑weight 3‑component vector with basic arithmetic helpers."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float):
        self.x, self.y, self.z = x, y, z

    # ---------------------------------------------------------------------
    # Magic methods
    # ---------------------------------------------------------------------
    def __iter__(self):
        return iter((self.x, self.y, self.z))

    def __repr__(self):
        return f"Vector3({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__  # allow scalar * vec

    # ------------------------------------------------------------------
    # Basic helpers
    # ------------------------------------------------------------------
    def magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self) -> "Vector3":
        mag = self.magnitude()
        return Vector3(0.0, 0.0, 0.0) if mag == 0 else Vector3(self.x / mag, self.y / mag, self.z / mag)

    def dot(self, other: "Vector3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vector3") -> "Vector3":
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    # ------------------------------------------------------------------
    # Convenience angles
    # ------------------------------------------------------------------
    def horizontal_angle_deg(self) -> float:
        """Azimuth angle in degrees from +X toward +Y (right‑hand Z up)."""
        return math.degrees(math.atan2(self.y, self.x))

    def vertical_angle_deg(self) -> float:
        """Elevation angle in degrees from XY‑plane toward +Z."""
        return math.degrees(math.atan2(self.z, math.hypot(self.x, self.y)))

    # ------------------------------------------------------------------
    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)