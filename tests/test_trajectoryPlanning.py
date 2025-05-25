import math
import pytest
from types import SimpleNamespace

from core.vector3 import Vector3
from core.quaternion import Quaternion
from trajectory_planning.trajectory_planning import TrajectoryPlanner, pixel_to_vector_sphere, ros_quat_to_frd, \
    quat_to_frd



# Dummy sensor data structures
class DummyIMU:
    def __init__(self, timestamp, orientation_q, angular_velocity):
        self.timestamp = timestamp
        self.orientation_q = orientation_q  # (x,y,z,w)
        self.angular_velocity = angular_velocity  # (x_rate, y_rate, z_rate)

class DummyMag:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z

# A minimal config for TrajectoryPlanner
class Cfg:
    INITIAL_TILT_DEG = 10.0
    TILT_RAMP_RATE_DEG_S = 5.0
    MAX_TILT_DEG = 30.0
    GAIN_Z = 2.0
    GAIN_XY = 3.0
    ACCEL_GAIN = 0.0      # simplify accel term
    BORDER_FRAC = 0.2
    CAMERA_H_FOV = 90.0
    CAMERA_V_FOV = 60.0
    GRAVITY = 0.0         # zero out gravity for simpler thrust
    GRAV_KOEF = 1.0
    LOS_KOEF = 1.0
    MAG_OFFSET_YAW_RAD = 0.0

@pytest.fixture
def planner():
    return TrajectoryPlanner(Cfg)

# pixel_to_vector_sphere tests

def test_pixel_to_vector_center():
    v = pixel_to_vector_sphere(
        cx=50, cy=50,
        image_width=100, image_height=100,
        h_fov_deg=90, v_fov_deg=90
    )
    # center should map to forward X-axis in FRD: (1,0,0)
    assert v.to_tuple() == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)


def test_pixel_to_vector_edges():
    # top center pixel → look up
    v = pixel_to_vector_sphere(50, 0, 100, 100, 90, 90)
    half = math.radians(90/2)
    expected = Vector3(math.cos(half), 0.0, -math.sin(half))
    assert v.to_tuple() == pytest.approx(expected.normalized().to_tuple(), abs=1e-6)
    # right center pixel → look right
    v2 = pixel_to_vector_sphere(100, 50, 100, 100, 90, 90)
    exp2 = Vector3(math.cos(half), math.sin(half), 0.0)
    assert v2.to_tuple() == pytest.approx(exp2.normalized().to_tuple(), abs=1e-6)

# ROS quaternion conversion

def test_ros_quat_to_frd_and_quat_to_frd():
    # ROS identity quaternion
    qf = ros_quat_to_frd((0.0, 0.0, 0.0, 1.0))
    qt = quat_to_frd(Quaternion(1.0, 0.0, 0.0, 0.0))
    # both flips should equal the same 180° about X quaternion
    flip = Quaternion.from_axis_angle(Vector3(1,0,0), math.pi)
    assert qf.to_tuple() == pytest.approx(flip.to_tuple(), abs=1e-6)
    assert qt.to_tuple() == pytest.approx(flip.to_tuple(), abs=1e-6)

# TrajectoryPlanner internal helpers

def test__ramped_max_tilt(planner):
    # before start_time
    assert planner._ramped_max_tilt(now=0.0) == Cfg.INITIAL_TILT_DEG
    # after start_time
    planner.start_time = 0.0
    t = 2.0
    expect = min(Cfg.MAX_TILT_DEG, Cfg.INITIAL_TILT_DEG + Cfg.TILT_RAMP_RATE_DEG_S * t)
    assert planner._ramped_max_tilt(now=t) == pytest.approx(expect)


def test__angle_from_imu_identity(planner):
    imu = DummyIMU(timestamp=1.0, orientation_q=(0,0,0,1), angular_velocity=(0,0,0))
    # identity orientation → tilt = 0
    assert planner._angle_from_imu(imu) == pytest.approx(0.0)


def test__angle_from_imu_rotated(planner):
    q = Quaternion.from_axis_angle(Vector3(1,0,0), math.pi/2)
    imu = DummyIMU(1.0, (q.x, q.y, q.z, q.w), (0,0,0))
    # rotation → tilt = 90
    assert planner._angle_from_imu(imu) == pytest.approx(90.0)


def test__limit_tilt_branch(planner):
    thrust = Vector3(1,2,3)
    imu = DummyIMU(0.0, (0,0,0,1), (0,0,0))
    out = planner._limit_tilt(thrust, max_tilt=10.0, imu=imu)
    assert out.to_tuple() == pytest.approx(thrust.normalized().to_tuple())

    q = Quaternion.from_axis_angle(Vector3(1,0,0), math.pi/2)
    imu2 = DummyIMU(0.0, (q.x, q.y, q.z, q.w), (0,0,0))
    result = planner._limit_tilt(Vector3(1,0,2), max_tilt=45, imu=imu2)
    zt = math.copysign(2 * Cfg.GAIN_Z, 2)
    expected = Vector3(1, 0, zt).normalized()
    assert result.to_tuple() == pytest.approx(expected.to_tuple(), abs=1e-6)

# Boundary compensation tests

def test__apply_boundary_compensation_no_edges(planner):
    # no edge violation: use centered bbox
    thrust = Vector3(2,0,0)
    imu = DummyIMU(1.0, (0,0,0,1), (0,0,0))
    W, H = 100, 100
    # bbox centered at frame center
    bbox = (W/2 - 5, H/2 - 5, 10, 10)
    out1 = planner._apply_boundary_compensation(thrust, bbox, (W,H), imu)
    assert out1.to_tuple() == pytest.approx(thrust.normalized().to_tuple(), abs=1e-6)


def test__apply_boundary_compensation_x_edge(planner):
    thrust = Vector3(1, 0, 0)
    imu = DummyIMU(2.0, (0,0,0,1), (0,0,0))
    W, H = 100, 100
    # only x-edge: center Y
    bbox = (80, H/2, 0, 0)
    out = planner._apply_boundary_compensation(thrust, bbox, (W,H), imu)
    off_x = (80 - W/2)/(W/2)
    ex = off_x - Cfg.BORDER_FRAC
    comp_y = -Cfg.GAIN_XY * ex
    new = Vector3(1, comp_y, 0).normalized()
    assert out.to_tuple() == pytest.approx(new.to_tuple(), abs=1e-6)


def test__apply_boundary_compensation_y_edge(planner):
    thrust = Vector3(0, 0, 1)
    imu = DummyIMU(3.0, (0,0,0,1), (0,0,0))
    W, H = 100, 100
    # only y-edge: center X
    bbox = (W/2, 0, 0, 0)
    out = planner._apply_boundary_compensation(thrust, bbox, (W,H), imu)
    off_y = (H/2 - 0)/(H/2)
    ey = off_y - Cfg.BORDER_FRAC
    comp_z = -Cfg.GAIN_Z * ey
    new = Vector3(0, 0, 1 + comp_z).normalized()
    assert out.to_tuple() == pytest.approx(new.to_tuple(), abs=1e-6)

# Integration test

def test_compute_command_integration(planner):
    now = 5.0
    cam = SimpleNamespace(timestamp=now, frame=SimpleNamespace(shape=(480, 640)))
    imu = DummyIMU(now, (0,0,0,1), (0,0,0))
    mag = DummyMag(0.0, 1.0, 0.0)
    q_tuple, throttle = planner.compute_command((0,0,0,0), cam, imu, mag)
    assert throttle == pytest.approx(0.6)
    w,x,y,z = q_tuple
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    assert norm == pytest.approx(1.0, abs=1e-6)

def test_ramped_max_tilt_saturates(planner):
    # If you wait long enough, tilt should cap at MAX_TILT_DEG
    planner.start_time = 0.0
    # choose now so that INITIAL + rate*elapsed >> MAX
    now = (Cfg.MAX_TILT_DEG - Cfg.INITIAL_TILT_DEG) / Cfg.TILT_RAMP_RATE_DEG_S + 100.0
    assert planner._ramped_max_tilt(now) == pytest.approx(Cfg.MAX_TILT_DEG)

def test_compute_command_with_tilt_exceed(planner):
    # force tilt>max so _limit_tilt gets called inside compute_command
    now = 0.0
    cam = SimpleNamespace(timestamp=now, frame=SimpleNamespace(shape=(480, 640)))
    # IMU oriented so tilt = 90°
    q90 = Quaternion.from_axis_angle(Vector3(1,0,0), math.pi/2)
    imu = DummyIMU(now, (q90.x, q90.y, q90.z, q90.w), (0,0,0))
    mag = DummyMag(1.0, 0.0, 0.0)  # so psi=0
    planner.start_time = now  # so max_tilt == INITIAL
    cmd, throttle = planner.compute_command((0,0,0,0), cam, imu, mag)
    assert throttle == pytest.approx(0.6)
    # result quaternion must still be unit-length
    w, x, y, z = cmd
    assert pytest.approx(1.0, rel=1e-6) == math.sqrt(w*w + x*x + y*y + z*z)

def test_ros_quat_to_frd_nonidentity():
    # roll 90° about Y in ROS: (x,y,z,w) = (0, sin45, 0, cos45)
    sin45 = math.sin(math.pi/4)
    cos45 = math.cos(math.pi/4)
    qf = ros_quat_to_frd((0.0, sin45, 0.0, cos45))
    # that ROS rotation (+90° pitch) then flip about X gives a known composite;
    # just check it's still a valid unit quaternion
    mag = math.sqrt(qf.w*qf.w + qf.x*qf.x + qf.y*qf.y + qf.z*qf.z)
    assert pytest.approx(1.0, abs=1e-6) == mag

