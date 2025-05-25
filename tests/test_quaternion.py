import math
import pytest
from core.quaternion import Quaternion
from core.vector3 import Vector3


def assert_quat_close(q1, q2, tol=1e-6):
    assert pytest.approx(q2.w, abs=tol) == q1.w
    assert pytest.approx(q2.x, abs=tol) == q1.x
    assert pytest.approx(q2.y, abs=tol) == q1.y
    assert pytest.approx(q2.z, abs=tol) == q1.z


def test_iter_and_repr_and_to_tuple():
    q = Quaternion(1.23456, -0.12345, 0.000123, 5.6789)
    w, x, y, z = q
    assert (w, x, y, z) == pytest.approx((1.23456, -0.12345, 0.000123, 5.6789))
    rep = repr(q)
    assert rep == "Quaternion(1.235, -0.123, 0.000, 5.679)"
    assert q.to_tuple() == pytest.approx((1.23456, -0.12345, 0.000123, 5.6789))


def test_mul_quaternion_with_identity():
    q = Quaternion(0.7071, 0.0, 0.7071, 0.0)
    identity = Quaternion(1, 0, 0, 0)
    # identity multiplies should return original
    assert (q * identity).to_tuple() == q.to_tuple()
    assert (identity * q).to_tuple() == q.to_tuple()


def test_mul_quaternion_quaternion():
    # Test associativity against manual known product
    q1 = Quaternion(1, 2, 3, 4)
    q2 = Quaternion(0.5, -1, 2, -0.5)
    prod = q1 * q2
    # Manually computed expected
    w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
    w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
    expected = Quaternion(
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )
    assert prod.to_tuple() == pytest.approx(expected.to_tuple())


def test_mul_vector_rotation_alias():
    # rotation of (1,0,0) by 90deg around Z yields (0,1,0)
    axis = Vector3(0, 0, 1)
    q = Quaternion.from_axis_angle(axis, math.pi/2)
    v = Vector3(1, 0, 0)
    rotated = q * v
    assert isinstance(rotated, Vector3)
    assert rotated.to_tuple() == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)


def test_mul_unsupported_type_raises():
    q = Quaternion(1, 0, 0, 0)
    with pytest.raises(TypeError):
        _ = q * 123


def test_inverse_and_conjugate_property():
    q = Quaternion.from_axis_angle(Vector3(0,1,0), math.pi/3)
    inv = q.inverse()
    # q * inv should be identity
    prod = q * inv
    assert prod.to_tuple() == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-6)
    # inverse of inverse is original
    assert inv.inverse().to_tuple() == pytest.approx(q.to_tuple(), abs=1e-6)


def test_rotate_short_circuit_w_zero():
    v = Vector3(1,2,3)
    # w == 0 branch
    q1 = Quaternion(0, 1, 2, 3)
    assert q1.rotate(v).to_tuple() == v.to_tuple()
    # x=y=z==0 branch
    q2 = Quaternion(2, 0, 0, 0)
    assert q2.rotate(v).to_tuple() == v.to_tuple()


def test_between_identical_and_opposite():
    v = Vector3(1,0,0)
    # identical vectors => identity quaternion
    q_same = Quaternion.between(v, v)
    assert q_same.to_tuple() == pytest.approx((1.0, 0.0, 0.0, 0.0))
    # opposite vectors => zero quaternion result
    q_opp = Quaternion.between(v, Vector3(-1,0,0))
    assert q_opp.to_tuple() == pytest.approx((0.0, 0.0, 0.0, 0.0))


def test_from_axis_angle_symmetry():
    axis = Vector3(1, 1, 1)
    angle = math.pi/4
    q_pos = Quaternion.from_axis_angle(axis, angle)
    q_neg = Quaternion.from_axis_angle(axis, -angle)
    # inverse of positive angle equals negative angle quaternion
    assert q_pos.inverse().to_tuple() == pytest.approx(q_neg.to_tuple(), abs=1e-6)


def test_from_ros_and_to_tuple():
    ros = (0.1, 0.2, 0.3, 0.4)
    q = Quaternion.from_ros(ros)
    assert q.to_tuple() == pytest.approx((0.4, 0.1, 0.2, 0.3))


def test_from_axis_angle_unit_length():
    axis = Vector3(2, 0, 0)
    angle = 1.234
    q = Quaternion.from_axis_angle(axis, angle)
    # quaternion from axis-angle should be unit-length
    mag = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
    assert pytest.approx(1.0, abs=1e-6) == mag

def test_from_axis_angle_normalizes_axis():
    # a non-unit axis should be normalized internally
    axis_long = Vector3(0, 2, 0)
    axis_unit = Vector3(0, 1, 0)
    angle = math.pi / 3
    q_long = Quaternion.from_axis_angle(axis_long, angle)
    q_unit = Quaternion.from_axis_angle(axis_unit, angle)
    # They should be identical quaternions
    assert q_long.to_tuple() == pytest.approx(q_unit.to_tuple(), abs=1e-6)


def test_between_perpendicular_vectors_rotates_correctly():
    # between should give 90° rotation around Z for X→Y
    v_from = Vector3(1, 0, 0)
    v_to   = Vector3(0, 1, 0)
    q = Quaternion.between(v_from, v_to)
    # rotating v_from by q yields v_to
    result = q * v_from
    assert result.to_tuple() == pytest.approx(v_to.to_tuple(), abs=1e-6)


def test_multiplication_associativity_on_vector():
    # (q1⊗q2)⊗v == q1⊗(q2⊗v)
    axis1 = Vector3(0, 0, 1)
    axis2 = Vector3(1, 0, 0)
    q1 = Quaternion.from_axis_angle(axis1, math.pi/4)
    q2 = Quaternion.from_axis_angle(axis2, math.pi/6)
    v = Vector3(1, 2, 3)
    left  = (q1 * q2) * v
    right = q1 * (q2 * v)
    assert left.to_tuple() == pytest.approx(right.to_tuple(), abs=1e-6)


def test_inverse_of_non_unit_quaternion():
    # even if q isn't unit-length, inverse should satisfy q⊗q⁻¹ = identity
    # pick an arbitrary quaternion
    q = Quaternion(2.0, -1.0, 0.5, 3.0)
    inv = q.inverse()
    prod = q * inv
    assert prod.to_tuple() == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-6)


def test_from_ros_rejects_bad_input():
    # from_ros expects a 4-tuple; anything else should be a TypeError
    with pytest.raises(ValueError):
        Quaternion.from_ros((1.0, 2.0, 3.0))  # too short

def test_rotate_consistency_with_mul_alias():
    # rotate(...) and __mul__(Vector3) should agree exactly
    axis = Vector3(0, 1, 0)
    angle = math.pi / 5
    q = Quaternion.from_axis_angle(axis, angle)
    v = Vector3(5, -3, 2)
    assert q.rotate(v).to_tuple() == pytest.approx((q * v).to_tuple(), abs=1e-8)
