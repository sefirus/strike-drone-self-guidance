import math
import pytest
from core.vector3 import Vector3


def test_iter_unpack():
    v = Vector3(1.0, 2.0, 3.0)
    x, y, z = v
    assert (x, y, z) == (1.0, 2.0, 3.0)


def test_to_tuple_and_repr():
    v = Vector3(1.23456, -7.891011, 0.0001)
    assert v.to_tuple() == (1.23456, -7.891011, 0.0001)
    # repr formats to three decimals
    rep = repr(v)
    assert rep == "Vector3(1.235, -7.891, 0.000)"


def test_add_sub():
    a = Vector3(1, 2, 3)
    b = Vector3(4, -5, 6)
    sum_ab = a + b
    diff_ab = a - b
    assert sum_ab.to_tuple() == (5, -3, 9)
    assert diff_ab.to_tuple() == (-3, 7, -3)


def test_mul_and_rmul():
    v = Vector3(1, -2, 3)
    scaled1 = v * 3
    scaled2 = 3 * v
    assert scaled1.to_tuple() == (3, -6, 9)
    assert scaled2.to_tuple() == (3, -6, 9)


def test_magnitude():
    v = Vector3(3, 4, 12)
    assert math.isclose(v.magnitude(), 13.0)


def test_normalized_nonzero():
    v = Vector3(0, 3, 4)
    nv = v.normalized()
    # magnitude of normalized should be 1
    assert math.isclose(nv.magnitude(), 1.0)
    # direction should match
    expected = Vector3(0, 3/5, 4/5)
    assert math.isclose(nv.x, expected.x)
    assert math.isclose(nv.y, expected.y)
    assert math.isclose(nv.z, expected.z)


def test_normalized_zero():
    v = Vector3(0, 0, 0)
    nz = v.normalized()
    assert nz.to_tuple() == (0.0, 0.0, 0.0)


def test_dot():
    a = Vector3(1, 2, 3)
    b = Vector3(-4, 5, -6)
    assert a.dot(b) == (1 * -4 + 2 * 5 + 3 * -6)
    # orthogonal vectors
    u = Vector3(1, 0, 0)
    v = Vector3(0, 1, 0)
    assert math.isclose(u.dot(v), 0.0)


def test_cross():
    i = Vector3(1, 0, 0)
    j = Vector3(0, 1, 0)
    k = Vector3(0, 0, 1)
    assert i.cross(j).to_tuple() == (0, 0, 1)
    assert j.cross(i).to_tuple() == (0, 0, -1)
    # parallel vectors produce zero
    assert i.cross(i).to_tuple() == (0, 0, 0)


def test_horizontal_angle_deg():
    # along +X axis
    assert math.isclose(Vector3(1, 0, 0).horizontal_angle_deg(), 0.0)
    # along +Y axis
    assert math.isclose(Vector3(0, 1, 0).horizontal_angle_deg(), 90.0)
    # along -X axis
    assert math.isclose(Vector3(-1, 0, 0).horizontal_angle_deg(), 180.0)
    # along -Y axis
    assert math.isclose(Vector3(0, -1, 0).horizontal_angle_deg(), -90.0)


def test_vertical_angle_deg():
    # flat XY plane
    assert math.isclose(Vector3(3, 4, 0).vertical_angle_deg(), 0.0)
    # straight up
    assert math.isclose(Vector3(0, 0, 1).vertical_angle_deg(), 90.0)
    # straight down
    assert math.isclose(Vector3(0, 0, -1).vertical_angle_deg(), -90.0)
    # 45 degrees elevation
    v = Vector3(1, 0, 1)
    assert math.isclose(v.vertical_angle_deg(), 45.0)
