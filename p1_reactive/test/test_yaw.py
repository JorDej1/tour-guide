import math
from types import SimpleNamespace

from p1_reactive.controller import yaw_from_quaternion
import pytest


def quat(deg):
    half = math.radians(deg) / 2
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def test_facing_forward_is_zero():
    assert yaw_from_quaternion(quat(0)) == pytest.approx(0.0)


def test_quarter_turn_left():
    assert yaw_from_quaternion(quat(90)) == pytest.approx(math.pi / 2)


def test_wraps_past_180():
    assert yaw_from_quaternion(quat(270)) == pytest.approx(-math.pi / 2)
