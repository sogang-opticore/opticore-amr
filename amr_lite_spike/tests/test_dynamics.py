import math

import pytest

from amr_lite.envs.dynamics import RobotState, integrate
from amr_lite.envs.geometry import wrap_angle


def test_straight_line():
    state = integrate(RobotState(0, 0, 0), 1.0, 0.0, 1.0)
    assert (state.x, state.y, state.theta) == pytest.approx((1.0, 0.0, 0.0))


def test_arc_and_in_place_rotation():
    arc = integrate(RobotState(0, 0, 0), 1.0, 1.0, math.pi / 2)
    assert (arc.x, arc.y) == pytest.approx((1.0, 1.0))
    spin = integrate(RobotState(1, 2, 0), 0.0, 1.0, 0.5)
    assert (spin.x, spin.y, spin.theta) == pytest.approx((1.0, 2.0, 0.5))


def test_angle_normalization():
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert -math.pi <= wrap_angle(123.0) <= math.pi

