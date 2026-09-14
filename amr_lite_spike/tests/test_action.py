import numpy as np
import pytest

from amr_lite.envs.dynamics import limit_velocity
from amr_lite.learning.action import normalize_command, scale_action


def test_action_roundtrip_and_scaling():
    assert scale_action(np.array([-1, -1]), 1.5, 2.0) == (0.0, -2.0)
    assert scale_action(np.array([1, 1]), 1.5, 2.0) == (1.5, 2.0)
    assert normalize_command(0.75, 1.0, 1.5, 2.0) == pytest.approx([0.0, 0.5])


def test_acceleration_and_deceleration_limits():
    assert limit_velocity(0, 0, 1, 2, 0.1, 2, 3, 1, 2, 4) == pytest.approx((0.1, 0.4))
    assert limit_velocity(1, 0, 0, -2, 0.1, 2, 3, 1, 2, 4) == pytest.approx((0.8, -0.4))

