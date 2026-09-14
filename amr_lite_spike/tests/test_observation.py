import numpy as np

from amr_lite.envs import WarehouseEnv


def test_observation_contract_and_frame_order():
    env = WarehouseEnv(scenario_id="D0")
    observation, _ = env.reset(seed=12)
    assert observation.shape == (296,)
    assert observation.dtype == np.float32
    assert np.isfinite(observation).all()
    assert np.max(observation) <= 1.0 and np.min(observation) >= -1.0
    assert np.array_equal(observation[:72], observation[216:288])
    next_observation, *_ = env.step([0.0, 0.0])
    assert np.array_equal(next_observation[:216], observation[72:288])
