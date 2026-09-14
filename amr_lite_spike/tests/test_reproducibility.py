import numpy as np

from amr_lite.config import load_config
from amr_lite.envs import WarehouseEnv


def test_deterministic_reset_and_step():
    first, second = WarehouseEnv(scenario_id="D0"), WarehouseEnv(scenario_id="D0")
    a, _ = first.reset(seed=99)
    b, _ = second.reset(seed=99)
    assert np.array_equal(a, b)
    for _ in range(3):
        a, ra, ta, tra, ia = first.step([0.0, 0.1])
        b, rb, tb, trb, ib = second.step([0.0, 0.1])
        assert np.array_equal(a, b)
        assert (ra, ta, tra, ia["robot_pose"]) == (rb, tb, trb, ib["robot_pose"])


def test_randomized_scenario_same_seed_replays_and_different_seed_changes():
    config = load_config("env", {"scenario_randomization": {"enabled": True}})
    first, second, third = (
        WarehouseEnv(config, scenario_id="D1"),
        WarehouseEnv(config, scenario_id="D1"),
        WarehouseEnv(config, scenario_id="D1"),
    )
    a, info_a = first.reset(seed=20000)
    b, info_b = second.reset(seed=20000)
    c, info_c = third.reset(seed=20001)
    assert np.array_equal(a, b)
    assert info_a["scenario_instance_id"] == info_b["scenario_instance_id"]
    assert info_a["scenario_instance_id"] != info_c["scenario_instance_id"]
    assert not np.array_equal(a, c)
    assert first.global_path and third.global_path


def test_selection_and_test_seed_namespaces_do_not_overlap():
    selection = load_config("evaluation_selection")
    test = load_config("evaluation_test")
    selection_seeds = set(range(selection["seed"],
                                selection["seed"] + selection["episodes_per_scenario"]))
    test_seeds = set(range(test["seed"], test["seed"] + test["episodes_per_scenario"]))
    assert not selection_seeds & test_seeds
