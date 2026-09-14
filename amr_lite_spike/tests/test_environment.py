from amr_lite.envs import WarehouseEnv
from amr_lite.planning import RuleBasedPlanner


def test_rule_reaches_empty_corridor_and_reward_components_exist():
    env = WarehouseEnv(scenario_id="S0")
    observation, _ = env.reset(seed=5)
    planner = RuleBasedPlanner()
    for _ in range(360):
        observation, reward, terminated, truncated, info = env.step(planner.predict(observation))
        assert set(info["reward_components"]) == {"progress", "goal", "collision", "near", "smooth", "path", "idle",
                                                   "heading", "turn", "time"}
        if terminated or truncated:
            break
    assert info["success"] and not info["collision"]


def test_static_obstacle_episode_is_collision_free():
    env = WarehouseEnv(scenario_id="S3")
    observation, _ = env.reset(seed=5)
    planner = RuleBasedPlanner()
    for _ in range(360):
        observation, _, terminated, truncated, info = env.step(planner.predict(observation))
        if terminated or truncated:
            break
    assert info["success"] and not info["collision"]
