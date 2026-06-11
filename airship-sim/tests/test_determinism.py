"""可复现性:同配置同种子 → 逐位一致;配置 JSON 往返。"""
import dataclasses

import numpy as np

from airship_sim.config import SimConfig, default_config
from airship_sim.simulation import Simulation


def run_states(cfg: SimConfig, duration_s: float = 2.0) -> np.ndarray:
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0, 0, -1.5))
    sim.controller.setpoints.altitude_m = 2.0
    sim.controller.setpoints.pos_hold = True
    states = []
    n = round(duration_s / cfg.dt_physics_s)
    for _ in range(n):
        sim.step(record=False)
        states.append(sim.x.copy())
    return np.array(states)


def test_bitwise_reproducible():
    cfg = default_config()
    a = run_states(cfg)
    b = run_states(cfg)
    assert np.array_equal(a, b)   # 逐位一致(非近似)


def test_config_json_roundtrip():
    cfg = default_config()
    cfg2 = SimConfig.from_json(cfg.to_json())
    assert dataclasses.asdict(cfg) == dataclasses.asdict(cfg2)


def test_roundtripped_config_same_trajectory():
    cfg = default_config()
    cfg2 = SimConfig.from_json(cfg.to_json())
    assert np.array_equal(run_states(cfg, 1.0), run_states(cfg2, 1.0))


def test_different_seed_different_noise():
    cfg = default_config()
    cfg2 = default_config()
    cfg2.seed = cfg.seed + 1
    assert not np.array_equal(run_states(cfg, 1.0), run_states(cfg2, 1.0))
