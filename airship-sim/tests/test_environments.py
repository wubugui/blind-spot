"""环境预设:构建、配平、密度效应、配置 JSON 往返(含空间风场规格)。"""
import numpy as np
import pytest

from airship_sim.config import SimConfig
from airship_sim.environments import environment, glacier, lake, plateau, trim_for_altitude
from airship_sim.simulation import Simulation


def test_environments_build_and_run():
    for name in ("glacier", "plateau", "lake"):
        cfg = environment(name)
        sim = Simulation(cfg)
        sim.reset(p_ned_m=(0, 0, -8.0))
        assert sim.atmosphere.wind_field is not None, f"{name} 应挂上空间风场"
        for _ in range(500):
            sim.step(record=False)
        assert np.all(np.isfinite(sim.x)), f"{name} 状态发散"


def test_plateau_is_low_density_and_trimmed():
    cfg = plateau(ground_alt_m=4500.0, air_temp_C=5.0)
    sim = Simulation(cfg)
    st = sim.atmosphere.get_state(np.array([0, 0, -8.0]), 0.0)
    assert st.rho_kg_m3 < 0.85, f"4500m 密度应显著低于海平面,实测 {st.rho_kg_m3:.3f}"
    # 配平后近中性:净浮力(重量−浮力)约等于 heaviness,不再是海平面配重下的暴沉。
    # (直接查浮力平衡,隔离掉热泡上升气流对静止艇的气动升力)
    v = cfg.hull.volume_m3
    m_total = (cfg.mass.m_envelope_kg + cfg.mass.rho_helium_kg_m3 * v
               + cfg.mass.m_gondola_kg)
    net_kg = m_total - st.rho_kg_m3 * v
    assert abs(net_kg) < 0.02, f"配平后应近中性,净重 {net_kg:+.3f} kg"


def test_trim_raises_when_envelope_too_small():
    cfg = SimConfig()
    with pytest.raises(ValueError):
        trim_for_altitude(cfg, ground_alt_m=15000.0, dT_K=0.0)  # 太高,气囊配不平


def test_config_json_roundtrip_with_wind_field():
    cfg = glacier(ground_alt_m=4000.0, katabatic_peak_m_s=5.0, jet_height_m=6.0)
    txt = cfg.to_json()
    cfg2 = SimConfig.from_json(txt)
    assert cfg2.atmosphere.wind_field is not None
    assert cfg2.atmosphere.wind_field["kind"] == "glacier_katabatic"
    # 还原后风场逐点求值应与原配置一致
    s1 = Simulation(cfg); s2 = Simulation(cfg2)
    w1 = s1.atmosphere.get_state(np.array([0, 0, -6.0]), 0.0).wind_ned_m_s
    w2 = s2.atmosphere.get_state(np.array([0, 0, -6.0]), 0.0).wind_ned_m_s
    assert np.allclose(w1, w2)


def test_lake_breeze_present():
    cfg = lake(onshore_dir_deg=0.0, lake_breeze=True)
    sim = Simulation(cfg)
    # 锋面推进后陆侧应出现 onshore(+x)风分量
    w = sim.atmosphere.get_state(np.array([5.0, 0, -5.0]), 30.0).wind_ned_m_s
    assert w[0] > 0.0
