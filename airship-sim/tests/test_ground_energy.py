"""P0 引擎补件测试:地形/接触、能量模型、载人级参数集。"""
import numpy as np

from airship_sim.config import default_config
from airship_sim.energy import EnergyModel
from airship_sim.ground import AnalyticTerrain, FlatTerrain
from airship_sim.presets_crewed import crewed_config, retune_gains
from airship_sim.simulation import Simulation
from airship_sim.dynamics import IDX_POS, IDX_VEL


# ---------- 地形 ----------
def test_analytic_terrain_components():
    t = AnalyticTerrain({"base_m": 100.0,
                         "slope": {"azimuth_deg": 180.0, "grade": 0.1},
                         "ridges": [{"center_n": 0, "center_e": 500,
                                     "axis_deg": 0, "height_m": 50,
                                     "halfwidth_m": 100}]})
    # 下坡朝南(180°):往南(-N... 注意 NED n 减小是南)高度应下降
    assert t.height_m(-100, 0) < t.height_m(0, 0) < t.height_m(100, 0)
    # 山脊顶(e=500)高于山脊外
    assert t.height_m(0, 500) > t.height_m(0, 0) + 40
    # 水面:低于水位处钳位且 is_water
    tw = AnalyticTerrain({"base_m": 0.0,
                          "slope": {"azimuth_deg": 0.0, "grade": 0.05},
                          "water_level_m": 10.0})
    assert tw.height_m(0, 0) == 10.0 and tw.is_water(0, 0)
    assert not tw.is_water(-400, 0)         # 上坡(南)处高于水位


def test_ground_landing_settles_without_explosion():
    """略重飞艇自由下沉触地:应稳定停在地面(无弹飞/无 NaN),且记录撞击速度。"""
    cfg = default_config(heaviness_kg=0.02)   # 20g 重,缓慢下沉
    sim = Simulation(cfg, terrain=FlatTerrain(0.0))
    sim.reset(p_ned_m=(0.0, 0.0, -1.0))
    sim.controller.manual_mode = True          # 无控制,纯下沉
    for _ in range(30000):                     # 30 s
        sim.step(record=False)
    assert np.all(np.isfinite(sim.x))
    assert sim.contact["contact"], "应停在地面"
    assert -sim.x[IDX_POS][2] < 1.0, "应贴地"
    assert float(np.linalg.norm(sim.x[IDX_VEL])) < 0.05, "应静止"
    assert sim.last_impact is not None and sim.last_impact["v_down_m_s"] < 1.0


def test_ground_none_path_unchanged():
    """不挂地形时轨迹与原版逐位一致(回归保护)。"""
    cfg = default_config(0.005)
    a = Simulation(cfg); a.reset(p_ned_m=(0, 0, -1.5))
    b = Simulation(cfg, terrain=None); b.reset(p_ned_m=(0, 0, -1.5))
    for _ in range(2000):
        a.step(record=False); b.step(record=False)
    assert np.array_equal(a.x, b.x)


# ---------- 能量 ----------
def test_energy_power_and_drain():
    cfg = crewed_config()
    e = EnergyModel(cfg.energy)
    # 悬停诱导功率:150N 单桨 @海平面 v_i=sqrt(150/(2·1.225·3.14))≈4.41 m/s
    # → P = 150×4.41/0.6 ≈ 1104 W
    p = e.prop_power_W(150.0, 0.0, 1.225)
    assert 900 < p < 1400
    # 高原同推力更耗电(v_i ∝ 1/√ρ)
    assert e.prop_power_W(150.0, 0.0, 0.75) > p
    # 前飞附加 T·u/η
    assert e.prop_power_W(150.0, 10.0, 1.225) > p + 150 * 10 / 0.7
    # 积分掉电
    e.step(np.array([100.0, 100.0, 20.0]), 10.0, 0.0, 1.225, False, 0.0, 3600.0)
    assert e.battery_Wh < cfg.energy.capacity_Wh - 1000


def test_energy_depletion_stops_motors():
    cfg = crewed_config()
    cfg.energy.capacity_Wh = 1.0               # 几乎没电
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0, 0, -30.0))
    sp = sim.controller.setpoints
    sp.altitude_m = 30.0; sp.speed_m_s = 5.0; sp.pos_hold = False
    for _ in range(8000):
        sim.step(record=False)
    assert sim.energy.depleted
    # 电机指令被置零 → 实际推力衰减到接近零
    assert float(np.max(np.abs(sim.x[13:16]))) < 5.0


# ---------- 载人级 ----------
def test_crewed_trim_and_hover():
    cfg = crewed_config(heaviness_kg=5.0)
    v = cfg.hull.volume_m3
    m_total = (cfg.mass.m_envelope_kg + cfg.mass.rho_helium_kg_m3 * v
               + cfg.mass.m_gondola_kg)
    assert abs(m_total - (1.225 * v + 5.0)) < 1e-6      # 配平正确
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0, 0, -30.0))
    sp = sim.controller.setpoints
    sp.altitude_m = 30.0; sp.pos_hold = True; sp.target_n_m = 0; sp.target_e_m = 0
    for _ in range(40000):                               # 40 s
        sim.step(record=False)
    alt = -sim.x[IDX_POS][2]
    assert np.all(np.isfinite(sim.x))
    assert abs(alt - 30.0) < 3.0, f"悬停高度失稳: {alt:.1f}m"
    drift = float(np.hypot(sim.x[IDX_POS][0], sim.x[IDX_POS][1]))
    assert drift < 30.0, f"无风定点漂移过大: {drift:.1f}m"


def test_crewed_holds_wind_when_facing_it():
    """迎风起始 + 6m/s 均匀风:定点漂移应有界(死区量级)。"""
    from airship_sim.math3d import quat_from_euler
    cfg = crewed_config(heaviness_kg=5.0)
    cfg.actuator.thrust_max_N = 300.0            # 强风桨
    retune_gains(cfg, 5.0)
    cfg.atmosphere.wind_ned_m_s = (-6.0, 0.0, 0.0)   # 风从北吹向南
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0, 0, -40.0), q=tuple(quat_from_euler(0, 0, 0.0)))  # 朝北迎风
    sp = sim.controller.setpoints
    sp.altitude_m = 40.0; sp.pos_hold = True; sp.target_n_m = 0; sp.target_e_m = 0
    peak = 0.0
    for i in range(50000):                        # 50 s(前段为吹离-回收瞬态)
        sim.step(record=False)
        if i > 40000:                             # 只考核收敛后的最后 10 s
            peak = max(peak, float(np.hypot(sim.x[IDX_POS][0], sim.x[IDX_POS][1])))
    assert np.all(np.isfinite(sim.x))
    assert peak < 60.0, f"6m/s 顶风定点稳态漂移过大: {peak:.1f}m"


def test_retune_scales_with_airframe():
    """换更大囊体/更强电机后,整定应给出更大的力/力矩权限。"""
    small = crewed_config()
    big = crewed_config()
    big.hull.length_m = 35.0; big.hull.diameter_m = 11.4
    big.mass.m_envelope_kg = 300.0
    big.actuator.thrust_max_N = 300.0
    v = big.hull.volume_m3
    big.mass.m_gondola_kg = 1.225 * v + 5.0 - 300.0 - 0.18 * v
    retune_gains(big, 5.0)
    assert big.control.alt.out_max > small.control.alt.out_max
    assert big.control.yaw.out_max > small.control.yaw.out_max
    assert big.control.alt.kp > small.control.alt.kp   # 更大等效质量 → 更大 kp
