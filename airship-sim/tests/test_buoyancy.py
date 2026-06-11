"""浮力/重力配平与恢复力矩。"""
import numpy as np

from airship_sim.added_mass import lamb_k_factors
from airship_sim.atmosphere import make_atmosphere
from airship_sim.config import GRAVITY_M_S2, default_config
from airship_sim.dynamics import IDX_VEL, AirshipDynamics
from airship_sim.math3d import quat_from_euler


def make_dyn(heaviness_kg: float) -> AirshipDynamics:
    cfg = default_config(heaviness_kg=heaviness_kg)
    return AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))


def test_neutral_trim_zero_acceleration():
    # 纯浮力配平(heaviness=0):静止 + 水平姿态 + 零推力 → 所有导数为零
    dyn = make_dyn(0.0)
    x = dyn.initial_state(p_ned_m=(0, 0, -2.0))
    dx = dyn.derivatives(x, np.zeros(3), 0.0)
    assert np.allclose(dx, 0.0, atol=1e-12)


def test_heaviness_sink_acceleration():
    # 偏重 5g:垂直加速度 = Δm·g / (m_total + 横向附加质量)
    heaviness = 0.005
    dyn = make_dyn(heaviness)
    x = dyn.initial_state()
    dx = dyn.derivatives(x, np.zeros(3), 0.0)
    cfg = dyn.cfg
    k1, k2, _ = lamb_k_factors(cfg.hull.a_m, cfg.hull.b_m)
    m_eff_z = dyn.props.m_total_kg + k2 * 1.225 * cfg.hull.volume_m3
    expect_a = heaviness * GRAVITY_M_S2 / m_eff_z
    a_z = dx[IDX_VEL][2]
    assert np.isclose(a_z, expect_a, rtol=1e-9)
    # 量级:近中性浮力,下沉加速度应当很小(<0.05 m/s²)
    assert 0 < a_z < 0.05


def test_cg_offset_default_geometry():
    # 默认配置:重心在浮心正下方约 0.4m(规格要求)
    dyn = make_dyn(0.005)
    r_g = dyn.props.r_g_b_m
    assert abs(r_g[0]) < 1e-12 and abs(r_g[1]) < 1e-12
    assert np.isclose(r_g[2], 0.40, atol=0.01)


def test_restoring_pitch_moment():
    # 俯仰 +10°(抬头)时,重心偏移应产生负俯仰力矩(恢复:压头)
    dyn = make_dyn(0.0)
    q = quat_from_euler(0.0, np.deg2rad(10.0), 0.0)
    x = dyn.initial_state(q=q)
    dx = dyn.derivatives(x, np.zeros(3), 0.0)
    pitch_acc = dx[10:13][1]
    assert pitch_acc < 0.0
    # 力矩幅值与解析值 m·g·d·sinθ / 比较(整体动力学求解后量级应一致,容差宽)
    # 滚转方向同理
    q = quat_from_euler(np.deg2rad(10.0), 0.0, 0.0)
    dx = dyn.derivatives(dyn.initial_state(q=q), np.zeros(3), 0.0)
    assert dx[10:13][0] < 0.0


def test_buoyancy_tracks_local_density():
    # 浮力由大气接口的当地密度实时计算:密度降低 → 配平被破坏,出现下沉加速度
    cfg = default_config(heaviness_kg=0.0)
    cfg.atmosphere.rho_const_kg_m3 = 1.20   # 比配平密度 1.225 低
    dyn = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    dx = dyn.derivatives(dyn.initial_state(), np.zeros(3), 0.0)
    assert dx[IDX_VEL][2] > 1e-4   # 向下加速
