"""环境风场:Prandtl 坡风解析性质、对数律、叠加、与大气/动力学的集成。"""
import math

import numpy as np

from airship_sim.config import AtmosphereConfig, default_config
from airship_sim.atmosphere import IsaAtmosphere
from airship_sim.dynamics import AirshipDynamics, IDX_POS, IDX_VEL
from airship_sim.wind_field import (CompositeWind, LogLawWind, PrandtlSlopeWind,
                                    UniformWind, glacier_katabatic)


def test_prandtl_peak_height_and_magnitude():
    """峰值应出现在 z = jet_height 处且等于 peak_speed;地表为 0。"""
    jet_h, peak = 6.0, 4.0
    w = PrandtlSlopeWind(downhill_azimuth_deg=180.0, peak_speed_m_s=peak,
                         jet_height_m=jet_h, katabatic=True)
    assert np.linalg.norm(w.wind_ned(np.zeros(3), 0.0, 0.0)) < 1e-9  # 贴地无风
    peak_spd = np.linalg.norm(w.wind_ned(np.zeros(3), jet_h, 0.0))
    assert abs(peak_spd - peak) < 1e-6, f"峰速应为 {peak}, 实得 {peak_spd}"
    # 峰值处确实是极大:两侧略小
    below = np.linalg.norm(w.wind_ned(np.zeros(3), jet_h * 0.5, 0.0))
    above = np.linalg.norm(w.wind_ned(np.zeros(3), jet_h * 1.6, 0.0))
    assert below < peak_spd and above < peak_spd


def test_prandtl_direction_downhill_and_return_flow():
    """katabatic 顺坡下泄(去向=下坡方位);高处出现弱反向回流。"""
    w = PrandtlSlopeWind(180.0, 4.0, 6.0, katabatic=True)  # 下坡=去向南 = -x(NED)
    near = w.wind_ned(np.zeros(3), 6.0, 0.0)
    assert near[0] < 0 and abs(near[1]) < 1e-9      # 指向 -x(南),沿坡
    # z/λ > π 处 sin<0 → 反向(补偿回流)。λ = 4*6/π ≈ 7.64, π*λ ≈ 24m
    high = w.wind_ned(np.zeros(3), 26.0, 0.0)
    assert high[0] > 0, "高处应出现弱反向回流"


def test_prandtl_anabatic_reverses():
    kata = PrandtlSlopeWind(180.0, 3.0, 8.0, katabatic=True).wind_ned(np.zeros(3), 8.0, 0.0)
    ana = PrandtlSlopeWind(180.0, 3.0, 8.0, katabatic=False).wind_ned(np.zeros(3), 8.0, 0.0)
    assert np.allclose(kata, -ana)                  # 上坡风与下坡风反号


def test_loglaw_profile():
    w = LogLawWind(u_ref_m_s=5.0, dir_to_deg=90.0, z0_m=1e-3, z_ref_m=10.0)
    assert abs(np.linalg.norm(w.wind_ned(np.zeros(3), 10.0, 0.0)) - 5.0) < 1e-9  # 参考高度=参考风速
    u2 = np.linalg.norm(w.wind_ned(np.zeros(3), 2.0, 0.0))
    u20 = np.linalg.norm(w.wind_ned(np.zeros(3), 20.0, 0.0))
    assert u2 < 5.0 < u20                            # 单调递增
    assert np.linalg.norm(w.wind_ned(np.zeros(3), 1e-4, 0.0)) < 1e-9  # z<=z0 无风
    v = w.wind_ned(np.zeros(3), 10.0, 0.0)
    assert v[1] > 0 and abs(v[0]) < 1e-9             # 去向东 = +y


def test_composite_is_sum():
    a = UniformWind(2.0, 0.0)      # 向北 +x
    b = UniformWind(3.0, 90.0)     # 向东 +y
    c = CompositeWind([a, b])
    assert np.allclose(c.wind_ned(np.zeros(3), 5.0, 0.0), [2.0, 3.0, 0.0])


def test_atmosphere_wind_field_integration_and_determinism():
    """接入 IsaAtmosphere 后逐点求值,且固定种子逐位可复现。"""
    def build():
        cfg = AtmosphereConfig(model="isa", ground_alt_m=4000.0, turbulence_intensity=1.0)
        atm = IsaAtmosphere(cfg, seed=7)
        atm.wind_field = glacier_katabatic(downhill_azimuth_deg=180.0,
                                           peak_speed_m_s=5.0, jet_height_m=6.0)
        return atm
    atm1, atm2 = build(), build()
    # 不同高度处平均风不同(空间结构)
    lo = atm1.get_state(np.array([0, 0, -6.0]), 0.0).wind_ned_m_s
    hi = atm1.get_state(np.array([0, 0, -30.0]), 0.0).wind_ned_m_s
    assert not np.allclose(lo, hi)
    # 逐位复现:两个同种子实例走同一序列
    seq1 = [atm1.get_state(np.array([0, 0, -8.0]), t * 0.01).wind_ned_m_s.copy()
            for t in range(200)]
    seq2 = [atm2.get_state(np.array([0, 0, -8.0]), t * 0.01).wind_ned_m_s.copy()
            for t in range(200)]
    assert np.array_equal(np.array(seq1), np.array(seq2))


def test_airship_drifts_downhill_in_katabatic():
    """无控飞艇置于下降风急流中,应朝下坡方向(−x, 南)漂移。"""
    cfg = default_config(0.0)
    cfg.atmosphere = AtmosphereConfig(model="isa", ground_alt_m=4000.0,
                                      turbulence_intensity=0.0)
    atm = IsaAtmosphere(cfg.atmosphere, seed=1)
    atm.wind_field = PrandtlSlopeWind(180.0, 5.0, 6.0, katabatic=True)  # 顺坡向南
    dyn = AirshipDynamics(cfg, atm)
    x = dyn.initial_state((0.0, 0.0, -6.0))            # 6m AGL,峰值急流处
    for _ in range(5000):                              # 5 s,无推力
        x = dyn.step_rk4(x, np.zeros(3), 0.0, cfg.dt_physics_s)
    assert np.all(np.isfinite(x))
    assert x[IDX_POS][0] < -0.05, f"应向下坡(−x)漂移,实得 N={x[IDX_POS][0]:.3f}"
    assert abs(x[IDX_POS][1]) < abs(x[IDX_POS][0])     # 主要沿坡,横向小
