"""环境风场:Prandtl 坡风解析性质、对数律、叠加、与大气/动力学的集成。"""
import math

import numpy as np

from airship_sim.config import AtmosphereConfig, default_config
from airship_sim.atmosphere import IsaAtmosphere
from airship_sim.dynamics import AirshipDynamics, IDX_POS, IDX_VEL
from airship_sim.wind_field import (CompositeWind, LakeBreezeFront, LogLawWind,
                                    PrandtlSlopeWind, RidgeTerrain, TabulatedProfileWind,
                                    ThermalField, UniformWind, build_wind_field,
                                    glacier_katabatic)


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


def test_slope_fetch_enhancement():
    """沿坡 fetch:下坡越远峰速越大;上坡端不增强。"""
    w = PrandtlSlopeWind(180.0, 4.0, 6.0, katabatic=True,
                         fetch_gain_per_m=0.01, fetch_max=1.0)  # 下坡=去向南 -x
    # 下坡方向 = 去向南 = -x;沿下坡走 = N 减小
    near = np.linalg.norm(w.wind_ned(np.array([0.0, 0, 0]), 6.0, 0.0))
    far = np.linalg.norm(w.wind_ned(np.array([-50.0, 0, 0]), 6.0, 0.0))  # 顺坡下去 50m
    up = np.linalg.norm(w.wind_ned(np.array([+50.0, 0, 0]), 6.0, 0.0))
    assert far > near > 0
    assert abs(up - near) < 1e-9          # 上坡端不增强(clip 到 0)


def test_tabulated_profile_interpolates():
    w = TabulatedProfileWind.from_speed_dir([0, 10, 50], [0, 5, 10], [90, 90, 90])
    v10 = w.wind_ned(np.zeros(3), 10.0, 0.0)
    assert abs(np.linalg.norm(v10) - 5.0) < 1e-9 and v10[1] > 0  # 东向 5 m/s
    v5 = w.wind_ned(np.zeros(3), 5.0, 0.0)
    assert abs(np.linalg.norm(v5) - 2.5) < 1e-9                   # 线性插值
    v100 = w.wind_ned(np.zeros(3), 100.0, 0.0)                    # 超界钳位
    assert abs(np.linalg.norm(v100) - 10.0) < 1e-9


def test_lake_breeze_front_advances_and_updraft():
    """锋面随时间内陆推进;锋面带有上升(D<0);锋前无风。"""
    lb = LakeBreezeFront(onshore_dir_deg=0.0, shore_offset_m=0.0, inflow_speed_m_s=3.0,
                         depth_m=200.0, front_speed_m_s=2.0, front_width_m=100.0,
                         updraft_m_s=1.0)
    # onshore = 去向北 +x;陆地在 +x
    # t=0 锋面在 s=0;s=10(锋前)应无风
    assert np.allclose(lb.wind_ned(np.array([10.0, 0, 0]), 5.0, 0.0), 0.0)
    # t=20s 锋面推进到 s=40;此时 s=10 在锋后,有 onshore 入流(+x)
    w = lb.wind_ned(np.array([10.0, 0, 0]), 5.0, 20.0)
    assert w[0] > 0
    # 锋心附近(s≈front)有上升(D<0)
    wf = lb.wind_ned(np.array([40.0, 0, 0]), 5.0, 20.0)
    assert wf[2] < 0


def test_thermal_field_updraft_core_and_sink_ring():
    th = ThermalField(spacing_m=200.0, core_radius_m=40.0, peak_updraft_m_s=3.0, top_m=400.0)
    core = th.wind_ned(np.array([0.0, 0, 0]), 50.0, 0.0)     # 单元中心
    assert core[2] < 0                                        # 上升(D<0)
    ring = th.wind_ned(np.array([70.0, 0, 0]), 50.0, 0.0)    # 核外环
    assert ring[2] > 0                                        # 补偿下沉(D>0)
    top = th.wind_ned(np.array([0.0, 0, 0]), 2000.0, 0.0)    # 高处包络衰减
    assert abs(top[2]) < abs(core[2])


def test_ridge_speedup_and_determinism():
    base = LogLawWind(5.0, 90.0, z0_m=1e-3)   # 向东 +y
    ridge = RidgeTerrain(base, crest_ne_m=(0, 0), ridge_axis_deg=0.0,
                         height_m=30.0, halfwidth_m=100.0, z_influence_m=60.0)
    crest = np.linalg.norm(ridge.wind_ned(np.array([0.0, 0, 0]), 10.0, 0.0))
    far = np.linalg.norm(ridge.wind_ned(np.array([0.0, 500, 0]), 10.0, 0.0))
    base_spd = np.linalg.norm(base.wind_ned(np.array([0.0, 0, 0]), 10.0, 0.0))
    assert crest > base_spd                    # 脊顶加速
    assert abs(far - base_spd) < 0.1           # 远离脊回到基础风


def test_build_wind_field_spec_roundtrip():
    spec = {"kind": "composite", "components": [
        {"kind": "loglaw", "params": {"u_ref_m_s": 2.0, "dir_to_deg": 180.0, "z0_m": 2e-4}},
        {"kind": "slope", "params": {"downhill_azimuth_deg": 180.0, "peak_speed_m_s": 4.0,
                                     "jet_height_m": 6.0}},
    ]}
    wf = build_wind_field(spec)
    assert wf is not None
    v = wf.wind_ned(np.zeros(3), 6.0, 0.0)
    assert np.linalg.norm(v) > 3.0             # 坡风峰 + 边界层
    assert build_wind_field(None) is None
    # 预设也可声明式构建
    wf2 = build_wind_field({"kind": "glacier_katabatic",
                            "params": {"peak_speed_m_s": 5.0, "jet_height_m": 6.0}})
    assert np.linalg.norm(wf2.wind_ned(np.zeros(3), 6.0, 0.0)) > 4.0
