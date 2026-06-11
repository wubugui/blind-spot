"""大气模块:ISA 对表、风层插值、湍流统计特性、阵风、氦气热力学。"""
import numpy as np

from airship_sim.atmosphere import (GustEvent, IsaAtmosphere, isa_state,
                                    wind_layers_high_altitude,
                                    wind_layers_low_altitude)
from airship_sim.config import AtmosphereConfig, default_config
from airship_sim.simulation import Simulation


def test_isa_sea_level():
    t, p, rho = isa_state(0.0)
    assert np.isclose(t, 288.15)
    assert np.isclose(p, 101325.0)
    assert np.isclose(rho, 1.2250, atol=1e-4)


def test_isa_11km_density_vs_table():
    # 标准大气表 11km:T=216.65K, p=22632 Pa, ρ=0.36392 kg/m³;要求误差 <0.1%
    t, p, rho = isa_state(11000.0)
    assert np.isclose(t, 216.65, atol=0.01)
    assert abs(p - 22632.1) / 22632.1 < 1e-3
    assert abs(rho - 0.36392) / 0.36392 < 1e-3


def test_isa_20km_25km():
    # 标准表(位势高度行):H=20km p=5474.9 Pa;H=25km T=221.65K, p=2511.0 Pa,
    # ρ=0.0394658 kg/m³(US Standard Atmosphere 1976)
    t20, p20, _ = isa_state(20000.0)
    assert np.isclose(t20, 216.65, atol=0.01)
    assert abs(p20 - 5474.9) / 5474.9 < 1e-3
    t25, p25, rho25 = isa_state(25000.0)
    assert np.isclose(t25, 221.65, atol=0.01)
    assert abs(p25 - 2511.0) / 2511.0 < 1e-3
    assert abs(rho25 - 0.0394658) / 0.0394658 < 1e-3


def test_isa_dT_offset():
    # 热天(ΔT>0):密度降低,气压剖面不变
    t0, p0, rho0 = isa_state(100.0, dT_K=0.0)
    t1, p1, rho1 = isa_state(100.0, dT_K=15.0)
    assert np.isclose(t1 - t0, 15.0)
    assert p1 == p0
    assert rho1 < rho0


def make_isa(layers=(), turb=0.0, seed=7) -> IsaAtmosphere:
    cfg = AtmosphereConfig(model="isa", wind_layers=layers,
                           turbulence_intensity=turb)
    return IsaAtmosphere(cfg, seed=seed)


def test_wind_layer_interpolation():
    atm = make_isa(layers=((0.0, 2.0, 0.0), (100.0, 4.0, 0.0)))
    w50 = atm.wind_mean_ned(50.0)
    assert np.isclose(w50[0], 3.0)          # 中点速度线性插值
    assert np.isclose(w50[1], 0.0, atol=1e-12)
    # 超出层范围钳位
    assert np.isclose(atm.wind_mean_ned(500.0)[0], 4.0)


def test_wind_direction_interpolation_wraparound():
    # 350° 与 10°(均近北)插值应仍指向北,而非反向(分量插值,无绕角问题)
    atm = make_isa(layers=((0.0, 2.0, 350.0), (100.0, 2.0, 10.0)))
    w = atm.wind_mean_ned(50.0)
    assert w[0] > 1.9                        # 北分量主导
    assert abs(w[1]) < 0.2


def test_presets():
    low = wind_layers_low_altitude(speed_10m_m_s=1.5)
    assert low[0][0] == 2.0 and low[-1][0] == 100.0
    speeds = [l[1] for l in low]
    assert all(np.diff(speeds) > 0)          # 幂律:风速随高度单调增
    assert np.isclose(dict((l[0], l[1]) for l in low)[10.0], 1.5)
    high = wind_layers_high_altitude()
    assert high[-1][0] == 30000.0
    assert max(l[1] for l in high) == 25.0   # 急流

def test_turbulence_statistics():
    # 一阶 Gauss-Markov 稳态统计:均值≈0,标准差≈σ。
    # 取较大风速(8m/s → τ=L/V≈6.6s)缩短相关时间,1000s ≈ 150 个独立样本,
    # 均值估计标准差 ≈ σ/√150,界取 0.3σ(≈3.7 个估计标准差,假阳性可忽略)
    atm = make_isa(layers=((0.0, 8.0, 90.0),), turb=1.0)
    sigma, L = atm._turb_params(10.0)
    pos = np.array([0.0, 0.0, -10.0])
    mean_w = atm.wind_mean_ned(10.0)
    samples = []
    for i in range(100000):
        s = atm.get_state(pos, i * 0.01)
        samples.append(s.wind_ned_m_s - mean_w)
    samples = np.array(samples)
    along = samples[:, 1]   # 风向 90°(吹向东),沿风分量为东分量
    assert sigma > 0.5      # 自检:地面机械湍流项生效(0.1×8m/s)
    assert abs(np.mean(along)) < 0.3 * sigma
    assert 0.85 * sigma < np.std(along) < 1.15 * sigma
    # 垂直分量按 0.7 折减
    assert 0.85 * 0.7 * sigma < np.std(samples[:, 2]) < 1.15 * 0.7 * sigma


def test_turbulence_reproducible():
    a1 = make_isa(layers=((0.0, 2.0, 0.0),), turb=1.0, seed=11)
    a2 = make_isa(layers=((0.0, 2.0, 0.0),), turb=1.0, seed=11)
    pos = np.array([0.0, 0.0, -10.0])
    seq1, seq2 = [], []
    for i in range(100):
        seq1.append(a1.get_state(pos, i * 0.01).wind_ned_m_s)
        seq2.append(a2.get_state(pos, i * 0.01).wind_ned_m_s)
    assert np.array_equal(np.array(seq1), np.array(seq2))   # 同种子逐位一致
    # reset 后从头逐位重现同一序列
    a1.reset()
    seq3 = [a1.get_state(pos, i * 0.01).wind_ned_m_s for i in range(100)]
    assert np.array_equal(np.array(seq1), np.array(seq3))


def test_gust_shape():
    g = GustEvent(t0_s=10.0, dir_to_deg=90.0, amplitude_m_s=1.0, duration_s=4.0)
    assert np.allclose(g.wind_ned(9.9), 0.0)
    assert np.allclose(g.wind_ned(14.1), 0.0)
    peak = g.wind_ned(12.0)                  # 半程处达到幅值
    assert np.isclose(peak[1], 1.0) and abs(peak[0]) < 1e-12
    mid = g.wind_ned(11.0)                   # 1/4 程:A/2(1-cos(π/2)) = A/2
    assert np.isclose(mid[1], 0.5)


def test_trigger_gust_via_atmosphere():
    atm = make_isa()
    pos = np.array([0.0, 0.0, -5.0])
    atm.trigger_gust(t_s=1.0, dir_to_deg=0.0, amplitude_m_s=0.8, duration_s=2.0)
    w = atm.get_state(pos, 2.0).wind_ned_m_s
    assert np.isclose(w[0], 0.8)
    assert np.allclose(atm.get_state(pos, 4.0).wind_ned_m_s, 0.0)


def test_buoyancy_from_isa_density_hover_trim():
    # 浮力改由 ISA 当地密度计算后,在地面 ISA 密度下配平的飞艇应仍近似配平
    cfg = default_config(heaviness_kg=0.0)
    cfg.atmosphere.model = "isa"
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0.0, 0.0, -1.5))
    x0 = sim.x.copy()
    dx = sim.dynamics.derivatives(x0, np.zeros(3), 0.0)
    # 1.5m 高度处 ISA 密度比海平面低约 0.014%(Δρ≈1.7e-4)→ 残余下沉加速度 ~8e-4 m/s²
    assert abs(dx[7:10][2]) < 1.5e-3


def _helium_cfg(solar_on: bool):
    cfg = default_config(heaviness_kg=0.005)
    cfg.atmosphere.model = "isa"
    cfg.helium.enabled = True
    cfg.helium.solar_on = solar_on
    # 测试加速:cp 缩小 10 倍把热时间常数从 ~48s 缩到 ~4.8s,
    # 模型方程不变,平衡 superheat 与 cp 无关
    cfg.helium.cp_J_kgK = 519.3
    return cfg


def test_helium_superheat_increases_lift():
    cfg = _helium_cfg(solar_on=True)
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0.0, 0.0, -1.5))
    m0 = sim.helium_m_kg
    sim.run(10.0, record=False)              # ≈ 2 个时间常数
    h = cfg.helium
    superheat = sim.helium_T_K - sim.atmosphere.get_state(
        sim.x[0:3], sim.t_s).temperature_K
    # 平衡 superheat = Q_solar/(h·A_surf) ≈ 20K;2τ 后应达到 (1-e⁻²)≈0.86 平衡值
    eq = h.absorptivity * h.solar_flux_W_m2 * sim._A_proj_m2 / (
        h.h_film_W_m2K * sim._A_surf_m2)
    assert 15.0 < eq < 25.0                  # 量级自检(实测飞艇 superheat 10~30K)
    assert 0.7 * eq < superheat < eq
    assert sim.helium_m_kg < m0              # 排气 → 变轻 → 净浮力增大


def test_helium_night_returns_to_ambient():
    cfg = _helium_cfg(solar_on=False)
    sim = Simulation(cfg)
    sim.reset(p_ned_m=(0.0, 0.0, -1.5))
    sim.helium_T_K += 10.0                   # 人为加 10K superheat
    sim.run(15.0, record=False)              # ≈ 3 个时间常数
    t_amb = sim.atmosphere.get_state(sim.x[0:3], sim.t_s).temperature_K
    assert abs(sim.helium_T_K - t_amb) < 1.0  # 夜间冷却回环境温度