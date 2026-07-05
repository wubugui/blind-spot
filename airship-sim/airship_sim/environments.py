"""环境预设 — 一次调用配好一个极端环境的完整 SimConfig。

把"环境密度(海拔+气温)+ 空间风场 + 高原配平 + 太阳/热负荷"打包,返回可直接
Simulation(cfg) 运行的配置。密度对浮力/气动/推力的影响自动生效;吊舱质量按当地
密度重新配平(相当于为该海拔选定合适的气囊/载荷)。

低空飞艇(2~50m AGL),三类环境:
  glacier(...)  冰川:低温高密度 + 顺坡下降风急流 + 冰面高反照太阳负荷
  plateau(...)  高原:低密度(浮力/推力显著下降)+ 白天上坡风/热泡或夜间下降风
  lake(...)     湖泊:水面低粗糙度 + 湖陆风推进锋面

也可用 environment(name, ...) 统一入口。所有返回的 cfg 可 JSON 往返(含风场规格)。
"""
from __future__ import annotations

import numpy as np

from .atmosphere import isa_state
from .config import (GRAVITY_M_S2, AtmosphereConfig, PidGains, SimConfig,
                     default_config)
from .math3d import quat_from_euler


def heavy_airframe(cfg: SimConfig) -> SimConfig:
    """抗风重型机型:加大气囊/电机/尾翼 + 重整定增益,可在极端环境定点悬停。

    相对默认(2.0×0.8m、0.05N 电机、0.15m² 尾翼、~0.5m/s 抗风):
      囊体 2.8×1.3m(浮力/容纳更大);电机 3N(推力↑60×)、横距 0.6m(偏航权限↑);
      尾翼 1.5m²(S·CLα·|x| 超 2(k2-k1)V,静稳定/自动迎风);增益按新权限重整定;
      定点死区 3m(欠驱动无侧推,守半径而非守点,避免目标附近方位角急甩)。
    验证(env_cases,代表性强风):冰川≈2.7m、高原≈0.9m、湖泊≈2.0m 定点漂移。
    配合"迎风起始朝向"(heading_into_wind_rad)使用——否则起飞背风、掉头期间被吹散。

    [假设] 这是"能扛住"所需的机型量级结论(大而有力),参数仍需实测标定;
    3m 电机对 2.8m 模型偏大但物理上就是低空 3~4m/s 定点所需。
    """
    cfg.hull.length_m = 2.8
    cfg.hull.diameter_m = 1.3
    cfg.mass.m_envelope_kg = 0.28          # ≈7.3m² 表面 × 38g/m²(更厚蒙皮+更大尾翼)
    a = cfg.actuator
    a.thrust_max_N = 3.0
    a.motor_sep_y_m = 0.6
    f = cfg.fins
    f.s_plane_m2 = 1.5
    f.x_fin_m = -1.4
    T = a.thrust_max_N
    tau = 2.0 * T * a.motor_sep_y_m
    c = cfg.control
    c.alt = PidGains(kp=1.5, ki=0.1, kd=3.5, tau_d_s=1.0, tau_meas_s=0.5,
                     i_band=2.0, out_min=-T, out_max=T)
    c.yaw = PidGains(kp=2.0, ki=0.12, kd=4.0, tau_d_s=0.4, i_band=1.5,
                     out_min=-tau, out_max=tau)
    c.speed = PidGains(kp=4.0, ki=0.6, kd=0.6, out_min=-2 * T, out_max=2 * T)
    c.pos_hold_kp_1_s = 0.5
    c.pos_hold_max_speed_m_s = 3.0
    c.pos_hold_yaw_rate_max_rad_s = 0.5
    c.pos_hold_deadband_m = 3.0
    return cfg


def heading_into_wind_rad(atmosphere, pos_ned_m, t_s: float = 0.0) -> float:
    """迎风朝向(rad):机头指向风的来向(= 风去向方位 + π)。定点前如此初始化,
    可避免起飞背风、掉头穿越 ±180° 期间被吹散。"""
    w = atmosphere.get_state(np.asarray(pos_ned_m, dtype=float), t_s).wind_ned_m_s
    return float(np.arctan2(w[1], w[0]) + np.pi)


def heading_into_wind_quat(atmosphere, pos_ned_m, t_s: float = 0.0):
    """迎风朝向四元数(reset 用)。"""
    return quat_from_euler(0.0, 0.0, heading_into_wind_rad(atmosphere, pos_ned_m, t_s))


def trim_for_altitude(cfg: SimConfig, ground_alt_m: float, dT_K: float,
                      heaviness_kg: float = 0.005) -> float:
    """把吊舱质量配平到:总重 = 当地密度浮力 + heaviness。返回当地密度。

    当地密度不足以配平(气囊太小)时抛错并提示加大气囊/减配重。
    """
    _, _, rho = isa_state(ground_alt_m, dT_K)
    v = cfg.hull.volume_m3
    m_he = cfg.mass.rho_helium_kg_m3 * v
    m_gondola = rho * v + heaviness_kg - cfg.mass.m_envelope_kg - m_he
    if m_gondola <= 0.0:
        raise ValueError(
            f"海拔 {ground_alt_m:.0f}m 密度 {rho:.3f} kg/m³ 下,气囊体积 {v:.2f} m³ "
            f"不足以配平(蒙皮+氦气已超浮力)——需加大气囊或减配重。")
    cfg.mass.m_gondola_kg = m_gondola
    cfg.control.alt_ff_N = heaviness_kg * GRAVITY_M_S2
    return rho


def _apply(cfg: SimConfig, ground_alt_m: float, air_temp_C: float | None,
           wind_field: dict, turbulence: float, heaviness_kg: float,
           solar: bool, solar_flux_W_m2: float) -> SimConfig:
    # ΔT:目标气温相对该海拔 ISA 标准温度的偏移(None=标准大气)
    t_std, _, _ = isa_state(ground_alt_m, 0.0)
    dT = 0.0 if air_temp_C is None else (air_temp_C + 273.15) - t_std
    cfg.atmosphere = AtmosphereConfig(
        model="isa", ground_alt_m=ground_alt_m, dT_K=dT,
        turbulence_intensity=turbulence, wind_field=wind_field)
    trim_for_altitude(cfg, ground_alt_m, dT, heaviness_kg)
    if solar:
        cfg.helium.enabled = True
        cfg.helium.solar_on = True
        cfg.helium.solar_flux_W_m2 = solar_flux_W_m2
    return cfg


def glacier(ground_alt_m: float = 4000.0, air_temp_C: float = -12.0,
            downhill_azimuth_deg: float = 180.0, katabatic_peak_m_s: float = 4.0,
            jet_height_m: float = 6.0, fetch_gain_per_m: float = 0.0,
            turbulence: float = 0.4, heaviness_kg: float = 0.005,
            solar_albedo_load: bool = False, airframe: str = "default",
            seed: int = 20260611) -> SimConfig:
    """冰川:低温高密度空气 + 顺坡下降风急流 + 光滑冰面。

    solar_albedo_load=True 时叠加冰面高反照的太阳负荷(氦气超温)。
    airframe="heavy" 用抗风重型机型(能定点悬停,配合迎风起始朝向)。
    """
    cfg = default_config(heaviness_kg)
    if airframe == "heavy":
        heavy_airframe(cfg)
    cfg.seed = seed
    spec = {"kind": "glacier_katabatic", "params": {
        "downhill_azimuth_deg": downhill_azimuth_deg,
        "peak_speed_m_s": katabatic_peak_m_s, "jet_height_m": jet_height_m,
        "base_wind_m_s": 1.0, "z0_m": 2e-4, "fetch_gain_per_m": fetch_gain_per_m}}
    # 冰面反照:等效抬高太阳辐照(反射叠加)
    return _apply(cfg, ground_alt_m, air_temp_C, spec, turbulence, heaviness_kg,
                  solar=solar_albedo_load, solar_flux_W_m2=1300.0)


def plateau(ground_alt_m: float = 4500.0, air_temp_C: float = 5.0,
            downhill_azimuth_deg: float = 180.0, slope_peak_m_s: float = 3.0,
            jet_height_m: float = 10.0, daytime: bool = True,
            thermals: bool = True, thermal_updraft_m_s: float = 2.0,
            base_wind_m_s: float = 2.0, turbulence: float = 0.6,
            heaviness_kg: float = 0.005, airframe: str = "default",
            seed: int = 20260611) -> SimConfig:
    """高原:低密度(浮力/推力显著下降)+ 白天上坡风+热泡 / 夜间下降风 + 强日照。

    airframe="heavy" 用抗风重型机型(能定点悬停,配合迎风起始朝向)。
    """
    cfg = default_config(heaviness_kg)
    if airframe == "heavy":
        heavy_airframe(cfg)
    cfg.seed = seed
    spec = {"kind": "plateau_slope", "params": {
        "downhill_azimuth_deg": downhill_azimuth_deg, "peak_speed_m_s": slope_peak_m_s,
        "jet_height_m": jet_height_m, "daytime": daytime, "base_wind_m_s": base_wind_m_s,
        "z0_m": 0.03, "thermals": thermals, "thermal_updraft_m_s": thermal_updraft_m_s}}
    return _apply(cfg, ground_alt_m, air_temp_C, spec, turbulence, heaviness_kg,
                  solar=daytime, solar_flux_W_m2=1361.0)


def lake(ground_alt_m: float = 0.0, air_temp_C: float | None = None,
         onshore_dir_deg: float = 0.0, surface_wind_m_s: float = 3.0,
         lake_breeze: bool = True, inflow_speed_m_s: float = 3.0,
         front_speed_m_s: float = 2.0, turbulence: float = 0.3,
         heaviness_kg: float = 0.005, airframe: str = "default",
         seed: int = 20260611) -> SimConfig:
    """湖泊:水面低粗糙度边界层 + 可选湖陆风推进锋面(白天湖→陆)。

    airframe="heavy" 用抗风重型机型(能定点悬停,配合迎风起始朝向)。
    """
    cfg = default_config(heaviness_kg)
    if airframe == "heavy":
        heavy_airframe(cfg)
    cfg.seed = seed
    spec = {"kind": "lake_env", "params": {
        "onshore_dir_deg": onshore_dir_deg, "u10_m_s": surface_wind_m_s,
        "breeze": lake_breeze, "inflow_speed_m_s": inflow_speed_m_s,
        "front_speed_m_s": front_speed_m_s, "z0_m": 1e-3}}
    return _apply(cfg, ground_alt_m, air_temp_C, spec, turbulence, heaviness_kg,
                  solar=False, solar_flux_W_m2=1000.0)


_ENVIRONMENTS = {"glacier": glacier, "plateau": plateau, "lake": lake}


def environment(name: str, **kwargs) -> SimConfig:
    """统一入口:environment("glacier"|"plateau"|"lake", **参数)。"""
    if name not in _ENVIRONMENTS:
        raise ValueError(f"unknown environment {name!r}; choose from {list(_ENVIRONMENTS)}")
    return _ENVIRONMENTS[name](**kwargs)
