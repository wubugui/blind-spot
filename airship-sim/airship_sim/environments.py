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

from .atmosphere import isa_state
from .config import (GRAVITY_M_S2, AtmosphereConfig, SimConfig, default_config)


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
            solar_albedo_load: bool = False, seed: int = 20260611) -> SimConfig:
    """冰川:低温高密度空气 + 顺坡下降风急流 + 光滑冰面。

    solar_albedo_load=True 时叠加冰面高反照的太阳负荷(氦气超温)。
    """
    cfg = default_config(heaviness_kg)
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
            heaviness_kg: float = 0.005, seed: int = 20260611) -> SimConfig:
    """高原:低密度(浮力/推力显著下降)+ 白天上坡风+热泡 / 夜间下降风 + 强日照。"""
    cfg = default_config(heaviness_kg)
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
         heaviness_kg: float = 0.005, seed: int = 20260611) -> SimConfig:
    """湖泊:水面低粗糙度边界层 + 可选湖陆风推进锋面(白天湖→陆)。"""
    cfg = default_config(heaviness_kg)
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
