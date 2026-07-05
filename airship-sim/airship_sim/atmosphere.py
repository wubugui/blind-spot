"""大气模块 — 统一接口与各模型实现。

物理引擎只通过 Atmosphere.get_state(position, time) 取大气数据,
保证可整体替换(constant ↔ ISA 分层风/湍流,将来 ERA5 再分析数据)而不改动力学代码。

实现:
  ConstantAtmosphere — 常密度 + 常值风(室内/单元测试)
  IsaAtmosphere      — ISA 0~32km 剖面 + 分层风(线性插值) + Gauss-Markov 湍流 + 1-cos 阵风

== ISA 标准大气(ISO 2533 / US Standard Atmosphere 1976,0~32km 三层) ==
  0~11 km   对流层:   T = 288.15 − 6.5e-3·h,   p 按多方关系
  11~20 km  平流层底等温层: T = 216.65,        p 按指数律
  20~32 km  平流层升温段:  T = 216.65 + 1.0e-3·(h−20000)
  ρ = p / (R_air·(T + ΔT)),R_air = 287.05287 J/(kg·K)
  (ΔT 温度偏移见 AtmosphereConfig 注释)

== 湍流模型 ==
[假设] 用一阶 Gauss-Markov 过程近似 Dryden 谱(纵向分量谱形同构,横/竖向二阶谱
  简化为一阶):x_{k+1} = x_k·e^{−Δt/τ} + σ√(1−e^{−2Δt/τ})·N(0,1),τ = L/V
/ 方差与积分时间尺度与 Dryden 一致,仅高频谱斜率略缓
/ 需要精确谱响应(结构载荷/抖振分析)时换完整 Dryden 成形滤波器。
强度 σ 与尺度 L 随高度变化(地面边界层强、平流层平稳),剖面见 _TURB_PROFILE,
参考 MIL-HDBK-1797 低空模型量级,系数为工程估值并随 turbulence_intensity 整体缩放。
[假设] 湍流分量在(沿平均风、横风、垂直)风轴系生成后转到 NED;冻结湍流假设,
  以平均风速(下限 0.5m/s)作为对流速度 / 悬停低速下合理 / 高速穿越湍流场时需用空速。

== 阵风 ==
1-cosine 脉冲:v(t) = A/2·(1−cos(2π(t−t₀)/T)),t∈[t₀, t₀+T],水平方向可配,
多个阵风事件叠加。可手动触发(trigger_gust)或随机触发(可配平均间隔)。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import AtmosphereConfig

R_AIR_J_KGK = 287.05287
G0_M_S2 = 9.80665

# ISA 各层:(底高度 m, 底温度 K, 温度梯度 K/m, 底压力 Pa)
_T0_K, _P0_PA = 288.15, 101325.0
_ISA_LAYERS = []


def _build_isa_layers():
    layers = []
    h_b, t_b, p_b = 0.0, _T0_K, _P0_PA
    for h_top, lapse in ((11000.0, -6.5e-3), (20000.0, 0.0), (32000.0, 1.0e-3)):
        layers.append((h_b, t_b, lapse, p_b))
        dh = h_top - h_b
        if lapse == 0.0:
            p_top = p_b * np.exp(-G0_M_S2 * dh / (R_AIR_J_KGK * t_b))
        else:
            p_top = p_b * ((t_b + lapse * dh) / t_b) ** (-G0_M_S2 / (R_AIR_J_KGK * lapse))
        h_b, t_b, p_b = h_top, t_b + lapse * dh, p_top
    return layers


_ISA_LAYERS = _build_isa_layers()


def isa_state(alt_m: float, dT_K: float = 0.0) -> tuple[float, float, float]:
    """ISA 剖面:位势海拔 → (温度 K, 气压 Pa, 密度 kg/m³)。范围 0~32km,超界钳位。

    [假设] 输入按位势高度处理(ISA 分层与标准表按位势高度定义),
    不做几何高度→位势高度换算(H = z·Re/(Re+z))
    / 25km 处两者差 ~100m,密度差 ~0.4%;低空(<2km)差 <0.03% 可忽略
    / 与按几何高度索引的表格对照、或 30km 级任务精算时需先换算。
    """
    h = float(np.clip(alt_m, 0.0, 32000.0))
    for h_b, t_b, lapse, p_b in reversed(_ISA_LAYERS):
        if h >= h_b:
            break
    dh = h - h_b
    if lapse == 0.0:
        t = t_b
        p = p_b * np.exp(-G0_M_S2 * dh / (R_AIR_J_KGK * t_b))
    else:
        t = t_b + lapse * dh
        p = p_b * (t / t_b) ** (-G0_M_S2 / (R_AIR_J_KGK * lapse))
    t_eff = t + dT_K
    rho = p / (R_AIR_J_KGK * t_eff)
    return float(t_eff), float(p), float(rho)


@dataclass
class AtmosphereState:
    """某一点、某一时刻的大气状态(SI)。"""
    rho_kg_m3: float
    temperature_K: float
    pressure_Pa: float
    wind_ned_m_s: np.ndarray         # 风矢量(世界系 NED,含湍流与阵风分量)


class Atmosphere:
    """大气模型基类/接口。"""

    def get_state(self, position_ned_m: np.ndarray, time_s: float) -> AtmosphereState:
        raise NotImplementedError

    def set_uniform_wind(self, wind_ned_m_s) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        """仿真重置时清除内部时变状态(湍流/阵风),无状态模型为 no-op。"""


class ConstantAtmosphere(Atmosphere):
    """常密度、常温压、常值风。适用于室内场景与单元测试。

    [假设] 室内空气密度均匀恒定 / 室内 0~5m 高差密度变化 <0.06%,可忽略
    / 户外或高空场景失效,届时用 IsaAtmosphere。
    """

    def __init__(self, cfg: AtmosphereConfig):
        self._rho_kg_m3 = cfg.rho_const_kg_m3
        self._temperature_K = cfg.temperature_const_K
        self._pressure_Pa = cfg.pressure_const_Pa
        self._wind_ned_m_s = np.array(cfg.wind_ned_m_s, dtype=float)

    def get_state(self, position_ned_m: np.ndarray, time_s: float) -> AtmosphereState:
        return AtmosphereState(
            rho_kg_m3=self._rho_kg_m3,
            temperature_K=self._temperature_K,
            pressure_Pa=self._pressure_Pa,
            wind_ned_m_s=self._wind_ned_m_s.copy(),
        )

    def set_uniform_wind(self, wind_ned_m_s) -> None:
        self._wind_ned_m_s = np.array(wind_ned_m_s, dtype=float)


# 湍流强度/尺度剖面:(海拔 m, σ m/s, L m)。线性插值,见模块 docstring。
# σ 地面段取 0.1×(6m 高度平均风),无风时下限 0.05;以下为 intensity=1 的基准剖面。
_TURB_PROFILE = (
    # alt_m, sigma_base_m_s, L_m
    (0.0,     0.30,   50.0),    # 地面边界层:机械湍流强
    (300.0,   0.50,  150.0),
    (2000.0,  0.50,  300.0),    # 对流层中部
    (9000.0,  0.70,  500.0),    # 急流附近风切变区
    (12000.0, 0.50,  500.0),
    (18000.0, 0.10,  500.0),    # 平流层:平稳
    (32000.0, 0.05,  500.0),
)


@dataclass
class GustEvent:
    t0_s: float
    dir_to_deg: float        # 吹向方位角(0=北)
    amplitude_m_s: float
    duration_s: float

    def wind_ned(self, t_s: float) -> np.ndarray:
        if not (self.t0_s <= t_s <= self.t0_s + self.duration_s):
            return np.zeros(3)
        v = 0.5 * self.amplitude_m_s * (
            1.0 - np.cos(2.0 * np.pi * (t_s - self.t0_s) / self.duration_s))
        a = np.deg2rad(self.dir_to_deg)
        return v * np.array([np.cos(a), np.sin(a), 0.0])


class IsaAtmosphere(Atmosphere):
    """ISA 剖面 + 分层风 + 湍流 + 阵风。参数可运行时修改(调参面板)。"""

    def __init__(self, cfg: AtmosphereConfig, seed: int = 0):
        self.cfg = cfg
        self.dT_K = cfg.dT_K
        self.ground_alt_m = cfg.ground_alt_m
        self.turbulence_intensity = cfg.turbulence_intensity
        self.wind_layers = [list(l) for l in cfg.wind_layers]  # 可变,面板实时改
        self._seed = seed
        self._rng = np.random.default_rng(seed)
        self._gusts: list[GustEvent] = []
        # 湍流状态(沿风/横风/垂直三分量),固定步长推进、查询间保持
        self._turb_state = np.zeros(3)
        self._turb_t_s = 0.0
        self._dt_turb_s = cfg.turb_update_dt_s
        # 可选空间结构化平均风(wind_field.WindField);None 时用分层风 wind_layers。
        # 物理引擎经 get_state(position, time) 逐点求值,动力学代码零改动。
        self.wind_field = None

    def reset(self) -> None:
        """清除湍流/阵风状态并重建 RNG,保证 reset 后轨迹与新建实例逐位一致。"""
        self._rng = np.random.default_rng(self._seed)
        self._turb_state[:] = 0.0
        self._turb_t_s = 0.0
        self._gusts = []

    # ---- 平均风 ----
    def wind_mean_ned(self, alt_m: float) -> np.ndarray:
        if not self.wind_layers:
            return np.zeros(3)
        # 按 N/E 分量插值(直接插值方向角在 ±180° 处会绕错方向)
        alts = [l[0] for l in self.wind_layers]
        comps = []
        for _, spd, dir_deg in self.wind_layers:
            a = np.deg2rad(dir_deg)
            comps.append((spd * np.cos(a), spd * np.sin(a)))
        comps = np.array(comps)
        n = np.interp(alt_m, alts, comps[:, 0])
        e = np.interp(alt_m, alts, comps[:, 1])
        return np.array([n, e, 0.0])

    # ---- 湍流 ----
    def _turb_params(self, alt_m: float,
                     local_speed_m_s: float | None = None) -> tuple[float, float]:
        prof = np.array(_TURB_PROFILE)
        sigma = float(np.interp(alt_m, prof[:, 0], prof[:, 1]))
        L = float(np.interp(alt_m, prof[:, 0], prof[:, 2]))
        # 地面段 σ 随当地平均风加强(机械湍流):σ = max(基准, 0.1×W6m)。
        # 有空间风场时用当地风速;否则用分层风在 6m 的值。
        if alt_m < 300.0:
            w6 = (local_speed_m_s if local_speed_m_s is not None
                  else float(np.linalg.norm(self.wind_mean_ned(6.0))))
            sigma = max(sigma, 0.1 * w6)
        return sigma * self.turbulence_intensity, L

    def _advance_turbulence(self, t_s: float, alt_m: float,
                            mean_speed_m_s: float | None = None) -> None:
        """固定步长推进 Gauss-Markov 状态;查询时间早于内部时间则保持(不回退)。"""
        while t_s >= self._turb_t_s + self._dt_turb_s:
            self._turb_t_s += self._dt_turb_s
            sigma, L = self._turb_params(alt_m, mean_speed_m_s)
            if sigma <= 0.0:
                self._turb_state[:] = 0.0
                continue
            v_adv = max(mean_speed_m_s if mean_speed_m_s is not None
                        else float(np.linalg.norm(self.wind_mean_ned(alt_m))), 0.5)
            tau = L / v_adv
            phi = np.exp(-self._dt_turb_s / tau)
            q = sigma * np.sqrt(1.0 - phi * phi)
            self._turb_state = phi * self._turb_state + q * self._rng.standard_normal(3)

    def turbulence_ned(self, alt_m: float, mean: np.ndarray | None = None) -> np.ndarray:
        """当前湍流分量(风轴系 → NED)。mean 给定时用它定沿风轴(空间风场用)。

        [假设] 垂直分量 σ_w = 0.7·σ_u(近地面垂直脉动受地面抑制的典型比值)
        / 与边界层观测一致 / 强对流(热泡)环境下垂直分量可反超水平分量。
        """
        if self.turbulence_intensity <= 0.0:
            return np.zeros(3)
        if mean is None:
            mean = self.wind_mean_ned(alt_m)
        spd = float(np.hypot(mean[0], mean[1]))
        if spd > 1e-6:
            ax = mean[:2] / spd          # 沿风单位矢量
        else:
            ax = np.array([1.0, 0.0])    # 无风时取北
        cross = np.array([-ax[1], ax[0]])
        u, v, w = self._turb_state       # w 向上为正
        return np.array([u * ax[0] + v * cross[0],
                         u * ax[1] + v * cross[1],
                         -0.7 * w])      # NED z 向下

    # ---- 阵风 ----
    def trigger_gust(self, t_s: float, dir_to_deg: float,
                     amplitude_m_s: float, duration_s: float = 3.0) -> None:
        self._gusts.append(GustEvent(t_s, dir_to_deg, amplitude_m_s, duration_s))
        # 清理已结束事件
        self._gusts = [g for g in self._gusts if g.t0_s + g.duration_s >= t_s]

    # ---- 接口 ----
    def get_state(self, position_ned_m: np.ndarray, time_s: float) -> AtmosphereState:
        alt_m = self.ground_alt_m - float(position_ned_m[2])   # z 向下
        t_K, p_Pa, rho = isa_state(alt_m, self.dT_K)
        if self.wind_field is not None:
            # 空间结构化风场:逐点求平均风,湍流按当地风速标定
            height_agl_m = -float(position_ned_m[2])   # 离地高度(z=0 为放飞点地面)
            mean = np.asarray(self.wind_field.wind_ned(
                np.asarray(position_ned_m, dtype=float), height_agl_m, time_s),
                dtype=float)
            self._advance_turbulence(time_s, alt_m, float(np.hypot(mean[0], mean[1])))
            wind = mean + self.turbulence_ned(alt_m, mean)
        else:
            self._advance_turbulence(time_s, alt_m)
            wind = self.wind_mean_ned(alt_m) + self.turbulence_ned(alt_m)
        for g in self._gusts:
            wind = wind + g.wind_ned(time_s)
        return AtmosphereState(rho_kg_m3=rho, temperature_K=t_K,
                               pressure_Pa=p_Pa, wind_ned_m_s=wind)

    def set_uniform_wind(self, wind_ned_m_s) -> None:
        """兼容接口:把所有风层设为同一矢量。"""
        w = np.asarray(wind_ned_m_s, dtype=float)
        spd = float(np.hypot(w[0], w[1]))
        dir_deg = float(np.rad2deg(np.arctan2(w[1], w[0])))
        if not self.wind_layers:
            self.wind_layers = [[0.0, spd, dir_deg]]
        else:
            for l in self.wind_layers:
                l[1], l[2] = spd, dir_deg


# ---- 预置风场 ----
def wind_layers_low_altitude(speed_10m_m_s: float = 1.5, dir_to_deg: float = 180.0,
                             alpha: float = 0.14) -> tuple:
    """低空模式(0~100m):幂律边界层 u(h) = u10·(h/10)^α。

    [假设] α=0.14(开阔地面典型;城市 0.25~0.4,水面 0.10)
    / 中性层结下良好 / 强对流或逆温时剖面显著偏离幂律。
    """
    layers = []
    for h in (2.0, 5.0, 10.0, 20.0, 40.0, 70.0, 100.0):
        layers.append((h, round(speed_10m_m_s * (h / 10.0) ** alpha, 3), dir_to_deg))
    return tuple(layers)


def wind_layers_high_altitude() -> tuple:
    """高空模式(0~30km):典型中纬度剖面,急流位于 10~12km,平流层风速回落。

    [假设] 风速/风向为中纬度气候平均量级(急流 25m/s),仅作演示初值
    / 用于联调与定性研究 / 任务级分析必须替换为当地探空或再分析数据。
    """
    return (
        (0.0,    3.0, 90.0),
        (2000.0, 8.0, 100.0),
        (4000.0, 12.0, 110.0),
        (7000.0, 18.0, 115.0),
        (10000.0, 25.0, 120.0),   # 急流
        (12000.0, 22.0, 120.0),
        (16000.0, 10.0, 115.0),
        (20000.0, 6.0, 100.0),
        (26000.0, 8.0, 80.0),
        (30000.0, 12.0, 70.0),
    )


def make_atmosphere(cfg: AtmosphereConfig, seed: int = 0) -> Atmosphere:
    """根据配置构建大气模型。seed 用于湍流(可复现,由 SimConfig.seed 派生)。"""
    if cfg.model == "constant":
        return ConstantAtmosphere(cfg)
    if cfg.model == "isa":
        return IsaAtmosphere(cfg, seed)
    raise ValueError(f"unknown atmosphere model: {cfg.model}")
