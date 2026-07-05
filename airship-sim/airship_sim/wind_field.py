"""环境风场 — 空间结构化的平均风。

物理引擎只通过 Atmosphere.get_state(position_ned_m, time_s) 取风,风本就是
位置与时间的函数,因此可在此加入空间结构化风场,动力学代码零改动。

面向低空(2~50m)工程验证的解析风场:
  UniformWind      常值风(调试/对照)
  LogLawWind       中性边界层对数风廓线,粗糙度 z0 随下垫面
  PrandtlSlopeWind 冰川/坡地下降风(katabatic)或上坡风(anabatic)的 Prandtl 解析解
  CompositeWind    分量线性叠加

坐标:NED;风矢量为"去向"(0=吹向北 +x,90=吹向东 +y),与 atmosphere.py 一致。
高度用离地高度 AGL(= −z,z=0 取放飞点地面),低空小坡度下近似坡面法向距离。

[假设] 各分量线性叠加、互不作用(边界层 + 坡风简单相加)
/ 弱风、缓坡下合理 / 强风急坡强耦合时需数值风场(RANS/LES 或实测)。
"""
from __future__ import annotations

import math

import numpy as np

_KARMAN = 0.4
# Prandtl 归一化廓线 f(s)=e^{-s} sin(s) 在 s=π/4 处取峰值,用于把峰值标定到 U_peak
_PRANDTL_PEAK = math.exp(-math.pi / 4.0) * math.sin(math.pi / 4.0)  # ≈ 0.32239


def _azimuth_unit(dir_to_deg: float) -> np.ndarray:
    """去向方位角 → NED 水平单位矢量 (N=+x, E=+y)。"""
    a = math.radians(dir_to_deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


class WindField:
    """环境风场接口:给定位置/离地高度/时间,返回平均风矢量(NED, m/s)。"""

    def wind_ned(self, pos_ned_m: np.ndarray, height_agl_m: float,
                 time_s: float) -> np.ndarray:
        raise NotImplementedError


class UniformWind(WindField):
    """常值均匀风(去向方位角 + 风速)。对照/调试用。"""

    def __init__(self, speed_m_s: float, dir_to_deg: float):
        self._w = float(speed_m_s) * _azimuth_unit(dir_to_deg)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        return self._w.copy()


class LogLawWind(WindField):
    """中性层结对数边界层:u(z) = u_ref · ln(z/z0) / ln(z_ref/z0),方向常值。

    z0 粗糙长度(m)典型值:雪/冰 ~2e-4,水面 ~1e-3(随浪),草地 ~0.03,
    灌木/碎石 ~0.1。低于 z0 处风速取 0。

    [假设] 中性层结、平坦均匀下垫面、定常 / 近地面 10~100m 内良好
    / 强稳定(冰川夜间)或强对流时对数律偏离,应叠加坡风/热泡分量。
    """

    def __init__(self, u_ref_m_s: float, dir_to_deg: float,
                 z0_m: float = 1e-3, z_ref_m: float = 10.0):
        self._z0 = max(float(z0_m), 1e-6)
        self._zref = max(float(z_ref_m), self._z0 * 2.0)
        self._uref = float(u_ref_m_s)
        self._dir = _azimuth_unit(dir_to_deg)
        self._denom = math.log(self._zref / self._z0)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        z = max(float(height_agl_m), self._z0)
        u = self._uref * math.log(z / self._z0) / self._denom
        return max(u, 0.0) * self._dir

    def u_star_m_s(self) -> float:
        """摩擦速度 u* = κ·u_ref / ln(z_ref/z0)(供湍流强度标定)。"""
        return _KARMAN * self._uref / self._denom


class PrandtlSlopeWind(WindField):
    """冰川/坡地热力驱动的沿坡风(Prandtl 1942 稳态坡风解析解)。

    冷却坡面(冰川、夜间)→ 贴地冷空气顺坡下泄(katabatic);受热坡面(白天)
    → 上坡风(anabatic)。沿坡速度随离地高度呈"低空急流"结构:
        u(z) = U_peak · e^{-z/λ} sin(z/λ) / f_max,   f_max = e^{-π/4} sin(π/4)
    在 z = π·λ/4 处达峰值 U_peak,之上衰减并出现弱反向回流(补偿流,物理真实)。
    λ = 4·z_peak/π 为 Prandtl 尺度(由急流峰高决定)。

    [假设] 无限均匀坡、稳态、常湍流扩散系数(Prandtl 经典解)
    / 冰川/缓坡近地面下降风的一阶垂直结构正确;峰高/峰速需实测,或按坡度、
      表面冷却强度、层结估算 / 复杂地形、非稳态爆发(katabatic surge)时失效。
    [假设] 只随离地高度变化,沿坡方向均匀(不含 fetch 加速)
    / 局地验证足够 / 需沿坡增强时叠加水平梯度分量。
    """

    def __init__(self, downhill_azimuth_deg: float, peak_speed_m_s: float,
                 jet_height_m: float, katabatic: bool = True):
        self._peak = float(peak_speed_m_s)
        self._lambda = max(4.0 * float(jet_height_m) / math.pi, 1e-3)
        sign = 1.0 if katabatic else -1.0   # katabatic 顺坡下泄(去向=下坡);anabatic 反向
        self._dir = sign * _azimuth_unit(downhill_azimuth_deg)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        z = max(float(height_agl_m), 0.0)
        s = z / self._lambda
        speed = self._peak * math.exp(-s) * math.sin(s) / _PRANDTL_PEAK
        return speed * self._dir

    @property
    def jet_lambda_m(self) -> float:
        return self._lambda


class CompositeWind(WindField):
    """分量线性叠加(边界层 + 坡风 + …)。"""

    def __init__(self, components):
        self._c = list(components)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        w = np.zeros(3)
        for c in self._c:
            w = w + c.wind_ned(pos_ned_m, height_agl_m, time_s)
        return w


# ---------- 环境预设 ----------
def glacier_katabatic(downhill_azimuth_deg: float = 180.0,
                      peak_speed_m_s: float = 4.0, jet_height_m: float = 6.0,
                      base_wind_m_s: float = 1.0, z0_m: float = 2e-4) -> WindField:
    """冰川下降风:光滑冰面对数边界层 + 顺坡 Prandtl 急流。

    典型冰川夜/日持续下降风峰速 2~8 m/s,急流峰高数米~十几米。默认下坡方位
    180°(向南)。z0=2e-4 对应雪/冰面。
    """
    return CompositeWind([
        LogLawWind(base_wind_m_s, downhill_azimuth_deg, z0_m=z0_m),
        PrandtlSlopeWind(downhill_azimuth_deg, peak_speed_m_s, jet_height_m,
                         katabatic=True),
    ])


def plateau_slope(downhill_azimuth_deg: float = 180.0, peak_speed_m_s: float = 3.0,
                  jet_height_m: float = 10.0, daytime: bool = False,
                  base_wind_m_s: float = 2.0, z0_m: float = 0.03) -> WindField:
    """高原坡地风:白天上坡(anabatic)、夜间下坡(katabatic);草/碎石 z0≈0.03。"""
    return CompositeWind([
        LogLawWind(base_wind_m_s, downhill_azimuth_deg, z0_m=z0_m),
        PrandtlSlopeWind(downhill_azimuth_deg, peak_speed_m_s, jet_height_m,
                         katabatic=not daytime),
    ])


def lake_surface(u10_m_s: float = 3.0, dir_to_deg: float = 90.0,
                 z0_m: float = 1e-3) -> WindField:
    """湖面:水面对数边界层(低粗糙度)。湖陆风锋面待后续加入。"""
    return LogLawWind(u10_m_s, dir_to_deg, z0_m=z0_m, z_ref_m=10.0)
