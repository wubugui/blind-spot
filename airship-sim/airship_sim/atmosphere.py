"""大气模块 — 统一接口。

物理引擎只通过 Atmosphere.get_state(position, time) 取大气数据,
保证以后可整体替换为 ISA 分层风/湍流模型(阶段5)乃至 ERA5 再分析数据,
而不改动力学代码。

阶段1实现:ConstantAtmosphere(常密度 + 可选常值风),接口签名与最终版一致。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import AtmosphereConfig


@dataclass
class AtmosphereState:
    """某一点、某一时刻的大气状态(SI)。"""
    rho_kg_m3: float                 # 空气密度
    temperature_K: float
    pressure_Pa: float
    wind_ned_m_s: np.ndarray         # 风矢量(世界系 NED,含湍流分量)


class Atmosphere:
    """大气模型基类/接口。"""

    def get_state(self, position_ned_m: np.ndarray, time_s: float) -> AtmosphereState:
        raise NotImplementedError

    def set_uniform_wind(self, wind_ned_m_s) -> None:
        """运行时修改常值风(抗风场景/调参面板用)。子类按需覆盖。"""
        raise NotImplementedError


class ConstantAtmosphere(Atmosphere):
    """常密度、常温压、常值风。适用于室内场景与单元测试。

    [假设] 室内空气密度均匀恒定 / 室内 0~5m 高差密度变化 <0.06%,可忽略
    / 户外或高空场景失效,届时用 ISA 模型(阶段5)。
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


def make_atmosphere(cfg: AtmosphereConfig) -> Atmosphere:
    """根据配置构建大气模型。"""
    if cfg.model == "constant":
        return ConstantAtmosphere(cfg)
    raise ValueError(f"unknown atmosphere model: {cfg.model}")
