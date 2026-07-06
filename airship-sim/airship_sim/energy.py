"""能量模型(可选模块):电池 + 螺旋桨功率 + 太阳能。模型假设见 EnergyConfig。

由 simulation.py 按控制周期(50Hz)推进;cfg.energy.enabled=False 时不构造,
现有路径零改动。电量耗尽后 Simulation 将电机指令置零(航电负载仍计)。
"""
from __future__ import annotations

import math

import numpy as np

from .config import EnergyConfig


class EnergyModel:
    def __init__(self, cfg: EnergyConfig):
        self.cfg = cfg
        self.battery_Wh = cfg.capacity_Wh
        self.last_power_W = 0.0

    def reset(self) -> None:
        self.battery_Wh = self.cfg.capacity_Wh
        self.last_power_W = 0.0

    @property
    def depleted(self) -> bool:
        return self.battery_Wh <= 0.0

    @property
    def fraction(self) -> float:
        return max(0.0, self.battery_Wh / self.cfg.capacity_Wh)

    def prop_power_W(self, thrust_N: float, airspeed_axial_m_s: float,
                     rho_kg_m3: float) -> float:
        """单桨功率:P = |T|·(|u| + v_i)/η,v_i = sqrt(|T|/(2ρA))。"""
        t = abs(float(thrust_N))
        if t <= 0.0:
            return 0.0
        v_i = math.sqrt(t / (2.0 * rho_kg_m3 * self.cfg.prop_disc_area_m2))
        return t * (abs(float(airspeed_axial_m_s)) + v_i) / self.cfg.prop_efficiency

    def step(self, thrust_N: np.ndarray, u_axial_m_s: float, w_axial_m_s: float,
             rho_kg_m3: float, solar_on: bool, solar_flux_W_m2: float,
             dt_s: float) -> None:
        """推进一步。thrust_N = [左, 右, 垂直] 实际推力;垂直桨轴向速度取 w。"""
        p = self.cfg.avionics_W
        p += self.prop_power_W(thrust_N[0], u_axial_m_s, rho_kg_m3)
        p += self.prop_power_W(thrust_N[1], u_axial_m_s, rho_kg_m3)
        p += self.prop_power_W(thrust_N[2], w_axial_m_s, rho_kg_m3)
        charge = 0.0
        if solar_on and self.cfg.solar_area_m2 > 0.0:
            charge = self.cfg.solar_area_m2 * solar_flux_W_m2 * self.cfg.solar_efficiency
        self.last_power_W = p - charge
        self.battery_Wh -= self.last_power_W * dt_s / 3600.0
        self.battery_Wh = min(self.battery_Wh, self.cfg.capacity_Wh)
        if self.battery_Wh < 0.0:
            self.battery_Wh = 0.0
