"""仿真主循环:1ms 物理(RK4) + 50Hz 控制 + 传感器整数倍调度。

所有调度用整数 tick 计数(无浮点时间累积误差):
  物理   每 1 tick   (dt = 1ms)
  IMU    每 10 tick  (100 Hz)
  气压计 每 100 tick (10 Hz)
  控制   每 20 tick  (50 Hz)
传感器采样发生在控制更新之前(同一 tick 时先测量后控制)。

可复现性:同一 SimConfig(含 seed)→ 逐位一致轨迹(test_determinism 验证)。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .atmosphere import Atmosphere, make_atmosphere
from .config import SimConfig
from .controller import CascadeController, MotorCommands
from .dynamics import (IDX_OMEGA, IDX_POS, IDX_QUAT, IDX_THRUST, IDX_VEL,
                       AirshipDynamics)
from .math3d import quat_to_euler
from .sensors import SensorSuite


@dataclass
class History:
    """按控制周期(50Hz)记录的时间序列,场景脚本绘图用。"""
    t_s: list = field(default_factory=list)
    pos_ned_m: list = field(default_factory=list)
    euler_rad: list = field(default_factory=list)       # roll, pitch, yaw(真值)
    v_body_m_s: list = field(default_factory=list)
    omega_rad_s: list = field(default_factory=list)
    thrust_N: list = field(default_factory=list)        # 电机实际推力 [左,右,垂直]
    cmd_N: list = field(default_factory=list)           # 电机指令
    alt_meas_m: list = field(default_factory=list)      # 气压计测量(控制器所见)
    sp: list = field(default_factory=list)              # (alt_sp, yaw_sp, speed_sp)
    wind_ned_m_s: list = field(default_factory=list)

    def asarray(self) -> dict[str, np.ndarray]:
        return {k: np.array(v) for k, v in self.__dict__.items()}


class Simulation:
    def __init__(self, cfg: SimConfig, atmosphere: Atmosphere | None = None,
                 terrain=None):
        self.cfg = cfg
        # 大气湍流种子由全局种子派生(+1 偏移,与传感器流区分)
        self.atmosphere = atmosphere if atmosphere is not None else make_atmosphere(
            cfg.atmosphere, seed=cfg.seed + 1)
        self.dynamics = AirshipDynamics(cfg, self.atmosphere, terrain=terrain)
        # 可选能量模型(cfg.energy.enabled=False 时为 None,现有路径零改动)
        if cfg.energy.enabled:
            from .energy import EnergyModel
            self.energy = EnergyModel(cfg.energy)
        else:
            self.energy = None
        rng_root = np.random.default_rng(cfg.seed)
        self.sensors = SensorSuite(cfg.sensor, rng_root)
        self.controller = CascadeController(
            cfg.control, cfg.actuator.motor_sep_y_m, cfg.actuator.thrust_max_N)

        dt = cfg.dt_physics_s
        self._ticks_per_imu = max(1, round(1.0 / (cfg.sensor.imu_rate_hz * dt)))
        self._ticks_per_baro = max(1, round(1.0 / (cfg.sensor.baro_rate_hz * dt)))
        self._ticks_per_ctrl = max(1, round(1.0 / (cfg.control.rate_hz * dt)))
        self._dt_ctrl_s = self._ticks_per_ctrl * dt

        self.reset()

    def reset(self, p_ned_m=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0)) -> None:
        self.x = self.dynamics.initial_state(p_ned_m, q)
        self.tick = 0
        self.cmd = MotorCommands()
        self.controller.reset()
        self.history = History()
        self.atmosphere.reset()
        if self.energy is not None:
            self.energy.reset()
        # 地面接触/撞击记录(上层判定着陆与坠毁)
        self.contact = {"contact": False, "pen_m": 0.0,
                        "v_down_m_s": 0.0, "v_horiz_m_s": 0.0, "water": False}
        self.last_impact: dict | None = None
        self._prev_v_down = 0.0
        self._prev_v_horiz = 0.0
        self._init_helium(p_ned_m)
        # 初始测量 = 初始真值(控制器第一拍前已有有效测量)
        self.sensors.sample_imu(self.x, 0.0)
        self.sensors.sample_baro(self.x, 0.0)
        # 初始气压计测量去噪:reset 时刻用真值,避免首拍大误差(只影响第一拍前)
        self.sensors.meas.altitude_m = -self.x[IDX_POS][2]

    @property
    def t_s(self) -> float:
        return self.tick * self.cfg.dt_physics_s

    # ---- 氦气热力学(见 HeliumThermalConfig 注释) ----
    def _init_helium(self, p_ned_m) -> None:
        hcfg = self.cfg.helium
        self.helium_T_K: float | None = None
        self.helium_m_kg = self.cfg.mass.rho_helium_kg_m3 * self.cfg.hull.volume_m3
        if not hcfg.enabled:
            return
        atm = self.atmosphere.get_state(np.array(p_ned_m, dtype=float), 0.0)
        self.helium_T_K = atm.temperature_K
        # 等效气体常数:与配置密度在初始环境下自洽([假设] 见 HeliumThermalConfig)
        self._R_eff_J_kgK = atm.pressure_Pa / (
            self.cfg.mass.rho_helium_kg_m3 * atm.temperature_K)
        hull = self.cfg.hull
        a, b = hull.a_m, hull.b_m
        e = np.sqrt(1.0 - (b / a) ** 2)
        # 长椭球表面积(解析):A = 2πb²(1 + a/(b·e)·arcsin(e)) ≈ 4.19 m²
        self._A_surf_m2 = 2.0 * np.pi * b**2 * (1.0 + a / (b * e) * np.arcsin(e))
        self._A_proj_m2 = hull.side_area_m2   # 太阳辐照投影面积(侧照,保守取大面)

    def _step_helium(self, t: float) -> None:
        hcfg = self.cfg.helium
        if not hcfg.enabled or self.helium_T_K is None:
            return
        atm = self.atmosphere.get_state(self.x[IDX_POS], t)
        q_solar_W = (hcfg.absorptivity * hcfg.solar_flux_W_m2 * self._A_proj_m2
                     if hcfg.solar_on else 0.0)
        q_conv_W = hcfg.h_film_W_m2K * self._A_surf_m2 * (self.helium_T_K - atm.temperature_K)
        dT_dt = (q_solar_W - q_conv_W) / (self.helium_m_kg * hcfg.cp_J_kgK)
        self.helium_T_K += dT_dt * self._dt_ctrl_s
        # 定容排气:囊内压力 = 环境压力,质量随温度/高度变化
        self.helium_m_kg = atm.pressure_Pa / (self._R_eff_J_kgK * self.helium_T_K) \
            * self.cfg.hull.volume_m3
        self.dynamics.set_helium_mass(self.helium_m_kg)

    def step(self, record: bool = True) -> None:
        """推进一个物理 tick(1ms)。传感器/控制按整数倍 tick 触发。"""
        t = self.t_s
        if self.tick % self._ticks_per_imu == 0:
            self.sensors.sample_imu(self.x, t)
        if self.tick % self._ticks_per_baro == 0:
            self.sensors.sample_baro(self.x, t)
        if self.tick % self._ticks_per_ctrl == 0:
            self._step_helium(t)
            self.cmd = self.controller.update(
                self.sensors.meas, self.x[IDX_POS], self._dt_ctrl_s)
            self._step_energy(t)
            self._step_contact()
            if record:
                self._record(t)
        self.x = self.dynamics.step_rk4(self.x, self.cmd.as_array(), t, self.cfg.dt_physics_s)
        self.tick += 1

    def _step_energy(self, t: float) -> None:
        if self.energy is None:
            return
        atm = self.atmosphere.get_state(self.x[IDX_POS], t)
        hcfg = self.cfg.helium
        self.energy.step(self.x[IDX_THRUST], self.x[IDX_VEL][0], self.x[IDX_VEL][2],
                         atm.rho_kg_m3, hcfg.solar_on, hcfg.solar_flux_W_m2,
                         self._dt_ctrl_s)
        if self.energy.depleted:
            self.cmd = MotorCommands()   # 电量耗尽:电机停转(浮空不受影响)

    def _step_contact(self) -> None:
        """接触探针 + 首次触地撞击速度记录(50Hz;地面模块未挂时零开销)。"""
        gnd = self.dynamics.ground
        if gnd is None:
            return
        from .math3d import quat_normalize, quat_to_rotmat
        R = quat_to_rotmat(quat_normalize(self.x[IDX_QUAT]))
        nu = np.concatenate([self.x[IDX_VEL], self.x[IDX_OMEGA]])
        was = self.contact["contact"]
        self.contact = gnd.probe(self.x[IDX_POS], R, nu)
        if self.contact["contact"] and not was:
            # 撞击速度取触地前一拍的速度(触地后弹簧立即减速,当前值偏小)
            self.last_impact = {"v_down_m_s": self._prev_v_down,
                                "v_horiz_m_s": self._prev_v_horiz,
                                "water": self.contact["water"], "t_s": self.t_s}
        v_w = R @ self.x[IDX_VEL]
        self._prev_v_down = float(v_w[2])
        self._prev_v_horiz = float(np.hypot(v_w[0], v_w[1]))

    def run(self, duration_s: float, record: bool = True) -> History:
        n = round(duration_s / self.cfg.dt_physics_s)
        for _ in range(n):
            self.step(record=record)
        return self.history

    def _record(self, t: float) -> None:
        h = self.history
        h.t_s.append(t)
        h.pos_ned_m.append(self.x[IDX_POS].copy())
        h.euler_rad.append(quat_to_euler(self.x[IDX_QUAT]))
        h.v_body_m_s.append(self.x[IDX_VEL].copy())
        h.omega_rad_s.append(self.x[IDX_OMEGA].copy())
        h.thrust_N.append(self.x[IDX_THRUST].copy())
        h.cmd_N.append(self.cmd.as_array())
        h.alt_meas_m.append(self.sensors.meas.altitude_m)
        d = self.controller.debug
        h.sp.append((d.alt_sp_m, d.yaw_sp_rad, d.speed_sp_m_s))
        atm = self.atmosphere.get_state(self.x[IDX_POS], t)
        h.wind_ned_m_s.append(atm.wind_ned_m_s.copy())
