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
    def __init__(self, cfg: SimConfig, atmosphere: Atmosphere | None = None):
        self.cfg = cfg
        self.atmosphere = atmosphere if atmosphere is not None else make_atmosphere(cfg.atmosphere)
        self.dynamics = AirshipDynamics(cfg, self.atmosphere)
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
        # 初始测量 = 初始真值(控制器第一拍前已有有效测量)
        self.sensors.sample_imu(self.x, 0.0)
        self.sensors.sample_baro(self.x, 0.0)
        # 初始气压计测量去噪:reset 时刻用真值,避免首拍大误差(只影响第一拍前)
        self.sensors.meas.altitude_m = -self.x[IDX_POS][2]

    @property
    def t_s(self) -> float:
        return self.tick * self.cfg.dt_physics_s

    def step(self, record: bool = True) -> None:
        """推进一个物理 tick(1ms)。传感器/控制按整数倍 tick 触发。"""
        t = self.t_s
        if self.tick % self._ticks_per_imu == 0:
            self.sensors.sample_imu(self.x, t)
        if self.tick % self._ticks_per_baro == 0:
            self.sensors.sample_baro(self.x, t)
        if self.tick % self._ticks_per_ctrl == 0:
            self.cmd = self.controller.update(
                self.sensors.meas, self.x[IDX_POS], self._dt_ctrl_s)
            if record:
                self._record(t)
        self.x = self.dynamics.step_rk4(self.x, self.cmd.as_array(), t, self.cfg.dt_physics_s)
        self.tick += 1

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
