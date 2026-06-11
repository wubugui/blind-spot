"""串级 PID 控制器(50Hz,与物理步长分离)。

结构:
  外环(可选,位置保持):水平位置误差 → 航向设定 + 前向速度设定
  高度通道: 气压计高度 → PID → 垂直推力 [N]
  偏航通道: 航向角(含角度卷绕) → PID → 偏航力矩 [N·m] → 左右差速
  前速通道: 前向速度 → PI → 总前向推力 [N]

混控(电机几何见 dynamics:右推产生负偏航力矩):
  τ_z = d·(F_L - F_R)  →  F_L = F_fwd/2 + τ_cmd/(2d),F_R = F_fwd/2 - τ_cmd/(2d)

PID 实现要点:
- 微分项作用在测量值上(derivative-on-measurement)并带一阶低通,避免设定值阶跃踢
- 抗积分饱和:输出钳位 + 条件积分(输出饱和且误差同向时停止积分)
- 高度测量 10Hz 且噪声大,微分低通时间常数取 1s(见 ControlConfig)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import ControlConfig, PidGains
from .math3d import wrap_angle_rad
from .sensors import Measurements


class Pid:
    def __init__(self, g: PidGains):
        self.g = g
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._d_filt = 0.0
        self._prev_meas: float | None = None
        self._meas_filt: float | None = None

    def update(self, setpoint: float, meas_raw: float, dt_s: float,
               error_override: float | None = None, ff: float = 0.0) -> float:
        """error_override 用于角度通道传入卷绕后的误差(此时跳过测量滤波,由调用方处理);
        ff 为前馈项(参与饱和/抗积分判断)。"""
        g = self.g
        # 测量低通(见 PidGains.tau_meas_s 注释)
        if g.tau_meas_s > 0:
            if self._meas_filt is None:
                self._meas_filt = meas_raw
            a_m = dt_s / (g.tau_meas_s + dt_s)
            self._meas_filt += a_m * (meas_raw - self._meas_filt)
            meas = self._meas_filt
        else:
            meas = meas_raw
        err = error_override if error_override is not None else (setpoint - meas)

        # 微分(对测量,一阶低通)
        if self._prev_meas is None:
            d_meas = 0.0
        else:
            d_meas = (meas - self._prev_meas) / dt_s
        self._prev_meas = meas
        alpha = dt_s / (g.tau_d_s + dt_s) if g.tau_d_s > 0 else 1.0
        self._d_filt += alpha * (d_meas - self._d_filt)

        out_unsat = ff + g.kp * err + self._integral - g.kd * self._d_filt
        out = float(np.clip(out_unsat, g.out_min, g.out_max))
        # 条件积分抗饱和
        saturated = out != out_unsat
        if not (saturated and (out_unsat - out) * err > 0):
            self._integral += g.ki * err * dt_s
            self._integral = float(np.clip(self._integral, g.out_min, g.out_max))
        return out

    @property
    def integral(self) -> float:
        return self._integral


@dataclass
class Setpoints:
    altitude_m: float = 0.0
    yaw_rad: float = 0.0
    speed_m_s: float = 0.0
    # 位置保持模式:目标水平位置(NED 北/东),启用后覆盖 yaw/speed 设定
    pos_hold: bool = False
    target_n_m: float = 0.0
    target_e_m: float = 0.0


@dataclass
class MotorCommands:
    """电机推力指令 [N],顺序与 dynamics 状态一致:左、右、垂直。"""
    left_N: float = 0.0
    right_N: float = 0.0
    vertical_N: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.left_N, self.right_N, self.vertical_N])


@dataclass
class ChannelDebug:
    """遥测用:各通道 设定值/测量值/输出。"""
    alt_sp_m: float = 0.0
    alt_meas_m: float = 0.0
    alt_out_N: float = 0.0
    yaw_sp_rad: float = 0.0
    yaw_meas_rad: float = 0.0
    yaw_out_Nm: float = 0.0
    speed_sp_m_s: float = 0.0
    speed_meas_m_s: float = 0.0
    speed_out_N: float = 0.0


class CascadeController:
    def __init__(self, cfg: ControlConfig, motor_sep_y_m: float, thrust_max_N: float):
        self.cfg = cfg
        self._d_y_m = motor_sep_y_m
        self._thrust_max_N = thrust_max_N
        self.pid_alt = Pid(cfg.alt)
        self.pid_yaw = Pid(cfg.yaw)
        self.pid_speed = Pid(cfg.speed)
        self.setpoints = Setpoints()
        self.debug = ChannelDebug()
        self.manual_mode = False
        self.manual_cmd = MotorCommands()

    def reset(self) -> None:
        self.pid_alt.reset()
        self.pid_yaw.reset()
        self.pid_speed.reset()

    def set_gains(self, channel: str, gains: PidGains) -> None:
        """实时改增益(调参面板):保留积分状态,只换系数。"""
        pid = {"alt": self.pid_alt, "yaw": self.pid_yaw, "speed": self.pid_speed}[channel]
        pid.g = gains

    def update(self, meas: Measurements, pos_ned_m: np.ndarray, dt_s: float) -> MotorCommands:
        """pos_ned_m 仅位置保持外环使用([假设] 水平位置由外部定位系统提供,
        无噪声 / 外环带宽低(<0.05Hz),定位噪声影响小 / 纯惯性导航时不可用)。"""
        if self.manual_mode:
            return self.manual_cmd

        sp = self.setpoints
        yaw_sp = sp.yaw_rad
        speed_sp = sp.speed_m_s
        if sp.pos_hold:
            dn = sp.target_n_m - pos_ned_m[0]
            de = sp.target_e_m - pos_ned_m[1]
            dist = float(np.hypot(dn, de))
            if dist > self.cfg.pos_hold_deadband_m:
                yaw_sp = float(np.arctan2(de, dn))   # 指向目标
                speed_sp = float(np.clip(self.cfg.pos_hold_kp_1_s * dist,
                                         0.0, self.cfg.pos_hold_max_speed_m_s))
            else:
                speed_sp = 0.0
                yaw_sp = meas.yaw_rad  # 死区内保持当前航向

        # 高度(PID + 悬停油门前馈,前馈在 PID 内部参与饱和与抗积分判断)
        f_vert = self.pid_alt.update(sp.altitude_m, meas.altitude_m, dt_s,
                                     ff=self.cfg.alt_ff_N)
        # 偏航(角度卷绕)
        yaw_err = wrap_angle_rad(yaw_sp - meas.yaw_rad)
        tau_z = self.pid_yaw.update(yaw_sp, meas.yaw_rad, dt_s, error_override=yaw_err)
        # 前速
        f_fwd = self.pid_speed.update(speed_sp, meas.u_body_m_s, dt_s)

        f_left = f_fwd / 2.0 + tau_z / (2.0 * self._d_y_m)
        f_right = f_fwd / 2.0 - tau_z / (2.0 * self._d_y_m)
        m = self._thrust_max_N
        cmd = MotorCommands(
            left_N=float(np.clip(f_left, -m, m)),
            right_N=float(np.clip(f_right, -m, m)),
            vertical_N=float(np.clip(f_vert, -m, m)),
        )

        d = self.debug
        d.alt_sp_m, d.alt_meas_m, d.alt_out_N = sp.altitude_m, meas.altitude_m, cmd.vertical_N
        d.yaw_sp_rad, d.yaw_meas_rad, d.yaw_out_Nm = yaw_sp, meas.yaw_rad, tau_z
        d.speed_sp_m_s, d.speed_meas_m_s, d.speed_out_N = speed_sp, meas.u_body_m_s, f_fwd
        return cmd
