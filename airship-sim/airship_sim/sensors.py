"""传感器模型(模拟 ESP32 + IMU + BMP390):噪声 + 采样率限制。

采样调度由 simulation.py 按整数物理 tick 触发(IMU 100Hz、气压计 10Hz),
本模块只负责"给定真值 → 生成一次带噪测量"。每类传感器使用独立的派生 RNG 流,
保证逐位可复现且互不串扰。

[假设] 各传感器噪声为零均值高斯白噪声,无偏置漂移/温漂/延迟
/ 短时(分钟级)仿真中漂移项贡献小于白噪声 / 长航时或温度剧变场景需加入
  偏置随机游走与一阶延迟。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import SensorConfig
from .dynamics import IDX_OMEGA, IDX_POS, IDX_QUAT, IDX_VEL
from .math3d import quat_to_euler


@dataclass
class Measurements:
    """控制器可见的最新测量值(各自在采样时刻更新,期间保持)。"""
    altitude_m: float = 0.0          # 气压计高度(向上为正,= -z + 噪声)
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    gyro_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3))
    u_body_m_s: float = 0.0          # 前向速度估计
    t_imu_s: float = -1.0            # 最近一次更新时刻(调试/绘图用)
    t_baro_s: float = -1.0


class SensorSuite:
    def __init__(self, cfg: SensorConfig, rng_root: np.random.Generator):
        self.cfg = cfg
        # 独立子流:增删某一传感器不影响其他传感器的随机序列
        child = rng_root.spawn(3)
        self._rng_imu, self._rng_baro, self._rng_vel = child
        self.meas = Measurements()

    def sample_imu(self, x: np.ndarray, t_s: float) -> None:
        """100Hz:姿态角 + 角速度 + 前向速度估计(见 SensorConfig 的 [假设])。"""
        roll, pitch, yaw = quat_to_euler(x[IDX_QUAT])
        n = self.cfg.att_noise_std_rad
        self.meas.roll_rad = roll + self._rng_imu.normal(0.0, n)
        self.meas.pitch_rad = pitch + self._rng_imu.normal(0.0, n)
        self.meas.yaw_rad = yaw + self._rng_imu.normal(0.0, n)
        self.meas.gyro_rad_s = x[IDX_OMEGA] + self._rng_imu.normal(
            0.0, self.cfg.gyro_noise_std_rad_s, 3)
        self.meas.u_body_m_s = float(
            x[IDX_VEL][0] + self._rng_vel.normal(0.0, self.cfg.vel_noise_std_m_s))
        self.meas.t_imu_s = t_s

    def sample_baro(self, x: np.ndarray, t_s: float) -> None:
        """10Hz,σ=0.05m(2σ≈±0.1m,BMP390 量级)。"""
        alt_true_m = -x[IDX_POS][2]
        self.meas.altitude_m = alt_true_m + self._rng_baro.normal(
            0.0, self.cfg.baro_noise_std_m)
        self.meas.t_baro_s = t_s
