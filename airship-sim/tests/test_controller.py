"""控制器回归测试:偏航微分项的 ±π 卷绕处理。

回归针对的 bug:偏航通道的微分(derivative-on-measurement)原先对原始航向角求
差分,航向穿越 ±180° 时原始角度跳变 ~2π,被微分放大成假角速度,把偏航舵反向
打满约 τ_d/dt 个控制周期。修复:角度通道按最短弧(wrap_angle_rad)算变化率。
"""
import numpy as np

from airship_sim.config import default_config
from airship_sim.controller import Pid
from airship_sim.math3d import wrap_angle_rad


def _yaw_torque_sweep(meas_deg_seq, setpoint_deg):
    """用真实偏航增益与 50Hz 步长,回放一串航向测量,返回每步偏航力矩。"""
    cfg = default_config()
    pid = Pid(cfg.control.yaw)
    sp = np.deg2rad(setpoint_deg)
    out = []
    for md in meas_deg_seq:
        m = np.deg2rad(md)
        err = wrap_angle_rad(sp - m)
        out.append(pid.update(sp, m, 0.02, error_override=err))
    return out


def test_yaw_derivative_no_kick_across_pi_wrap():
    """匀速转向穿越 ±180° 时,偏航力矩不得出现方向相反的假脉冲。

    物理上是一次平滑的同向转动(每步约 +1.5°),因此力矩符号应保持一致;
    修复前在卷绕步会翻号并反向饱和。
    """
    # 航向 176->180->-171,朝设定值 -170(短弧,持续同向 +)平滑推进
    seq = [176, 177.5, 179, 179.5, -179.5, -178, -176.5, -175, -173, -171]
    torques = _yaw_torque_sweep(seq, setpoint_deg=-170)
    # 跳过第一步(prev_meas 尚未建立,纯 P 项);其余步力矩符号必须一致
    steady = torques[1:]
    signs = {np.sign(round(t, 9)) for t in steady if abs(t) > 1e-9}
    assert signs <= {-1.0}, f"偏航力矩在卷绕处翻号(出现方向相反脉冲): {torques}"

    # 与远离卷绕的等价转向对比:峰值力矩量级应一致(卷绕不应制造额外饱和)
    seq_flat = [6, 7.5, 9, 9.5, 10.5, 12, 13.5, 15, 17, 19]
    torques_flat = _yaw_torque_sweep(seq_flat, setpoint_deg=20)
    assert abs(max(map(abs, torques[1:])) - max(map(abs, torques_flat[1:]))) < 1e-9


def test_yaw_derivative_matches_gyro_rate_sign():
    """卷绕步算出的等效微分速率,应与真实短弧角速度同号、量级相当。"""
    cfg = default_config()
    pid = Pid(cfg.control.yaw)
    sp = np.deg2rad(-170)
    # 先建立 prev_meas
    pid.update(sp, np.deg2rad(179.5), 0.02, error_override=wrap_angle_rad(sp - np.deg2rad(179.5)))
    d_before = pid._d_filt
    # 穿越到 -179.5(真实变化 +1°,而非 -359°)
    pid.update(sp, np.deg2rad(-179.5), 0.02, error_override=wrap_angle_rad(sp - np.deg2rad(-179.5)))
    d_after = pid._d_filt
    # 真实短弧速率 = +1° / 0.02s ≈ +0.87 rad/s;滤波后应是小正量,而非 ~ -300 的假值
    assert abs(d_after) < 5.0, f"卷绕步微分被污染成假角速度: d_filt={d_after}"
