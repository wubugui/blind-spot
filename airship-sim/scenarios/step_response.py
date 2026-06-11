"""验证场景 2:阶跃响应 — 前进 2m、原地转 90°、爬升 1m。

各通道独立测试(其余通道保持初值),输出响应曲线与时域指标
(上升时间 10%→90%、超调量、±5% 整定时间、稳态误差)。

运行: python scenarios/step_response.py
"""
import numpy as np

from _common import plot_timeseries, print_metric, save_fig
from airship_sim.config import default_config
from airship_sim.simulation import Simulation

import matplotlib.pyplot as plt


def step_metrics(t: np.ndarray, y: np.ndarray, y0: float, y_sp: float,
                 settle_band: float = 0.05) -> dict:
    """阶跃指标。y_sp 为目标值,y0 为初值。"""
    span = y_sp - y0
    yn = (y - y0) / span                      # 归一化响应
    out = {}
    idx10 = np.argmax(yn >= 0.1) if np.any(yn >= 0.1) else None
    idx90 = np.argmax(yn >= 0.9) if np.any(yn >= 0.9) else None
    out["rise_time_s"] = (t[idx90] - t[idx10]) if (idx10 is not None and idx90 is not None
                                                   and idx90 > 0 and idx10 > 0) else float("nan")
    out["overshoot_pct"] = max(0.0, (np.max(yn) - 1.0) * 100.0)
    outside = np.abs(yn - 1.0) > settle_band
    out["settle_time_s"] = t[np.max(np.nonzero(outside))] if np.any(outside) else 0.0
    out["steady_err"] = abs(y[-1] - y_sp)
    return out


def make_sim(alt0_m: float = 1.5) -> Simulation:
    sim = Simulation(default_config(heaviness_kg=0.005))
    sim.reset(p_ned_m=(0.0, 0.0, -alt0_m))
    sim.controller.setpoints.altitude_m = alt0_m
    return sim


def run_forward_2m(duration_s: float = 60.0):
    """前进 2m:位置保持外环把 2m 北向目标转换为航向+前速设定。"""
    sim = make_sim()
    sp = sim.controller.setpoints
    sp.pos_hold = True
    sp.target_n_m, sp.target_e_m = 2.0, 0.0
    h = sim.run(duration_s).asarray()
    m = step_metrics(h["t_s"], h["pos_ned_m"][:, 0], 0.0, 2.0)
    print("== 阶跃:前进 2m ==")
    for k, v in m.items():
        print_metric(k, f"{v:.2f}")
    fig = plot_timeseries(h, "Step: forward 2 m")
    axN = fig.axes[2]
    axN.axhline(2.0, color="k", ls="--", lw=0.8)
    save_fig(fig, "step_forward.png")
    return h, m


def run_yaw_90deg(duration_s: float = 40.0):
    sim = make_sim()
    sim.controller.setpoints.yaw_rad = np.pi / 2
    h = sim.run(duration_s).asarray()
    yaw_deg = np.rad2deg(h["euler_rad"][:, 2])
    m = step_metrics(h["t_s"], yaw_deg, 0.0, 90.0)
    print("== 阶跃:原地转 90° ==")
    for k, v in m.items():
        print_metric(k, f"{v:.2f}")
    fig = plot_timeseries(h, "Step: yaw 90 deg")
    save_fig(fig, "step_yaw.png")
    return h, m


def run_climb_1m(duration_s: float = 120.0):
    """爬升 1m。注意:5g 配重下向上净推力余量仅 ~0.001N,
    预期爬升极慢(极限爬升率 ~0.04 m/s)——这是执行器选型结论,不是缺陷。"""
    sim = make_sim(alt0_m=1.0)
    sim.controller.setpoints.altitude_m = 2.0
    h = sim.run(duration_s).asarray()
    alt = -h["pos_ned_m"][:, 2]
    m = step_metrics(h["t_s"], alt, 1.0, 2.0)
    climb_rate = np.gradient(alt, h["t_s"])
    print("== 阶跃:爬升 1m ==")
    for k, v in m.items():
        print_metric(k, f"{v:.2f}")
    print_metric("最大爬升率", f"{np.max(climb_rate):.3f}", "m/s")
    fig = plot_timeseries(h, "Step: climb 1 m", alt_target_m=2.0)
    save_fig(fig, "step_climb.png")
    return h, m


if __name__ == "__main__":
    run_forward_2m()
    run_yaw_90deg()
    run_climb_1m()
