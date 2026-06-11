"""场景脚本公共工具:路径引导、绘图、指标计算。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")


def save_fig(fig, name: str) -> str:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.abspath(os.path.join(OUTPUT_DIR, name))
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[plot] {path}")
    return path


def plot_timeseries(h: dict, title: str, alt_target_m=None):
    """标准遥测图:高度/姿态/水平轨迹/电机推力。h = History.asarray()。"""
    t = h["t_s"]
    pos = h["pos_ned_m"]
    eul = np.rad2deg(h["euler_rad"])
    thrust = h["thrust_N"]
    cmd = h["cmd_N"]

    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    ax = axes[0]
    ax.plot(t, -pos[:, 2], label="altitude (true)", lw=1.2)
    ax.plot(t, h["alt_meas_m"], label="baro meas", lw=0.5, alpha=0.5)
    if alt_target_m is not None:
        ax.axhline(alt_target_m, color="k", ls="--", lw=0.8, label="target")
    ax.set_ylabel("altitude [m]")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    for i, name in enumerate(["roll", "pitch", "yaw"]):
        ax.plot(t, eul[:, i], label=name, lw=1.0)
    sp = h["sp"]
    ax.plot(t, np.rad2deg(sp[:, 1]), "k--", lw=0.8, label="yaw sp")
    ax.set_ylabel("attitude [deg]")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot(t, pos[:, 0], label="north", lw=1.0)
    ax.plot(t, pos[:, 1], label="east", lw=1.0)
    ax.set_ylabel("position [m]")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[3]
    for i, name in enumerate(["left", "right", "vertical"]):
        ax.plot(t, thrust[:, i], label=f"{name} (actual)", lw=1.0)
        ax.plot(t, cmd[:, i], lw=0.5, alpha=0.4)
    ax.axhline(0.05, color="r", ls=":", lw=0.8, label="±limit")
    ax.axhline(-0.05, color="r", ls=":", lw=0.8)
    ax.set_ylabel("thrust [N]")
    ax.set_xlabel("t [s]")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle(title)
    return fig


def saturation_fraction(thrust: np.ndarray, limit_N: float = 0.05, frac: float = 0.98) -> np.ndarray:
    """各电机推力处于饱和(≥98% 上限)的时间占比。"""
    return np.mean(np.abs(thrust) >= frac * limit_N, axis=0)


def print_metric(name: str, value, unit: str = ""):
    print(f"  {name:<42s} {value} {unit}")
