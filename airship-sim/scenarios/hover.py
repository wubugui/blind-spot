"""验证场景 1:无风定点悬停 60s。

考察:高度保持精度(气压计噪声 ±0.1m、10Hz 下)、姿态稳定性、
垂直电机出力水平(5g 配重 → 稳态需 0.049N,接近 0.05N 上限,关注饱和占比)。

运行: python scenarios/hover.py [--duration 60]
"""
import argparse

import numpy as np

from _common import plot_timeseries, print_metric, saturation_fraction, save_fig
from airship_sim.config import default_config
from airship_sim.simulation import Simulation


def run(duration_s: float = 60.0, seed: int | None = None):
    cfg = default_config(heaviness_kg=0.005)
    if seed is not None:
        cfg.seed = seed
    sim = Simulation(cfg)
    alt_target_m = 1.5
    sim.reset(p_ned_m=(0.0, 0.0, -alt_target_m))
    sp = sim.controller.setpoints
    sp.altitude_m = alt_target_m
    sp.pos_hold = True
    sp.target_n_m, sp.target_e_m = 0.0, 0.0

    h = sim.run(duration_s).asarray()

    t = h["t_s"]
    settle_mask = t > duration_s / 3  # 后 2/3 视为稳态
    alt_true = -h["pos_ned_m"][:, 2]
    alt_err = alt_true - alt_target_m
    eul_deg = np.rad2deg(h["euler_rad"])
    horiz_dist = np.hypot(h["pos_ned_m"][:, 0], h["pos_ned_m"][:, 1])
    sat = saturation_fraction(h["thrust_N"], cfg.actuator.thrust_max_N)

    print(f"== 悬停保持 {duration_s:.0f}s(无风,5g 偏重)==")
    print_metric("稳态高度误差 RMS", f"{np.sqrt(np.mean(alt_err[settle_mask]**2)):.3f}", "m")
    print_metric("稳态高度误差 max|·|", f"{np.max(np.abs(alt_err[settle_mask])):.3f}", "m")
    print_metric("稳态水平漂移 max", f"{np.max(horiz_dist[settle_mask]):.3f}", "m")
    print_metric("稳态 roll/pitch RMS",
                 f"{np.sqrt(np.mean(eul_deg[settle_mask, 0]**2)):.2f} / "
                 f"{np.sqrt(np.mean(eul_deg[settle_mask, 1]**2)):.2f}", "deg")
    print_metric("垂直电机平均推力(稳态)",
                 f"{np.mean(h['thrust_N'][settle_mask, 2]):.4f}", "N (上限 0.05)")
    print_metric("电机饱和时间占比 [左/右/垂直]",
                 np.array2string(sat, precision=3), "")

    fig = plot_timeseries(h, f"Hover hold {duration_s:.0f}s (no wind, 5g heavy)",
                          alt_target_m=alt_target_m)
    save_fig(fig, "hover.png")
    return h


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    run(args.duration, args.seed)
