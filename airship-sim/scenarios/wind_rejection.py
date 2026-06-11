"""验证场景 3:常值风下悬停 — 0.5 m/s 与 1.5 m/s,观察控制饱和。

飞艇唯一的抗水平风手段是机头迎风 + 前向推力。
最大前向推力 0.1N 对应的可抗风速(轴向阻力平衡):
    0.1 = ½·ρ·Cd_ax·A_cross·v²  →  v ≈ 1.9 m/s(理论上限,姿态/侧向扰动会再打折扣)
侧向完全无执行器,横风分量只能靠转向迎风消化。

运行: python scenarios/wind_rejection.py [--speeds 0.5 1.5]
"""
import argparse

import numpy as np

from _common import plot_timeseries, print_metric, saturation_fraction, save_fig
from airship_sim.config import default_config
from airship_sim.simulation import Simulation


def run_wind(wind_speed_m_s: float, duration_s: float = 90.0):
    cfg = default_config(heaviness_kg=0.005)
    # 风从北方吹来(指向南,-N 方向),飞艇需机头朝北顶风
    cfg.atmosphere.wind_ned_m_s = (-wind_speed_m_s, 0.0, 0.0)
    sim = Simulation(cfg)
    alt_target_m = 1.5
    sim.reset(p_ned_m=(0.0, 0.0, -alt_target_m))
    sp = sim.controller.setpoints
    sp.altitude_m = alt_target_m
    sp.pos_hold = True
    sp.target_n_m, sp.target_e_m = 0.0, 0.0

    h = sim.run(duration_s).asarray()
    t = h["t_s"]
    settle = t > duration_s / 2
    horiz_dist = np.hypot(h["pos_ned_m"][:, 0], h["pos_ned_m"][:, 1])
    alt_err = -h["pos_ned_m"][:, 2] - alt_target_m
    sat = saturation_fraction(h["thrust_N"], cfg.actuator.thrust_max_N)
    fwd_thrust = h["thrust_N"][:, 0] + h["thrust_N"][:, 1]

    print(f"== 抗风悬停:{wind_speed_m_s} m/s 常值风,{duration_s:.0f}s ==")
    print_metric("稳态水平偏离 均值/最大", f"{np.mean(horiz_dist[settle]):.2f} / "
                                    f"{np.max(horiz_dist[settle]):.2f}", "m")
    print_metric("稳态高度误差 RMS", f"{np.sqrt(np.mean(alt_err[settle]**2)):.3f}", "m")
    print_metric("前向总推力 均值(稳态)", f"{np.mean(fwd_thrust[settle]):.4f}", "N (上限 0.1)")
    print_metric("电机饱和占比 [左/右/垂直]", np.array2string(sat, precision=3), "")
    mean_d, max_d = np.mean(horiz_dist[settle]), np.max(horiz_dist[settle])
    if mean_d < 1.0 and max_d < 5.0:
        verdict = "保持(均值<1m,有界)"
    elif max_d < 5.0:
        verdict = "有界轨道(未发散,精度差)"
    else:
        verdict = "发散(偏航差速饱和,无法顶风)"
    print_metric("结论", verdict, "")

    fig = plot_timeseries(h, f"Hover in {wind_speed_m_s} m/s headwind",
                          alt_target_m=alt_target_m)
    save_fig(fig, f"wind_{str(wind_speed_m_s).replace('.', 'p')}.png")
    return h


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.5, 1.5])
    args = ap.parse_args()
    for v in args.speeds:
        run_wind(v)
