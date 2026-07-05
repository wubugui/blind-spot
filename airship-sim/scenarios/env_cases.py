"""极端环境 case 验证:冰川 / 高原 / 湖泊 下的定点悬停抗风。

一次跑完三类环境(或指定其一),打印每个环境的:当地密度、螺旋桨可用推力、
风场在飞行高度的风速、定点保持漂移/高度误差/电机饱和占比,并出曲线 PNG。
环境的密度、空间风场、高原配平、太阳负荷都由 environments 预设自动配好。

运行:
    python scenarios/env_cases.py                 # 三个环境全跑
    python scenarios/env_cases.py --env glacier   # 只跑冰川
    python scenarios/env_cases.py --duration 90 --alt 10
"""
import argparse

import numpy as np

from _common import plot_timeseries, print_metric, saturation_fraction, save_fig
from airship_sim import environments as envs
from airship_sim.environments import heading_into_wind_quat
from airship_sim.simulation import Simulation


def run_case(name: str, cfg, alt_m: float, duration_s: float):
    sim = Simulation(cfg)
    # 迎风起始朝向:欠驱动飞艇背风起飞、掉头期间会被吹散,故初始即指向风来向
    q = heading_into_wind_quat(sim.atmosphere, (0.0, 0.0, -alt_m), 0.0)
    sim.reset(p_ned_m=(0.0, 0.0, -alt_m), q=tuple(q))
    sp = sim.controller.setpoints
    sp.pos_hold = True
    sp.target_n_m, sp.target_e_m = 0.0, 0.0
    sp.altitude_m = alt_m

    # 环境上下文
    st = sim.atmosphere.get_state(np.array([0.0, 0.0, -alt_m]), 0.0)
    wf = sim.atmosphere.wind_field
    w = wf.wind_ned(np.array([0.0, 0.0, -alt_m]), alt_m, 0.0) if wf else np.zeros(3)
    thrust_avail = cfg.actuator.thrust_max_N * (st.rho_kg_m3 / 1.225
                   if cfg.actuator.density_scale_thrust else 1.0)

    h = sim.run(duration_s).asarray()
    t = h["t_s"]
    settle = t > duration_s / 3
    alt_true = -h["pos_ned_m"][:, 2]
    horiz = np.hypot(h["pos_ned_m"][:, 0], h["pos_ned_m"][:, 1])
    sat = saturation_fraction(h["thrust_N"], cfg.actuator.thrust_max_N)

    print(f"\n===== {name.upper()} =====")
    print_metric("海拔 / 当地密度", f"{cfg.atmosphere.ground_alt_m:.0f} m / "
                 f"{st.rho_kg_m3:.3f}", f"kg/m³ ({st.rho_kg_m3/1.225*100:.0f}% 海平面)")
    print_metric("可用螺旋桨推力", f"{thrust_avail*1000:.1f}",
                 f"g力 (海平面 {cfg.actuator.thrust_max_N*1000:.0f}g)")
    print_metric(f"飞行高度 {alt_m:.0f}m 处风", f"{np.hypot(w[0], w[1]):.2f} 水平 / "
                 f"{-w[2]:+.2f} 垂直", "m/s")
    print_metric("稳态水平漂移 均值/峰值",
                 f"{np.mean(horiz[settle]):.2f} / {np.max(horiz[settle]):.2f}", "m")
    print_metric("稳态高度误差 RMS", f"{np.sqrt(np.mean((alt_true[settle]-alt_m)**2)):.3f}", "m")
    print_metric("电机饱和占比 [左/右/垂直]", np.array2string(sat, precision=2), "")

    fig = plot_timeseries(h, f"{name} case  alt={alt_m:.0f}m  "
                          f"rho={st.rho_kg_m3:.2f}", alt_target_m=alt_m)
    save_fig(fig, f"env_{name}.png")
    return {"name": name, "drift_mean": float(np.mean(horiz[settle])),
            "drift_max": float(np.max(horiz[settle])), "rho": float(st.rho_kg_m3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", default="all", choices=["all", "glacier", "plateau", "lake"])
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--alt", type=float, default=8.0, help="飞行高度 AGL (m)")
    ap.add_argument("--airframe", default="default", choices=["default", "heavy"],
                    help="default=默认机型(抗风~0.5m/s);heavy=抗风重型(可定点)")
    args = ap.parse_args()

    builders = {"glacier": envs.glacier, "plateau": envs.plateau, "lake": envs.lake}
    names = list(builders) if args.env == "all" else [args.env]
    results = [run_case(n, builders[n](airframe=args.airframe), args.alt, args.duration)
               for n in names]

    print("\n===== 汇总 =====")
    for r in results:
        verdict = "可守住" if r["drift_max"] < 3.5 else ("勉强" if r["drift_max"] < 12 else "守不住")
        print_metric(r["name"], f"峰值漂移 {r['drift_max']:.1f} m  → {verdict}",
                     f"(ρ={r['rho']:.2f})")
    print("\n提示:改电机上限/尾翼/配重/风场参数(见 airship_sim/environments.py)重跑,"
          "对比能否把峰值漂移压进目标。")


if __name__ == "__main__":
    main()
