"""世界完备性验证:大陆上每条航线都必须能用"该阶段合理的船"飞完。

对每条航线(爬升方向 + 关键回程),用按海拔选择的船型自动驾驶飞完:
巡航 + 地形保持辅助 + 简单的压舱/放氦策略(与玩家可用手段一致)。
输出每段 PASS/FAIL;任何 FAIL = 世界数据或物理参数需要修。

用法: PYTHONPATH=airship-sim:airship-sim/game python3 game/verify_world.py [--slot N]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import game_core as gc

SHIPS = {
    "base": {"envelope": {"len_m": 25, "dia_m": 8, "skin_kg": 128},
             "fins": {"s_m2": 30, "mass_kg": 55},
             "motor": {"t_max_N": 300, "mass_kg": 110, "disc_m2": 4.5},
             "battery": {"wh": 20000, "mass_kg": 100}},
    "mid":  {"envelope": {"len_m": 30, "dia_m": 9.6, "skin_kg": 200},
             "fins": {"s_m2": 48, "mass_kg": 95},
             "motor": {"t_max_N": 450, "mass_kg": 160, "disc_m2": 5.7},
             "battery": {"wh": 20000, "mass_kg": 100}},
    "top":  {"envelope": {"len_m": 35, "dia_m": 11.2, "skin_kg": 290},
             "fins": {"s_m2": 48, "mass_kg": 95},
             "motor": {"t_max_N": 450, "mass_kg": 160, "disc_m2": 5.7},
             "battery": {"wh": 30000, "mass_kg": 150}},
}


def pick_ship(alt_max: float) -> dict:
    if alt_max < 1000:
        return json.loads(json.dumps(SHIPS["base"]))
    if alt_max < 3600:
        return json.loads(json.dumps(SHIPS["mid"]))
    return json.loads(json.dumps(SHIPS["top"]))


def fly(from_id: str, to_id: str, slot: int, verbose: bool = False) -> dict:
    W = gc.WORLD
    A, B = W["stations"][from_id], W["stations"][to_id]
    leg = next(l for l in W["legs"]
               if {l["a"], l["b"]} == {from_id, to_id})
    az = math.degrees(math.atan2(B["mx"] - A["mx"], -(B["my"] - A["my"])))
    d_alt = B["alt"] - A["alt"]
    ship = pick_ship(max(A["alt"], B["alt"]))
    phys = gc.make_leg_env({"biome": leg["biome"], "slot": slot, "az_deg": az,
                            "d_alt_m": d_alt, "dist_m": leg["dist"],
                            "ground_alt_m": A["alt"], "seed": 11})
    # 配平按较高一端(爬升靠浮力,到顶近中性;下坡途中放氦)
    trim_env = {"ground_alt_m": max(A["alt"], B["alt"]), "air_temp_C": None}
    ship["ballast_kg"] = gc.auto_ballast(ship, trim_env, max_ballast_kg=400.0)
    ship["cargo_kg"] = 0.0
    spec = {"ship": ship, "env": phys["env"], "terrain": phys["terrain"],
            "gusts": phys["gusts"],
            "route": {"start_n": 0, "start_e": 0,
                      "dest_n": leg["dist"] * math.cos(math.radians(az)),
                      "dest_e": leg["dist"] * math.sin(math.radians(az)),
                      "fly_alt_agl_m": 40},
            "seed": 11}
    gc.start_leg(json.dumps(spec))
    gc.leg_command(json.dumps({"type": "cruise", "on": True, "speed": 8.0}))
    cap_s = leg["dist"] / 3.5 + 520.0 + abs(d_alt) * 0.8
    st = None
    t0 = time.time()
    while True:
        gc.leg_step(2000)                       # 10 仿真秒
        st = json.loads(gc.leg_state())
        # 下降策略(玩家同款 V 键):高于目的地且偏轻 → 放氦
        if d_alt < -200 and st["agl_m"] > 180 and st["net_lift_kg"] > -25:
            gc.leg_command(json.dumps({"type": "vent_helium", "kg": 12}))
        # 到站悬高:接近目的地但下不来(近中性/偏轻)→ 放氦压高度
        if st["dest_dist_m"] < 600 and st["agl_m"] > 75 and st["net_lift_kg"] > -10:
            gc.leg_command(json.dumps({"type": "vent_helium", "kg": 8}))
        if verbose:
            print(f"    t={st['t_s']:5.0f} dist={st['dest_dist_m']:6.0f} agl={st['agl_m']:5.0f} "
                  f"u={st['u_m_s']:4.1f} lift={st['net_lift_kg']:+6.0f} batt={st['battery_frac']*100:3.0f}%")
        if st["outcome"] is not None or st["t_s"] > cap_s:
            break
    return {"from": from_id, "to": to_id, "slot": slot, "biome": leg["biome"],
            "outcome": (st["outcome"] or {"result": "timeout"}),
            "t_s": st["t_s"], "wall_s": round(time.time() - t0, 1),
            "battery": round(st["battery_frac"], 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slot", type=int, default=1, help="出发时段 0..3(默认正午)")
    ap.add_argument("--leg", default=None, help="只验一条,如 yunti:shidian")
    ap.add_argument("-v", action="store_true")
    args = ap.parse_args()

    W = gc.WORLD
    runs = []
    if args.leg:
        a, b = args.leg.split(":")
        runs.append((a, b))
    else:
        for l in W["legs"]:
            up = (a, b) = ((l["a"], l["b"])
                           if W["stations"][l["b"]]["alt"] >= W["stations"][l["a"]]["alt"]
                           else (l["b"], l["a"]))
            runs.append(up)
        # 关键回程(大下坡):高原下撤 + 冰川顺急流
        runs.append(("shidian", "yunti"))
        runs.append(("jiguang", "bingshe"))

    fails = 0
    for a, b in runs:
        r = fly(a, b, args.slot, verbose=args.v)
        ok = r["outcome"]["result"] == "arrived"
        fails += (not ok)
        print(f"{'PASS' if ok else 'FAIL'}  {a:>9s}→{b:<9s} [{r['biome']:<13s}] "
              f"{r['outcome']['result']:<8s} t={r['t_s']:5.0f}s batt={r['battery']:.2f} "
              f"(wall {r['wall_s']}s)", flush=True)
    print(f"\n{'全部通过 ✓' if fails == 0 else f'{fails} 条失败 ✗'}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
