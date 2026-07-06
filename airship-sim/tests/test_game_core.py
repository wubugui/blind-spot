"""游戏核心桥端到端测试:建航段→巡航→到站;坠毁判定;机库评级。"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "game"))
import game_core as gc


def base_ship(t_max=300.0):
    return {
        "envelope": {"len_m": 25.0, "dia_m": 8.0, "skin_kg": 128.0},
        "fins": {"s_m2": 30.0, "mass_kg": 55.0},
        "motor": {"t_max_N": t_max, "mass_kg": 60.0, "disc_m2": 3.14},
        "battery": {"wh": 20000.0, "mass_kg": 100.0},
        "ballast_kg": 100.0,
        "cargo_kg": 0.0,
    }


def plains_spec(dist=1200.0):
    ship = base_ship()
    env = {"ground_alt_m": 200.0, "air_temp_C": None, "turbulence": 0.2,
           "wind_field": {"kind": "loglaw",
                          "params": {"u_ref_m_s": 3.0, "dir_to_deg": 90.0,
                                     "z0_m": 0.03}}}
    # 压舱自动配平到略重
    ship["ballast_kg"] = gc.auto_ballast(ship, env)
    return {"ship": ship, "env": env,
            "terrain": {"base_m": 0.0,
                        "hills": {"amp_m": 3.0, "wavelength_m": 500.0, "seed": 3}},
            "route": {"start_n": 0.0, "start_e": 0.0,
                      "dest_n": dist, "dest_e": 0.0, "fly_alt_agl_m": 40.0},
            "seed": 7}


def test_leg_cruise_to_arrival():
    """完整旅程:起飞态→巡航→抵达判定触发。"""
    gc.start_leg(json.dumps(plains_spec(1200.0)))
    gc.leg_command(json.dumps({"type": "cruise", "on": True, "speed": 9.0}))
    for _ in range(60):                      # 最多 60×1000 tick×5ms = 300 仿真秒
        gc.leg_step(1000)
        st = json.loads(gc.leg_state())
        if st["outcome"] is not None:
            break
    assert st["outcome"] is not None, "300s 内未到站"
    assert st["outcome"]["result"] == "arrived", f"结局: {st['outcome']}"
    assert st["battery_frac"] > 0.5          # 电量充足


def test_leg_state_schema():
    gc.start_leg(json.dumps(plains_spec()))
    gc.leg_step(2000)
    st = json.loads(gc.leg_state())
    for k in ("pos_three", "quat_three_xyzw", "agl_m", "net_lift_kg",
              "battery_frac", "dest_dist_m", "dest_bearing_deg", "wind_three",
              "sp", "yaw_deg", "u_m_s"):
        assert k in st, f"缺字段 {k}"
    assert all(np.isfinite(st["pos_three"]))
    g = json.loads(gc.leg_terrain(1500, 1500, 24))
    assert len(g["heights"]) == 24 and len(g["heights"][0]) == 24


def test_crash_on_steep_impact():
    """满速冲向高坡 → 坠毁判定。"""
    spec = plains_spec(3000.0)
    spec["terrain"] = {"base_m": 0.0,
                       "ridges": [{"center_n": 600.0, "center_e": 0.0,
                                   "axis_deg": 90.0, "height_m": 120.0,
                                   "halfwidth_m": 150.0}]}
    spec["route"]["fly_alt_agl_m"] = 30.0
    gc.start_leg(json.dumps(spec))
    # 直冲山脊:锁高度 30m(山有 120m),全速
    gc.leg_command(json.dumps({"type": "cruise", "on": True, "speed": 12.0}))
    for _ in range(30):                      # 30×5s = 150 仿真秒
        gc.leg_step(1000)
        st = json.loads(gc.leg_state())
        if st["outcome"] is not None:
            break
    assert st["outcome"] is not None and st["outcome"]["result"] == "crashed", \
        f"应撞山坠毁,实得 {st['outcome']}"


def test_ballast_drop_and_vent():
    gc.start_leg(json.dumps(plains_spec()))
    st0 = json.loads(gc.leg_state())
    gc.leg_command(json.dumps({"type": "drop_ballast", "kg": 30.0}))
    st1 = json.loads(gc.leg_state())
    assert st1["net_lift_kg"] > st0["net_lift_kg"] + 25       # 变轻
    gc.leg_command(json.dumps({"type": "vent_helium", "kg": 5.0}))
    st2 = json.loads(gc.leg_state())
    assert st2["net_lift_kg"] < st1["net_lift_kg"]            # 放氦变重


def test_hangar_rating_sane():
    ship = base_ship(300.0)
    env = {"ground_alt_m": 0.0, "air_temp_C": None}
    r = json.loads(gc.hangar_rate(json.dumps(ship), json.dumps(env)))
    assert 10 < r["v_thrust_m_s"] < 25
    assert r["endurance_h"] > 1.0
    # 高原同船净浮力显著变小
    r45 = json.loads(gc.hangar_rate(json.dumps(ship),
                                    json.dumps({"ground_alt_m": 4500.0})))
    assert r45["net_lift_kg"] < r["net_lift_kg"] - 200
    # 自动配平:配平后净重 ≈ 目标
    b = gc.hangar_auto_ballast(json.dumps(ship), json.dumps(env))
    ship2 = dict(ship); ship2["ballast_kg"] = b
    r2 = json.loads(gc.hangar_rate(json.dumps(ship2), json.dumps(env)))
    assert abs(r2["net_lift_kg"] + 8.0) < 1.0 or b in (0.0, 300.0)
