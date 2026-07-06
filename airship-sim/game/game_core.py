"""《风程》游戏核心桥(Python,跑在 Pyodide;也可在 CPython 无头测试)。

职责:把 JS 侧的"航段规格"(船配置 + 环境 + 地形 + 起终点)翻译成引擎对象,
提供逐帧推进/状态导出/机库评级/飞行中指令。游戏经济与界面在 JS 侧,
本模块不含 UI 逻辑;引擎(airship_sim)保持纯净,本模块只调用其公开接口。

坐标:NED,z=0 为该航段出发驿站地面(atmosphere.ground_alt_m = 驿站海拔)。
"""
from __future__ import annotations

import json
import math

import numpy as np

from airship_sim.atmosphere import IsaAtmosphere, isa_state
from airship_sim.added_mass import lamb_k_factors
from airship_sim.config import GRAVITY_M_S2, AtmosphereConfig
from airship_sim.dynamics import (IDX_OMEGA, IDX_POS, IDX_QUAT, IDX_THRUST,
                                  IDX_VEL, build_mass_properties)
from airship_sim.ground import AnalyticTerrain
from airship_sim.math3d import quat_from_euler, quat_to_euler, wrap_angle_rad
from airship_sim.presets_crewed import crewed_config, retune_gains
from airship_sim.simulation import Simulation
from airship_sim.wind_field import build_wind_field

STRUCT_BASE_KG = 250.0     # 龙骨/吊舱结构/系统(不随改装变)
PILOT_KG = 80.0

# 坠毁阈值(设计文档:垂直 >4 m/s 或水平 >8 m/s 触地 = 坠毁;水面接触一律救援)
CRASH_V_DOWN = 4.0
CRASH_V_HORIZ = 8.0
ARRIVE_RADIUS_M = 150.0
ARRIVE_AGL_M = 60.0


class Leg:
    """一个航段的运行时状态。"""

    def __init__(self, spec: dict):
        self.spec = spec
        ship = spec["ship"]
        env = spec["env"]

        cfg = crewed_config(heaviness_kg=5.0)
        # ---- 机身(机库改装结果) ----
        cfg.hull.length_m = float(ship["envelope"]["len_m"])
        cfg.hull.diameter_m = float(ship["envelope"]["dia_m"])
        cfg.mass.m_envelope_kg = (float(ship["envelope"]["skin_kg"])
                                  + float(ship["fins"]["mass_kg"]))
        cfg.fins.s_plane_m2 = float(ship["fins"]["s_m2"])
        cfg.fins.x_fin_m = -0.8 * cfg.hull.a_m
        cfg.actuator.thrust_max_N = float(ship["motor"]["t_max_N"])
        cfg.actuator.motor_sep_y_m = 0.3 * cfg.hull.diameter_m + 1.6
        cfg.actuator.motor_z_b_m = cfg.hull.b_m + 0.6
        cfg.mass.r_gondola_b_m = (0.0, 0.0, cfg.hull.b_m + 0.6)
        cfg.energy.capacity_Wh = float(ship["battery"]["wh"])
        cfg.energy.battery_mass_kg = float(ship["battery"]["mass_kg"])
        cfg.energy.prop_disc_area_m2 = float(ship["motor"].get("disc_m2", 3.14))
        cfg.energy.solar_area_m2 = float(ship.get("solar_m2", 0.0))
        cfg.helium.absorptivity = float(ship.get("absorptivity", 0.2))
        # 吊舱集总质量 = 结构 + 飞行员 + 动力 + 电池 + 压舱 + 货
        self.ballast_kg = float(ship.get("ballast_kg", 0.0))
        self.cargo_kg = float(ship.get("cargo_kg", 0.0))
        cfg.mass.m_gondola_kg = (STRUCT_BASE_KG + PILOT_KG
                                 + float(ship["motor"]["mass_kg"])
                                 + float(ship["battery"]["mass_kg"])
                                 + self.ballast_kg + self.cargo_kg)

        # ---- 环境 ----
        ground_alt = float(env.get("ground_alt_m", 0.0))
        t_std, _, _ = isa_state(ground_alt, 0.0)
        temp_C = env.get("air_temp_C")
        dT = 0.0 if temp_C is None else (float(temp_C) + 273.15) - t_std
        cfg.atmosphere = AtmosphereConfig(
            model="isa", ground_alt_m=ground_alt, dT_K=dT,
            turbulence_intensity=float(env.get("turbulence", 0.3)),
            wind_field=env.get("wind_field"))
        cfg.helium.enabled = bool(env.get("solar_on", False))
        cfg.helium.solar_on = bool(env.get("solar_on", False))
        cfg.helium.solar_flux_W_m2 = float(env.get("solar_flux_W_m2", 1000.0))
        cfg.seed = int(spec.get("seed", 20260611))
        # 游戏用 5ms 物理步长:载人艇动力学慢(最高频=接触 ~2Hz、电机 τ=0.5s),
        # RK4@5ms 精度损失可忽略,换 ~5× 实时性(Pyodide 里保住 ≥1× 实时)。
        # 5ms 下各采样率仍为整数 tick:IMU 100Hz=2、控制 50Hz=4、气压计 10Hz=20。
        cfg.dt_physics_s = 0.005
        retune_gains(cfg, 5.0)
        self.cfg = cfg

        self.terrain = AnalyticTerrain(spec.get("terrain") or {})
        self.sim = Simulation(cfg, terrain=self.terrain)

        # ---- 起终点与初始姿态(迎风)----
        route = spec["route"]
        self.start = np.array([float(route["start_n"]), float(route["start_e"])])
        self.dest = np.array([float(route["dest_n"]), float(route["dest_e"])])
        fly_agl = float(route.get("fly_alt_agl_m", 40.0))
        h0 = self.terrain.height_m(self.start[0], self.start[1])
        p0 = (self.start[0], self.start[1], -(h0 + fly_agl))
        w = self.sim.atmosphere.get_state(np.array(p0), 0.0).wind_ned_m_s
        if np.hypot(w[0], w[1]) > 0.5:
            yaw0 = math.atan2(w[1], w[0]) + math.pi     # 迎风
        else:
            d = self.dest - self.start
            yaw0 = math.atan2(d[1], d[0])               # 无风朝目的地
        self.sim.reset(p_ned_m=p0, q=tuple(quat_from_euler(0, 0, yaw0)))
        sp = self.sim.controller.setpoints
        sp.pos_hold = False
        sp.altitude_m = h0 + fly_agl
        sp.yaw_rad = yaw0
        sp.speed_m_s = 0.0
        # 期望航向:实际下发的 yaw_sp 以 ≤0.12 rad/s 向它回转(大惯量船不吃阶跃,
        # 90° 跳变会让偏航环饱和进螺旋——与模型级 pos_hold 同一教训)
        self.yaw_target = yaw0
        # 期望速度:实发 speed_sp 按航向误差打折(转弯协调)。带速大角度转弯时
        # 侧滑 Munk 力矩超过全部偏航权限会甩艏(broach)——先收油、转正、再提速。
        self.speed_target = 0.0

        # 预置阵风事件(可选,种子确定)
        for g in spec.get("gusts", []):
            self.sim.atmosphere.trigger_gust(
                t_s=float(g["t_s"]), dir_to_deg=float(g["dir_deg"]),
                amplitude_m_s=float(g["amp_m_s"]), duration_s=float(g.get("dur_s", 6.0)))

        self.cruise = False
        # 地形保持辅助(默认开):沿航向前视,把高度设定顶到地形以上安全余量。
        # 设计基调"不过于硬核"——爬升航段不至于一头撞坡;可用指令关闭。
        self.terrain_assist = True
        self.outcome: dict | None = None     # {"result": "arrived"|"crashed"|...}
        self._gust_warned = list(spec.get("gusts", []))

    # ---- 运行 ----
    def _mass_total(self) -> float:
        return self.sim.dynamics.props.m_total_kg

    def _rebuild_mass(self) -> None:
        """压舱/氦变化后重建质量属性(沿用引擎 set_helium_mass 的失效机制)。"""
        d = self.sim.dynamics
        d.props = build_mass_properties(self.cfg, m_helium_kg=self.sim.helium_m_kg)
        d._minv_rho_key = None

    def step(self, n: int) -> None:
        if self.outcome is not None:
            return
        sp = self.sim.controller.setpoints
        for i in range(int(n)):
            if self.cruise and self.sim.tick % 100 == 0:   # 0.5s 刷新巡航期望航向
                p = self.sim.x[IDX_POS]
                d = self.dest - p[0:2]
                self.yaw_target = math.atan2(d[1], d[0])
            if self.sim.tick % 20 == 0 and not sp.pos_hold:
                # 航向设定回转限速 0.12 rad/s(10Hz × 0.012)
                err = wrap_angle_rad(self.yaw_target - sp.yaw_rad)
                sp.yaw_rad = wrap_angle_rad(
                    sp.yaw_rad + float(np.clip(err, -0.012, 0.012)))
                # 转弯协调:按真实航向误差收油(0.6 rad 以上完全收油)
                yaw_now = quat_to_euler(self.sim.x[IDX_QUAT])[2]
                yaw_err = abs(wrap_angle_rad(self.yaw_target - yaw_now))
                factor = float(np.clip(1.0 - yaw_err / 0.6, 0.0, 1.0))
                sp.speed_m_s = self.speed_target * factor
                if self.terrain_assist:
                    # 地形保持:当前 + 前视(≈12s 航程)地形上方 ≥28m,设定只升不降
                    p = self.sim.x[IDX_POS]
                    look = 12.0 * max(abs(self.speed_target), 2.0)
                    hn = self.terrain.height_m(float(p[0]), float(p[1]))
                    ha = self.terrain.height_m(
                        float(p[0] + look * math.cos(yaw_now)),
                        float(p[1] + look * math.sin(yaw_now)))
                    need = max(hn, ha) + 28.0
                    if sp.altitude_m < need:
                        sp.altitude_m = min(sp.altitude_m + 0.8, need)
            self.sim.step(record=False)
            if self.sim.tick % 20 == 0:                    # 10Hz 判定到站/坠毁
                if self._check_end():
                    break

    def _check_end(self) -> bool:
        sim = self.sim
        p = sim.x[IDX_POS]
        # 坠毁/触地判定
        imp = sim.last_impact
        if imp is not None:
            if imp["water"]:
                self.outcome = {"result": "water", "impact": imp}
                return True
            if (imp["v_down_m_s"] > CRASH_V_DOWN
                    or imp["v_horiz_m_s"] > CRASH_V_HORIZ):
                self.outcome = {"result": "crashed", "impact": imp}
                return True
            sim.last_impact = None      # 轻触地:清除,允许贴地飞
        # 到站判定
        dist = float(np.hypot(*(self.dest - p[0:2])))
        h = self.terrain.height_m(p[0], p[1])
        agl = -p[2] - h
        if dist < ARRIVE_RADIUS_M and agl < ARRIVE_AGL_M:
            self.outcome = {"result": "arrived",
                            "battery_frac": (sim.energy.fraction
                                             if sim.energy else 1.0)}
            return True
        if sim.energy is not None and sim.energy.depleted and agl < 2.0:
            self.outcome = {"result": "stranded"}
            return True
        return False

    # ---- 指令 ----
    def command(self, msg: dict) -> None:
        t = msg.get("type")
        sp = self.sim.controller.setpoints
        if t == "adjust":
            if "d_speed" in msg:
                self.speed_target = float(
                    np.clip(self.speed_target + msg["d_speed"], -4.0,
                            self.cfg.control.pos_hold_max_speed_m_s * 1.4))
            if "d_yaw_deg" in msg:
                self.yaw_target = wrap_angle_rad(
                    self.yaw_target + math.radians(msg["d_yaw_deg"]))
                self.cruise = False        # 手动改航向 = 退出巡航
            if "d_alt" in msg:
                sp.altitude_m = max(2.0, sp.altitude_m + float(msg["d_alt"]))
            sp.pos_hold = False
        elif t == "cruise":
            self.cruise = bool(msg["on"])
            if self.cruise:
                self.speed_target = float(msg.get("speed", 8.0))
                sp.pos_hold = False
        elif t == "hold":
            self.cruise = False
            sp.pos_hold = bool(msg.get("on", True))
            if sp.pos_hold:
                p = self.sim.x[IDX_POS]
                sp.target_n_m, sp.target_e_m = float(p[0]), float(p[1])
            else:
                self.speed_target = 0.0
                sp.speed_m_s = 0.0
        elif t == "drop_ballast":
            amount = min(float(msg.get("kg", 20.0)), self.ballast_kg)
            self.ballast_kg -= amount
            self.cfg.mass.m_gondola_kg -= amount
            self._rebuild_mass()
        elif t == "vent_helium":
            # 放氦下降:排出的氦体积由波纹管(ballonet)进气充填(囊体积不变),
            # 净效应 = 该体积里氦换成空气,质量增加 (ρ_air − ρ_he)·V_vented。
            # [假设] 排气瞬时完成、囊体积恒定 / 与真实飞艇 ballonet 操作一致
            # / 无 ballonet 的软式囊体应改为体积收缩模型。
            amount = min(float(msg.get("kg", 3.0)), self.sim.helium_m_kg - 10.0)
            if amount > 0:
                atm = self.sim.atmosphere.get_state(self.sim.x[IDX_POS], self.sim.t_s)
                v_vented = amount / self.cfg.mass.rho_helium_kg_m3
                self.cfg.mass.m_gondola_kg += atm.rho_kg_m3 * v_vented  # ballonet 空气
                self.sim.helium_m_kg -= amount
                self._rebuild_mass()
        elif t == "assist":
            self.terrain_assist = bool(msg.get("on", True))
        elif t == "manual":
            self.sim.controller.manual_mode = bool(msg["enable"])
        elif t == "manual_cmd":
            mc = self.sim.controller.manual_cmd
            mc.left_N = float(msg.get("left_N", mc.left_N))
            mc.right_N = float(msg.get("right_N", mc.right_N))
            mc.vertical_N = float(msg.get("vertical_N", mc.vertical_N))

    # ---- 状态导出 ----
    def state(self) -> dict:
        sim = self.sim
        x = sim.x
        q = x[IDX_QUAT]
        eul = quat_to_euler(q)
        p = x[IDX_POS]
        atm = sim.atmosphere.get_state(p, sim.t_s)
        h_gnd = self.terrain.height_m(p[0], p[1])
        d = self.dest - p[0:2]
        dist = float(np.hypot(d[0], d[1]))
        rho = atm.rho_kg_m3
        v = self.cfg.hull.volume_m3
        m_total = self._mass_total()
        sp = sim.controller.setpoints
        # 近期阵风预警(未来 20s 内)
        warn = None
        for g in self._gust_warned:
            if 0 < float(g["t_s"]) - sim.t_s < 20.0:
                warn = f"阵风将至 {float(g['t_s']) - sim.t_s:.0f}s"
                break
        return {
            "t_s": round(sim.t_s, 2),
            "pos_ned": [float(c) for c in p],
            "pos_three": [float(p[1]), float(-p[2]), float(-p[0])],
            "quat_three_xyzw": [float(q[2]), float(-q[3]), float(-q[1]), float(q[0])],
            "yaw_deg": math.degrees(eul[2]),
            "pitch_deg": math.degrees(eul[1]),
            "roll_deg": math.degrees(eul[0]),
            "u_m_s": float(x[IDX_VEL][0]),
            "agl_m": float(-p[2] - h_gnd),
            "alt_msl_m": float(self.cfg.atmosphere.ground_alt_m - p[2]),
            "wind_ned": [float(c) for c in atm.wind_ned_m_s],
            "wind_three": [float(atm.wind_ned_m_s[1]), float(-atm.wind_ned_m_s[2]),
                           float(-atm.wind_ned_m_s[0])],
            "rho": float(rho),
            "net_lift_kg": float(rho * v - m_total),
            "ballast_kg": self.ballast_kg,
            "helium_kg": float(sim.helium_m_kg),
            "battery_frac": (sim.energy.fraction if sim.energy else 1.0),
            "power_W": (sim.energy.last_power_W if sim.energy else 0.0),
            "thrust_N": [float(c) for c in x[IDX_THRUST]],
            "dest_dist_m": dist,
            "dest_bearing_deg": math.degrees(math.atan2(d[1], d[0])),
            "sp": {"alt": sp.altitude_m, "yaw_deg": math.degrees(self.yaw_target),
                   "speed": self.speed_target, "hold": sp.pos_hold},
            "cruise": self.cruise,
            "manual": sim.controller.manual_mode,
            "contact": sim.contact["contact"],
            "warning": warn,
            "outcome": self.outcome,
        }

    def terrain_grid(self, half_n: float, half_e: float, res: int) -> dict:
        """以航段中点为中心导出高度网格(渲染用)+ 水面掩码。"""
        c = 0.5 * (self.start + self.dest)
        ns = np.linspace(c[0] - half_n, c[0] + half_n, res)
        es = np.linspace(c[1] - half_e, c[1] + half_e, res)
        hs, wat = [], []
        for nn in ns:
            row_h, row_w = [], []
            for ee in es:
                row_h.append(round(self.terrain.height_m(nn, ee), 2))
                row_w.append(1 if self.terrain.is_water(nn, ee) else 0)
            hs.append(row_h)
            wat.append(row_w)
        return {"center_n": float(c[0]), "center_e": float(c[1]),
                "half_n": half_n, "half_e": half_e, "res": res,
                "heights": hs, "water": wat,
                "start": [float(self.start[0]), float(self.start[1])],
                "dest": [float(self.dest[0]), float(self.dest[1])]}


# ---------- 机库评级(解析,即时) ----------
def rate_ship(ship: dict, env: dict) -> dict:
    """净浮力 / 静稳定 / 最大可顶风 / 续航估计。全解析,机库面板即时刷新。"""
    len_m = float(ship["envelope"]["len_m"])
    dia_m = float(ship["envelope"]["dia_m"])
    a, b = len_m / 2.0, dia_m / 2.0
    v = 4.0 / 3.0 * math.pi * a * b * b
    m_total = (float(ship["envelope"]["skin_kg"]) + float(ship["fins"]["mass_kg"])
               + 0.18 * v + STRUCT_BASE_KG + PILOT_KG
               + float(ship["motor"]["mass_kg"]) + float(ship["battery"]["mass_kg"])
               + float(ship.get("ballast_kg", 0.0)) + float(ship.get("cargo_kg", 0.0)))
    ground_alt = float(env.get("ground_alt_m", 0.0))
    t_std, _, _ = isa_state(ground_alt, 0.0)
    temp_C = env.get("air_temp_C")
    dT = 0.0 if temp_C is None else (float(temp_C) + 273.15) - t_std
    _, _, rho = isa_state(ground_alt + 40.0, dT)
    net_lift = rho * v - m_total

    k1, k2, _ = lamb_k_factors(a, b)
    x_fin = 0.8 * a
    fin_term = float(ship["fins"]["s_m2"]) * 2.6 * x_fin
    munk_term = 2.0 * (k2 - k1) * v
    static_ratio = fin_term / munk_term

    t_max = float(ship["motor"]["t_max_N"])
    cd_ax, area = 0.075, math.pi * b * b
    v_thrust = math.sqrt(2.0 * t_max / (0.5 * 1.225 * cd_ax * area))  # ρ 消去
    resid = munk_term / 2.0 - 0.5 * 2.6 * float(ship["fins"]["s_m2"]) * x_fin
    tau_max = 2.0 * t_max * (0.3 * dia_m + 1.6)
    if resid <= 0:
        v_yaw = 99.0
    else:
        v_yaw = math.sqrt(tau_max / (rho * resid * 0.05))   # 允许 ~3° 侧滑
    v_hold = min(v_thrust, v_yaw)

    # 续航:巡航 8 m/s,双桨推力 = 阻力/2
    drag8 = 0.5 * rho * cd_ax * area * 64.0
    t_each = drag8 / 2.0
    v_i = math.sqrt(max(t_each, 1.0) / (2.0 * rho * 3.14))
    p_cruise = 2.0 * t_each * (8.0 + v_i) / 0.6 + 100.0
    endurance_h = float(ship["battery"]["wh"]) / max(p_cruise, 1.0)

    return {"net_lift_kg": round(net_lift, 1),
            "static_ratio": round(static_ratio, 2),
            "v_hold_m_s": round(v_hold, 1),
            "v_thrust_m_s": round(v_thrust, 1),
            "v_yaw_m_s": round(min(v_yaw, 99.0), 1),
            "endurance_h": round(endurance_h, 1),
            "mass_total_kg": round(m_total, 1),
            "buoyancy_kg": round(rho * v, 1),
            "rho": round(rho, 3)}


def auto_ballast(ship: dict, env: dict, target_heavy_kg: float = 8.0,
                 max_ballast_kg: float = 300.0) -> float:
    """自动配平:返回使净浮力 = −target_heavy_kg(略重)的压舱量(0..max)。

    略重是飞艇标准配平:垂直电机持续小幅托举,断电时缓慢下沉(而非飘走)。
    """
    s = dict(ship)
    s["ballast_kg"] = 0.0
    r = rate_ship(s, env)
    ballast = r["net_lift_kg"] + target_heavy_kg
    return float(np.clip(ballast, 0.0, max_ballast_kg))


# ---------- 模块级接口(Pyodide 调用) ----------
_leg: Leg | None = None


def start_leg(spec_json: str) -> str:
    global _leg
    _leg = Leg(json.loads(spec_json))
    return json.dumps({"ok": True})


def leg_step(n: int) -> None:
    if _leg is not None:
        _leg.step(n)


def leg_state() -> str:
    return json.dumps(_leg.state() if _leg is not None else {"outcome": None})


def leg_command(msg_json: str) -> None:
    if _leg is not None:
        _leg.command(json.loads(msg_json))


def leg_terrain(half_n: float, half_e: float, res: int) -> str:
    return json.dumps(_leg.terrain_grid(half_n, half_e, int(res)))


def hangar_rate(ship_json: str, env_json: str) -> str:
    return json.dumps(rate_ship(json.loads(ship_json), json.loads(env_json)))


def hangar_auto_ballast(ship_json: str, env_json: str) -> float:
    return auto_ballast(json.loads(ship_json), json.loads(env_json))


# ---------- 航段环境生成(生物群系 × 时段;与 JS 世界数据解耦的物理单一来源) ----------
SLOT_NAMES = ("dawn", "noon", "dusk", "night")


def make_leg_env(params: dict) -> dict:
    """由 {biome, slot(0..3), az_deg(航线去向), d_alt_m, dist_m, ground_alt_m, seed}
    生成 {env, terrain, gusts, note}。地形与风场同源:坡向=下坡方位,山脊⟂航线。"""
    biome = params["biome"]
    slot = int(params["slot"])
    az = float(params["az_deg"])
    d_alt = float(params["d_alt_m"])
    dist = float(params["dist_m"])
    ground_alt = float(params["ground_alt_m"])
    seed = int(params.get("seed", 0))
    grade = d_alt / dist
    az_down = (az + 180.0) % 360.0 if d_alt >= 0 else az

    terrain = {"base_m": 0.0,
               "slope": {"azimuth_deg": az_down, "grade": abs(grade)},
               "hills": {"amp_m": 4.0, "wavelength_m": 600.0, "seed": seed % 97}}
    wf = None
    turb = 0.25
    note = []
    day = slot in (1, 2)

    if biome == "plains":
        u = [1.5, 3.5, 2.5, 1.2][slot]
        wf = {"kind": "loglaw", "params": {"u_ref_m_s": u,
              "dir_to_deg": (az + 140) % 360, "z0_m": 0.03}}
        terrain["hills"]["amp_m"] = 6.0
        turb = 0.5 if slot == 1 else 0.2
        note.append(f"地面风约 {u} m/s")
    elif biome in ("lake", "lake_open"):
        onshore = (az + 180) % 360
        breeze = slot in (1, 2)
        u = 3.0 if breeze else 1.5
        if breeze:
            wf = {"kind": "lake_env", "params": {"onshore_dir_deg": onshore,
                  "u10_m_s": u, "breeze": True, "inflow_speed_m_s": 4.0,
                  "front_speed_m_s": 1.5, "z0_m": 1e-3}}
        else:
            wf = {"kind": "loglaw", "params": {"u_ref_m_s": u,
                  "dir_to_deg": onshore, "z0_m": 1e-3}}
        terrain["hills"]["amp_m"] = 2.0
        terrain["water_level_m"] = 0.5 if biome == "lake_open" else -2.0
        if biome == "lake_open":
            terrain["slope"]["grade"] = 0.0
        turb = 0.4 if breeze else 0.15
        note.append("湖陆风锋面活跃(入流 4 m/s,锋面推进)" if breeze else "湖面平静")
        if biome == "lake_open":
            note.append("全程水面:落水=货物尽失")
    elif biome == "foothill":
        u = [3, 4, 3.5, 2.5][slot]
        wf = {"kind": "loglaw", "params": {"u_ref_m_s": u,
              "dir_to_deg": (az_down + 180) % 360, "z0_m": 0.05}}
        turb = 0.35
        note.append(f"谷风约 {u} m/s,持续爬升 {d_alt:.0f} m")
    elif biome in ("ridge", "ridge_high"):
        u = [5, 7, 6, 4][slot]
        ridge_h = 90.0 if biome == "ridge" else 130.0
        cn = (dist / 2) * math.cos(math.radians(az))
        ce = (dist / 2) * math.sin(math.radians(az))
        terrain["ridges"] = [{"center_n": cn, "center_e": ce,
                              "axis_deg": (az + 90) % 360,
                              "height_m": ridge_h, "halfwidth_m": 260.0}]
        wf = {"kind": "ridge",
              "base": {"kind": "loglaw", "params": {"u_ref_m_s": u,
                       "dir_to_deg": az, "z0_m": 0.05}},
              "params": {"crest_ne_m": [cn, ce], "ridge_axis_deg": (az + 90) % 360,
                         "height_m": ridge_h, "halfwidth_m": 260.0,
                         "z_influence_m": 120.0,
                         "lee_recirc_frac": 0.35 if slot == 1 else 0.2}}
        turb = 0.5
        note.append(f"风门:脊顶顺风加速(基础 {u} m/s),背风面有回流")
    elif biome in ("plateau_climb", "plateau"):
        wf = {"kind": "plateau_slope", "params": {
              "downhill_azimuth_deg": az_down,
              "peak_speed_m_s": 3.0 if day else 5.0, "jet_height_m": 30.0,
              "daytime": day, "base_wind_m_s": 2.5, "z0_m": 0.03,
              "thermals": day, "thermal_updraft_m_s": 2.2}}
        turb = 0.7 if day else 0.35
        note.append("白天:上坡风+热泡(免费升力,但颠簸)" if day
                    else "夜/晨:下坡风 5 m/s")
        if biome == "plateau_climb":
            note.append(f"大爬升 {d_alt:.0f} m——建议轻配平,顶点放氦")
    elif biome in ("glacier_app", "glacier"):
        kata = [9, 4, 6, 8][slot]
        wf = {"kind": "glacier_katabatic", "params": {
              "downhill_azimuth_deg": az_down, "peak_speed_m_s": float(kata),
              "jet_height_m": 25.0, "base_wind_m_s": 2.0, "z0_m": 2e-4}}
        turb = 0.3
        head = abs(((az_down - az) % 360 + 540) % 360 - 180) > 90
        note.append(f"下降风急流 {kata} m/s @ ~25m"
                    f"({'顺你走' if head else '顶着你'});正午最弱")

    gusts = []
    if (seed % 10) < (5 if slot == 1 else 3):
        gusts.append({"t_s": 40.0 + seed % 90, "dir_deg": (az + 90 + seed % 180) % 360,
                      "amp_m_s": 2.0 + (seed % 30) / 10.0, "dur_s": 6.0})

    env = {"ground_alt_m": ground_alt, "air_temp_C": None, "turbulence": turb,
           "wind_field": wf, "solar_on": day,
           "solar_flux_W_m2": 1000.0 + ground_alt / 8.0}
    return {"env": env, "terrain": terrain, "gusts": gusts, "note": note}


def make_leg_spec(params_json: str) -> str:
    """JS 入口:世界数据参数 → 完整物理规格(不含 ship/route,由 JS 合并)。"""
    return json.dumps(make_leg_env(json.loads(params_json)))


# ---------- 世界数据(大陆/驿站/航线;JS 启动时经 get_world() 读取) ----------
WORLD = {
    "stations": {
        "weian":    {"name": "苇岸", "region": "plains", "alt": 200, "mx": 60, "my": 420,
                     "desc": "旅程的起点。芦苇荡边的小码头,风总是很温柔。"},
        "mailang":  {"name": "麦浪镇", "region": "plains", "alt": 230, "mx": 170, "my": 360,
                     "desc": "金色麦田上空常有轻缓的午后对流。"},
        "fengche":  {"name": "老风车", "region": "plains", "alt": 250, "mx": 150, "my": 480,
                     "desc": "风车匠人聚居地,可以打听风的脾气。"},
        "yinhu":    {"name": "银湖港", "region": "lake", "alt": 300, "mx": 290, "my": 400,
                     "desc": "大湖西岸。正午之后,湖风会推着锋面上岸。"},
        "shuangyu": {"name": "双屿", "region": "lake", "alt": 300, "mx": 390, "my": 340,
                     "desc": "湖心双岛。水面上没有热泡,也没有可以迫降的地方。"},
        "aikou":    {"name": "隘口堡", "region": "mountain", "alt": 1500, "mx": 470, "my": 430,
                     "desc": "山门。从这里开始,每一段都在爬升。"},
        "yingling": {"name": "鹰岭", "region": "mountain", "alt": 2300, "mx": 560, "my": 370,
                     "desc": "山脊上的驿站。风门的加速气流就在头顶。"},
        "yunti":    {"name": "云梯驿", "region": "mountain", "alt": 3200, "mx": 640, "my": 300,
                     "desc": "云上的台阶。空气开始变得稀薄。"},
        "shidian":  {"name": "石甸", "region": "plateau", "alt": 4200, "mx": 730, "my": 240,
                     "desc": "高原。密度只有海平面的六成——升力、推力都打了折。"},
        "jingfan":  {"name": "经幡台", "region": "plateau", "alt": 4400, "mx": 820, "my": 190,
                     "desc": "风把经幡吹成水平。白天有热泡,夜里有下坡风。"},
        "bingshe":  {"name": "冰舌前哨", "region": "glacier", "alt": 4600, "mx": 900, "my": 130,
                     "desc": "冰川末端。黎明的下降风像一条看不见的河。"},
        "jiguang":  {"name": "极光城", "region": "glacier", "alt": 4700, "mx": 980, "my": 70,
                     "desc": "旅程的终点。传说夜里天上有光。"},
    },
    "legs": [
        {"a": "weian", "b": "mailang", "dist": 1800, "biome": "plains"},
        {"a": "weian", "b": "fengche", "dist": 1600, "biome": "plains"},
        {"a": "mailang", "b": "yinhu", "dist": 2200, "biome": "lake"},
        {"a": "fengche", "b": "yinhu", "dist": 2400, "biome": "lake"},
        {"a": "yinhu", "b": "shuangyu", "dist": 2400, "biome": "lake_open"},
        {"a": "yinhu", "b": "aikou", "dist": 4800, "biome": "foothill"},
        {"a": "shuangyu", "b": "aikou", "dist": 4000, "biome": "foothill"},
        {"a": "aikou", "b": "yingling", "dist": 4000, "biome": "ridge"},
        {"a": "yingling", "b": "yunti", "dist": 4200, "biome": "ridge_high"},
        {"a": "yunti", "b": "shidian", "dist": 4800, "biome": "plateau_climb"},
        {"a": "shidian", "b": "jingfan", "dist": 2600, "biome": "plateau"},
        {"a": "jingfan", "b": "bingshe", "dist": 2800, "biome": "glacier_app"},
        {"a": "bingshe", "b": "jiguang", "dist": 3000, "biome": "glacier"},
    ],
}


def get_world() -> str:
    return json.dumps(WORLD, ensure_ascii=False)
