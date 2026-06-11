"""实时仿真服务:固定步长实时运行 + WebSocket 50Hz 状态广播 + 静态页面托管。

仿真进程不依赖前端存活:无客户端时照常推进,前端断线重连即恢复显示。

== 坐标系约定(协议层统一转换,渲染端不做任何坐标修补) ==
仿真:NED 右手系(x北, y东, z向下),四元数 [w,x,y,z] 体→世界。
Three.js:右手系 Y 向上。取 x_three=东, y_three=上(=-D), z_three=南(=-N),
这是一个真旋转(det=+1),四元数向量部分按同一旋转变换:
    pos_three  = ( e, -d, -n )
    quat_three = ( w, qy, -qz, -qx )   # [w,x,y,z] 顺序
(注意 Three.js 的 Quaternion 构造顺序是 (x,y,z,w),前端按 schema 字段取值。)

== 状态广播 schema(JSON,50Hz)==
{
  "type": "state",
  "t_s": float,                  # 仿真时间
  "running": bool,
  "time_scale": float,           # 时间倍率(>1 加速为尽力而为,见 README)
  "pos_ned_m": [n,e,d],
  "quat_wxyz": [w,x,y,z],        # NED 体→世界
  "pos_three": [x,y,z],          # 已转换,米
  "quat_three_xyzw": [x,y,z,w],  # 已转换,可直接喂 THREE.Quaternion
  "euler_deg": [roll,pitch,yaw],
  "v_body_m_s": [u,v,w],
  "omega_deg_s": [p,q,r],
  "thrust_N": [left,right,vert], # 电机实际推力
  "cmd_N": [left,right,vert],    # 电机指令
  "pid": {"alt": {"sp":..,"meas":..,"out":..},
           "yaw": {...}, "speed": {...}},   # 各通道 设定/测量/输出
  "wind_ned_m_s": [n,e,d],       # 飞艇当前位置的风(含湍流)
  "wind_three": [x,y,z],
  "rho_kg_m3": float,
  "manual": bool,
  "gains": {"alt": {kp,ki,kd,...}, "yaw":..., "speed":...}   # 当前增益(面板初始化)
}

== 控制指令 schema(JSON,客户端 → 服务端)==
{"type":"pause"} / {"type":"resume"} / {"type":"reset"}
{"type":"step"}                          # 暂停状态下单步推进一个控制周期(20ms)
{"type":"time_scale","value":2.0}
{"type":"set_gains","channel":"alt|yaw|speed","gains":{"kp":..,"ki":..,"kd":..}}
                                          # 缺省字段保持原值,实时生效不重启
{"type":"set_setpoints","altitude_m":..,"yaw_deg":..,"speed_m_s":..,
 "pos_hold":bool,"target_n_m":..,"target_e_m":..}      # 字段均可选
{"type":"manual","enable":true}          # 手动/自动模式切换
{"type":"manual_cmd","left_N":..,"right_N":..,"vertical_N":..}
{"type":"set_wind","wind_ned_m_s":[n,e,d]}             # 常值风(阶段5扩展为大气面板)
"""
from __future__ import annotations

import asyncio
import dataclasses
import functools
import http.server
import json
import os
import threading
import time

import numpy as np
import websockets

from .config import PidGains, SimConfig
from .dynamics import IDX_OMEGA, IDX_POS, IDX_QUAT, IDX_THRUST, IDX_VEL
from .math3d import quat_to_euler
from .simulation import Simulation

WEB_DIR = os.path.join(os.path.dirname(__file__), "..", "web")


def ned_to_three(v) -> list[float]:
    """NED 矢量 → Three.js 坐标(x=东, y=上, z=南)。"""
    return [float(v[1]), float(-v[2]), float(-v[0])]


def quat_ned_to_three_xyzw(q) -> list[float]:
    """NED 四元数 [w,x,y,z] → Three.js (x,y,z,w)。向量部分按基变换旋转。"""
    w, x, y, z = (float(c) for c in q)
    return [y, -z, -x, w]


class RealtimeServer:
    def __init__(self, cfg: SimConfig, host: str = "0.0.0.0",
                 ws_port: int = 8765, http_port: int = 8000,
                 init_pos_ned_m=(0.0, 0.0, -1.5)):
        self.sim = Simulation(cfg)
        self._init_pos = tuple(init_pos_ned_m)
        self._reset_sim()
        self.running = True
        self.time_scale = 1.0
        self.host, self.ws_port, self.http_port = host, ws_port, http_port
        self._clients: set = set()
        self._lock = threading.Lock()   # step/指令均在事件循环线程,锁保守起见保留
        self.ready = asyncio.Event()    # serve() 启动完成(端口已分配)后置位

    def _reset_sim(self) -> None:
        self.sim.reset(p_ned_m=self._init_pos)
        sp = self.sim.controller.setpoints
        sp.altitude_m = -self._init_pos[2]
        sp.pos_hold = True
        sp.target_n_m, sp.target_e_m = self._init_pos[0], self._init_pos[1]

    # ---- 状态序列化 ----
    def state_json(self) -> str:
        sim = self.sim
        x = sim.x
        q = x[IDX_QUAT]
        eul = quat_to_euler(q)
        d = sim.controller.debug
        atm = sim.atmosphere.get_state(x[IDX_POS], sim.t_s)
        ctl = sim.controller
        msg = {
            "type": "state",
            "t_s": round(sim.t_s, 4),
            "running": self.running,
            "time_scale": self.time_scale,
            "pos_ned_m": [float(v) for v in x[IDX_POS]],
            "quat_wxyz": [float(v) for v in q],
            "pos_three": ned_to_three(x[IDX_POS]),
            "quat_three_xyzw": quat_ned_to_three_xyzw(q),
            "euler_deg": [float(np.rad2deg(a)) for a in eul],
            "v_body_m_s": [float(v) for v in x[IDX_VEL]],
            "omega_deg_s": [float(np.rad2deg(v)) for v in x[IDX_OMEGA]],
            "thrust_N": [float(v) for v in x[IDX_THRUST]],
            "cmd_N": [float(v) for v in sim.cmd.as_array()],
            "pid": {
                "alt": {"sp": d.alt_sp_m, "meas": d.alt_meas_m, "out": d.alt_out_N},
                "yaw": {"sp": float(np.rad2deg(d.yaw_sp_rad)),
                        "meas": float(np.rad2deg(d.yaw_meas_rad)),
                        "out": d.yaw_out_Nm},
                "speed": {"sp": d.speed_sp_m_s, "meas": d.speed_meas_m_s,
                          "out": d.speed_out_N},
            },
            "wind_ned_m_s": [float(v) for v in atm.wind_ned_m_s],
            "wind_three": ned_to_three(atm.wind_ned_m_s),
            "rho_kg_m3": float(atm.rho_kg_m3),
            "manual": ctl.manual_mode,
            "gains": {ch: dataclasses.asdict(pid.g) for ch, pid in
                      [("alt", ctl.pid_alt), ("yaw", ctl.pid_yaw),
                       ("speed", ctl.pid_speed)]},
        }
        return json.dumps(msg)

    # ---- 指令处理 ----
    def handle_command(self, msg: dict) -> None:
        t = msg.get("type")
        sim = self.sim
        if t == "pause":
            self.running = False
        elif t == "resume":
            self.running = True
        elif t == "reset":
            with self._lock:
                self._reset_sim()
        elif t == "step":
            if not self.running:
                with self._lock:
                    for _ in range(sim._ticks_per_ctrl):
                        sim.step(record=False)
        elif t == "time_scale":
            self.time_scale = float(np.clip(float(msg["value"]), 0.1, 10.0))
        elif t == "set_gains":
            ch = msg["channel"]
            pid = {"alt": sim.controller.pid_alt, "yaw": sim.controller.pid_yaw,
                   "speed": sim.controller.pid_speed}[ch]
            cur = dataclasses.asdict(pid.g)
            cur.update({k: float(v) for k, v in msg["gains"].items()
                        if k in cur})
            sim.controller.set_gains(ch, PidGains(**cur))
        elif t == "set_setpoints":
            sp = sim.controller.setpoints
            if "altitude_m" in msg:
                sp.altitude_m = float(msg["altitude_m"])
            if "yaw_deg" in msg:
                sp.yaw_rad = float(np.deg2rad(float(msg["yaw_deg"])))
            if "speed_m_s" in msg:
                sp.speed_m_s = float(msg["speed_m_s"])
            if "pos_hold" in msg:
                sp.pos_hold = bool(msg["pos_hold"])
            if "target_n_m" in msg:
                sp.target_n_m = float(msg["target_n_m"])
            if "target_e_m" in msg:
                sp.target_e_m = float(msg["target_e_m"])
        elif t == "manual":
            sim.controller.manual_mode = bool(msg["enable"])
        elif t == "manual_cmd":
            mc = sim.controller.manual_cmd
            mc.left_N = float(msg.get("left_N", mc.left_N))
            mc.right_N = float(msg.get("right_N", mc.right_N))
            mc.vertical_N = float(msg.get("vertical_N", mc.vertical_N))
        elif t == "set_wind":
            sim.atmosphere.set_uniform_wind(msg["wind_ned_m_s"])

    # ---- 网络 ----
    async def _ws_handler(self, ws):
        self._clients.add(ws)
        try:
            async for raw in ws:
                try:
                    self.handle_command(json.loads(raw))
                except Exception as e:  # 单条坏指令不拖垮服务
                    await ws.send(json.dumps({"type": "error", "message": str(e)}))
        finally:
            self._clients.discard(ws)

    async def _broadcast(self, text: str) -> None:
        if not self._clients:
            return
        dead = []
        for ws in self._clients:
            try:
                await ws.send(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)

    async def _sim_loop(self) -> None:
        """实时节拍:每帧推进一个控制周期(20ms 仿真时间),按 time_scale 配速。
        落后于实时(如加速倍率超出算力)时放弃追赶,尽力而为。"""
        frame_sim_s = self.sim._ticks_per_ctrl * self.sim.cfg.dt_physics_s
        next_wall = time.perf_counter()
        while True:
            if self.running:
                with self._lock:
                    for _ in range(self.sim._ticks_per_ctrl):
                        self.sim.step(record=False)
            await self._broadcast(self.state_json())
            next_wall += frame_sim_s / self.time_scale
            delay = next_wall - time.perf_counter()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_wall = time.perf_counter()

    def _start_http(self) -> None:
        handler = functools.partial(
            http.server.SimpleHTTPRequestHandler, directory=os.path.abspath(WEB_DIR))
        self._httpd = http.server.ThreadingHTTPServer((self.host, self.http_port), handler)
        self.http_port = self._httpd.server_address[1]   # 端口 0 时取实际分配值
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        print(f"[http] serving {os.path.abspath(WEB_DIR)} at "
              f"http://localhost:{self.http_port}/")

    async def serve(self) -> None:
        """启动 HTTP+WS 并运行实时仿真循环。ws_port/http_port 传 0 表示自动分配,
        实际端口写回属性并置位 self.ready(测试用)。"""
        self._start_http()
        try:
            async with websockets.serve(self._ws_handler, self.host, self.ws_port) as ws_server:
                self.ws_port = ws_server.sockets[0].getsockname()[1]
                print(f"[ws] broadcasting at ws://localhost:{self.ws_port}/")
                self.ready.set()
                await self._sim_loop()
        finally:
            self._httpd.shutdown()


def main(cfg: SimConfig | None = None) -> None:
    from .config import default_config
    server = RealtimeServer(cfg if cfg is not None else default_config())
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        print("\n[exit]")


if __name__ == "__main__":
    main()
