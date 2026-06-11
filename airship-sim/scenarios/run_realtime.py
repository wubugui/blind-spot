"""启动实时仿真 + WebSocket/HTTP 服务。

用法:
  python scenarios/run_realtime.py                       # 室内(常密度,无风)
  python scenarios/run_realtime.py --wind 0.5            # 室内 + 常值北风
  python scenarios/run_realtime.py --atmo low --turb 1   # ISA + 低空边界层风 + 湍流
  python scenarios/run_realtime.py --atmo high --helium  # ISA 高空剖面 + 氦气热力学
浏览器打开 http://localhost:8000/ 查看 3D 观测窗口与调参面板。
"""
import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from airship_sim.atmosphere import (wind_layers_high_altitude,
                                    wind_layers_low_altitude)
from airship_sim.config import SimConfig, default_config
from airship_sim.server import RealtimeServer
import asyncio

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=str, default=None, help="JSON 配置文件路径")
    ap.add_argument("--wind", type=float, default=0.0, help="常值北风风速 m/s(指向南)")
    ap.add_argument("--atmo", choices=["indoor", "low", "high"], default="indoor",
                    help="大气模式:indoor=常密度;low/high=ISA+预置风层")
    ap.add_argument("--turb", type=float, default=0.0, help="湍流强度倍率(isa 模型)")
    ap.add_argument("--helium", action="store_true", help="开启氦气热力学(太阳辐照默认开)")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8000)
    args = ap.parse_args()

    cfg = SimConfig.load(args.config) if args.config else default_config()
    if args.atmo != "indoor":
        cfg.atmosphere.model = "isa"
        cfg.atmosphere.turbulence_intensity = args.turb
        cfg.atmosphere.wind_layers = (wind_layers_low_altitude() if args.atmo == "low"
                                      else wind_layers_high_altitude())
    if args.wind:
        if args.atmo == "indoor":
            cfg.atmosphere.wind_ned_m_s = (-args.wind, 0.0, 0.0)
        else:
            cfg.atmosphere.wind_layers = tuple(
                (l[0], args.wind, 180.0) for l in cfg.atmosphere.wind_layers)
    if args.helium:
        cfg.helium.enabled = True
        cfg.helium.solar_on = True
    server = RealtimeServer(cfg, ws_port=args.ws_port, http_port=args.http_port)
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        print("\n[exit]")
