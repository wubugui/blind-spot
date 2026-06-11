"""启动实时仿真 + WebSocket/HTTP 服务。

用法: python scenarios/run_realtime.py [--config saved.json] [--wind 0.5]
浏览器打开 http://localhost:8000/ 查看 3D 观测窗口。
"""
import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from airship_sim.config import SimConfig, default_config
from airship_sim.server import RealtimeServer
import asyncio

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=str, default=None, help="JSON 配置文件路径")
    ap.add_argument("--wind", type=float, default=0.0, help="常值北风风速 m/s(指向南)")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8000)
    args = ap.parse_args()

    cfg = SimConfig.load(args.config) if args.config else default_config()
    if args.wind:
        cfg.atmosphere.wind_ned_m_s = (-args.wind, 0.0, 0.0)
    server = RealtimeServer(cfg, ws_port=args.ws_port, http_port=args.http_port)
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        print("\n[exit]")
