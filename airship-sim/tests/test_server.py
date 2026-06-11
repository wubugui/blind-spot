"""WebSocket 服务:坐标变换正确性 + 协议集成测试(headless 客户端)。"""
import asyncio
import json
import urllib.request

import numpy as np
import pytest
import websockets

from airship_sim.config import default_config
from airship_sim.math3d import quat_normalize, quat_to_rotmat
from airship_sim.server import (RealtimeServer, ned_to_three,
                                quat_ned_to_three_xyzw)

# NED → Three.js 基变换矩阵(x_t=东, y_t=上, z_t=南),真旋转 det=+1
T_BASIS = np.array([[0.0, 1.0, 0.0],
                    [0.0, 0.0, -1.0],
                    [-1.0, 0.0, 0.0]])


def test_basis_is_proper_rotation():
    assert np.isclose(np.linalg.det(T_BASIS), 1.0)
    assert np.allclose(T_BASIS @ T_BASIS.T, np.eye(3))


def test_vector_conversion():
    # 北 1m → three (0,0,-1);天 1m(d=-1) → three (0,1,0)
    assert ned_to_three([1, 0, 0]) == [0.0, 0.0, -1.0]
    assert ned_to_three([0, 0, -1]) == [0.0, 1.0, 0.0]
    assert ned_to_three([0, 1, 0]) == [1.0, 0.0, 0.0]


def test_quat_conversion_matches_similarity_transform():
    # 四元数变换应满足 R_three = T·R_ned·Tᵀ(相似变换)
    rng = np.random.default_rng(3)
    for _ in range(30):
        q = quat_normalize(rng.normal(size=4))
        r_ned = quat_to_rotmat(q)
        x, y, z, w = quat_ned_to_three_xyzw(q)
        r_three = quat_to_rotmat(np.array([w, x, y, z]))
        assert np.allclose(r_three, T_BASIS @ r_ned @ T_BASIS.T, atol=1e-12)


REQUIRED_FIELDS = ["t_s", "running", "time_scale", "pos_ned_m", "quat_wxyz",
                   "pos_three", "quat_three_xyzw", "euler_deg", "v_body_m_s",
                   "omega_deg_s", "thrust_N", "cmd_N", "pid", "wind_ned_m_s",
                   "wind_three", "rho_kg_m3", "manual", "gains"]


async def _recv_state(ws, timeout=2.0):
    msg = json.loads(await asyncio.wait_for(ws.recv(), timeout))
    assert msg["type"] == "state"
    return msg


async def _drain_then_recv(ws):
    """丢弃积压消息,取最新状态。"""
    msg = None
    while True:
        try:
            msg = json.loads(await asyncio.wait_for(ws.recv(), 0.01))
        except asyncio.TimeoutError:
            if msg is not None:
                return msg
            msg = None
            await asyncio.sleep(0.05)


async def _integration():
    server = RealtimeServer(default_config(), host="127.0.0.1",
                            ws_port=0, http_port=0)   # 动态端口,避免并发冲突
    # 全实时(×1)下物理步进几乎占满事件循环,测试中降速让收发/定时器有空闲
    server.time_scale = 0.2
    task = asyncio.create_task(server.serve())
    try:
        await asyncio.wait_for(server.ready.wait(), 5)
        # HTTP 托管前端
        html = await asyncio.to_thread(
            lambda: urllib.request.urlopen(
                f"http://127.0.0.1:{server.http_port}/", timeout=3).read().decode())
        assert "three" in html and "quat_three_xyzw" in html

        async with websockets.connect(f"ws://127.0.0.1:{server.ws_port}/") as ws:
            # 1) 广播 schema 完整
            m = await _recv_state(ws)
            for f in REQUIRED_FIELDS:
                assert f in m, f"missing field {f}"
            assert len(m["pos_three"]) == 3 and len(m["quat_three_xyzw"]) == 4
            # 时间在推进
            m2 = await _recv_state(ws)
            assert m2["t_s"] >= m["t_s"]

            # 2) 暂停后时间冻结
            await ws.send(json.dumps({"type": "pause"}))
            await asyncio.sleep(0.1)
            a = await _drain_then_recv(ws)
            await asyncio.sleep(0.15)
            b = await _drain_then_recv(ws)
            assert a["running"] is False and b["t_s"] == a["t_s"]

            # 3) 单步推进一个控制周期(20ms)
            await ws.send(json.dumps({"type": "step"}))
            await asyncio.sleep(0.1)
            c = await _drain_then_recv(ws)
            assert np.isclose(c["t_s"] - b["t_s"], 0.02, atol=1e-6)

            # 4) 实时改增益(不重启生效)
            await ws.send(json.dumps({"type": "set_gains", "channel": "alt",
                                      "gains": {"kp": 0.42}}))
            await asyncio.sleep(0.1)
            d = await _drain_then_recv(ws)
            assert np.isclose(d["gains"]["alt"]["kp"], 0.42)
            assert d["gains"]["alt"]["ki"] > 0   # 未指定字段保持原值

            # 5) 手动模式 + 指令
            await ws.send(json.dumps({"type": "manual", "enable": True}))
            await ws.send(json.dumps({"type": "manual_cmd", "left_N": 0.03}))
            await asyncio.sleep(0.1)
            e = await _drain_then_recv(ws)
            assert e["manual"] is True
            # 6) 恢复运行;手动指令应真正作用到电机指令
            await ws.send(json.dumps({"type": "resume"}))
            await asyncio.sleep(0.25)
            f = await _drain_then_recv(ws)
            assert f["running"] is True and f["t_s"] > c["t_s"]
            assert np.isclose(f["cmd_N"][0], 0.03)

            # 7) 退出手动,下发高度设定值,应反映在 PID 通道设定中
            await ws.send(json.dumps({"type": "manual", "enable": False}))
            await ws.send(json.dumps({"type": "set_setpoints",
                                      "altitude_m": 2.5, "pos_hold": False,
                                      "yaw_deg": 45.0}))
            await asyncio.sleep(0.25)
            g = await _drain_then_recv(ws)
            assert g["manual"] is False
            assert np.isclose(g["pid"]["alt"]["sp"], 2.5)
            assert np.isclose(g["pid"]["yaw"]["sp"], 45.0, atol=1e-6)
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_server_integration():
    asyncio.run(asyncio.wait_for(_integration(), 60))
