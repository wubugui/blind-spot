"""四元数与三维旋转工具。

约定:
- 四元数 q = [w, x, y, z],单位范数,表示体轴系→世界系(NED)的旋转:
  v_world = R(q) @ v_body
- 欧拉角为航空 ZYX 顺序:yaw(ψ, 绕z/向下轴) → pitch(θ, 绕y) → roll(φ, 绕x),单位 rad
- 角速度 omega 为体轴系下的角速度 [p, q, r],单位 rad/s
"""
from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """归一化四元数。"""
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("zero quaternion")
    return q / n


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """四元数乘法 q1 ⊗ q2。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """四元数 → 旋转矩阵 R(体→世界), v_world = R @ v_body。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_rotate(q: np.ndarray, v_body: np.ndarray) -> np.ndarray:
    """用 q 旋转向量:体轴系 → 世界系。"""
    return quat_to_rotmat(q) @ v_body


def quat_rotate_inv(q: np.ndarray, v_world: np.ndarray) -> np.ndarray:
    """世界系 → 体轴系。"""
    return quat_to_rotmat(q).T @ v_world


def quat_derivative(q: np.ndarray, omega_body_rad_s: np.ndarray) -> np.ndarray:
    """姿态运动学: q̇ = ½ q ⊗ [0, ω_body]。"""
    omega_quat = np.array([0.0, *omega_body_rad_s])
    return 0.5 * quat_mul(q, omega_quat)


def quat_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """ZYX 欧拉角 → 四元数(体→世界)。"""
    cr, sr = np.cos(roll_rad / 2), np.sin(roll_rad / 2)
    cp, sp = np.cos(pitch_rad / 2), np.sin(pitch_rad / 2)
    cy, sy = np.cos(yaw_rad / 2), np.sin(yaw_rad / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """四元数 → (roll, pitch, yaw),ZYX 顺序,rad。pitch=±90° 处存在万向锁,按惯例截断。"""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def skew(v: np.ndarray) -> np.ndarray:
    """反对称矩阵 S(v),使 S(v) @ u = v × u。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def wrap_angle_rad(a: float) -> float:
    """把角度折叠到 (-π, π]。"""
    return float(np.arctan2(np.sin(a), np.cos(a)))
