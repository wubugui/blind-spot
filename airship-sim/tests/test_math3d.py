import numpy as np
import pytest

from airship_sim.math3d import (quat_conj, quat_from_euler, quat_mul,
                                quat_normalize, quat_rotate, quat_to_euler,
                                quat_to_rotmat, skew, wrap_angle_rad)


def test_quat_unit_and_inverse():
    q = quat_normalize(np.array([0.3, -0.5, 0.7, 0.2]))
    assert np.isclose(np.linalg.norm(q), 1.0)
    ident = quat_mul(q, quat_conj(q))
    assert np.allclose(ident, [1, 0, 0, 0], atol=1e-12)


def test_rotation_matches_rotmat():
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = quat_normalize(rng.normal(size=4))
        v = rng.normal(size=3)
        assert np.allclose(quat_rotate(q, v), quat_to_rotmat(q) @ v, atol=1e-12)
        # 旋转矩阵正交且行列式为 +1
        R = quat_to_rotmat(q)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(R), 1.0)


def test_euler_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        roll = rng.uniform(-np.pi, np.pi)
        pitch = rng.uniform(-np.pi / 2 + 0.01, np.pi / 2 - 0.01)
        yaw = rng.uniform(-np.pi, np.pi)
        q = quat_from_euler(roll, pitch, yaw)
        r2, p2, y2 = quat_to_euler(q)
        assert np.allclose([roll, pitch, yaw], [r2, p2, y2], atol=1e-10)


def test_yaw_rotation_direction():
    # 偏航 +90°(NED 下 z 朝下,正偏航 = 北转向东):体轴 x(前) → 世界东
    q = quat_from_euler(0.0, 0.0, np.pi / 2)
    v_world = quat_rotate(q, np.array([1.0, 0.0, 0.0]))
    assert np.allclose(v_world, [0.0, 1.0, 0.0], atol=1e-12)


def test_skew_cross():
    a, b = np.array([1.0, 2.0, 3.0]), np.array([-4.0, 0.5, 2.0])
    assert np.allclose(skew(a) @ b, np.cross(a, b))


@pytest.mark.parametrize("a", [0.0, np.pi, 3 * np.pi / 2, -3 * np.pi, 7.0, -100.0])
def test_wrap_angle(a):
    w = wrap_angle_rad(a)
    # 与原角模 2π 等价,且落在 [-π, π](±π 边界由浮点舍入决定,两端都接受)
    assert np.isclose(np.cos(w), np.cos(a), atol=1e-9)
    assert np.isclose(np.sin(w), np.sin(a), atol=1e-9)
    assert -np.pi - 1e-12 <= w <= np.pi + 1e-12
