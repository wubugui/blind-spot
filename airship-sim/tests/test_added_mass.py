"""附加质量与 Lamb 解析解/文献表值对比。"""
import numpy as np

from airship_sim.added_mass import added_mass_matrix_unit_rho, lamb_k_factors


def test_sphere_limit():
    # 球:k1 = k2 = 0.5(经典结果:球的附加质量 = 排开流体质量的一半),k' = 0
    k1, k2, kp = lamb_k_factors(1.0, 1.0)
    assert np.isclose(k1, 0.5) and np.isclose(k2, 0.5) and kp == 0.0
    # 接近球(数值路径)
    k1, k2, kp = lamb_k_factors(1.01, 1.0)
    assert abs(k1 - 0.5) < 0.01 and abs(k2 - 0.5) < 0.01 and abs(kp) < 0.01


def test_slender_limit():
    # 细长体极限:k1 → 0,k2 → 1,k' → 1
    k1, k2, kp = lamb_k_factors(100.0, 1.0)
    assert k1 < 0.01
    assert k2 > 0.98
    assert kp > 0.95


def test_published_values_fineness_2():
    # Lamb(1932)表值,长细比 2:k1=0.209, k2=0.702, k'=0.240
    k1, k2, kp = lamb_k_factors(2.0, 1.0)
    assert np.isclose(k1, 0.209, atol=0.002)
    assert np.isclose(k2, 0.702, atol=0.003)
    assert np.isclose(kp, 0.240, atol=0.003)


def test_monotonic_in_fineness():
    f = np.array([1.5, 2.0, 3.0, 5.0, 8.0])
    ks = np.array([lamb_k_factors(fi, 1.0) for fi in f])
    assert np.all(np.diff(ks[:, 0]) < 0)   # k1 随长细比下降
    assert np.all(np.diff(ks[:, 1]) > 0)   # k2 上升
    assert np.all(np.diff(ks[:, 2]) > 0)   # k' 上升


def test_matrix_structure():
    a, b = 1.0, 0.4   # 默认飞艇几何
    m = added_mass_matrix_unit_rho(a, b)
    rho = 1.225
    v = 4.0 / 3.0 * np.pi * a * b**2
    k1, k2, kp = lamb_k_factors(a, b)
    assert m.shape == (6, 6)
    assert np.allclose(m, np.diag(np.diag(m)))           # 浮心参考点下为对角阵
    assert np.isclose(rho * m[0, 0], k1 * rho * v)
    assert np.isclose(m[1, 1], m[2, 2])                  # 横向对称
    assert m[3, 3] == 0.0                                # 旋成体滚转附加惯量为零
    assert np.isclose(m[4, 4], kp * v * (a**2 + b**2) / 5.0)
    # 量级合理性:横向附加质量与排开空气质量同量级(这正是不能省略的原因)
    assert 0.5 < rho * m[1, 1] / (rho * v) < 1.0
