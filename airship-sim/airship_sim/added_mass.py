"""长椭球(prolate spheroid)的附加质量 — Lamb 系数。

对飞艇至关重要:排开空气质量(ρV ≈ 0.82 kg)与艇身总质量(≈0.83 kg)同量级,
附加质量不可省略。

来源公式(Lamb, Hydrodynamics, 1932, §114-115;亦见 Munk NACA TR-184):
长半轴 a(沿体轴 x),短半轴 b,离心率 e = sqrt(1 - b²/a²),令
    L = ln((1+e)/(1-e))
则两个形状因子为
    alpha0 = 2(1-e²)/e³ · (½L - e)            (轴向)
    beta0  = 1/e² - (1-e²)/(2e³) · L          (横向)
Lamb 惯性系数:
    k1 = alpha0 / (2 - alpha0)    轴向平动附加质量系数
    k2 = beta0  / (2 - beta0)     横向平动附加质量系数
    k' = e⁴(beta0-alpha0) / [(2-e²)·(2e² - (2-e²)(beta0-alpha0))]
                                  绕横轴转动附加惯量系数
附加质量/惯量:
    m_a_axial   = k1 · ρ V                  (体轴 x 平动)
    m_a_lateral = k2 · ρ V                  (体轴 y、z 平动)
    I_a_trans   = k' · I_fluid,  I_fluid = ρV(a²+b²)/5
                                  (绕 y、z 转动;I_fluid 为排开流体椭球绕横轴的转动惯量)
    I_a_roll    = 0   (理想流体中旋成体绕纵轴旋转不带动流体,势流解为零)

极限验证(单元测试用):
    a/b → 1(球):k1 = k2 = 0.5,k' = 0
    a/b → ∞(细长体):k1 → 0,k2 → 1,k' → 1

[假设] 囊体为精确长椭球且为刚体,用势流附加质量(无粘、无分离)
/ 低速(室内 <2 m/s, Re~1e5)下附加质量主要由势流决定,误差小
/ 大攻角流动分离或囊体柔性变形明显时失效。
"""
from __future__ import annotations

import numpy as np


def lamb_k_factors(a_m: float, b_m: float) -> tuple[float, float, float]:
    """返回 (k1, k2, k_prime)。要求 a > b(长椭球)。

    a/b 非常接近 1 时直接用球的解析极限,避免 e→0 的数值病态。
    """
    if a_m <= 0 or b_m <= 0:
        raise ValueError("semi-axes must be positive")
    if a_m < b_m:
        raise ValueError("expected prolate spheroid: a >= b")
    fineness = a_m / b_m
    if fineness < 1.0 + 1e-6:
        return 0.5, 0.5, 0.0

    e = np.sqrt(1.0 - (b_m / a_m) ** 2)
    L = np.log((1.0 + e) / (1.0 - e))
    alpha0 = 2.0 * (1.0 - e**2) / e**3 * (0.5 * L - e)
    beta0 = 1.0 / e**2 - (1.0 - e**2) / (2.0 * e**3) * L

    k1 = alpha0 / (2.0 - alpha0)
    k2 = beta0 / (2.0 - beta0)
    num = e**4 * (beta0 - alpha0)
    den = (2.0 - e**2) * (2.0 * e**2 - (2.0 - e**2) * (beta0 - alpha0))
    k_prime = num / den
    return float(k1), float(k2), float(k_prime)


def added_mass_matrix_unit_rho(a_m: float, b_m: float) -> np.ndarray:
    """单位空气密度下的 6×6 附加质量矩阵 M_A/ρ(对角,体轴系,参考点=浮心/体心)。

    实际使用时乘以当地空气密度: M_A = rho_kg_m3 * added_mass_matrix_unit_rho(a, b)。
    椭球几何中心 = 浮心,附加质量张量在此点为对角,这是把动力学参考点取在浮心的原因之一。
    顺序: [x平动, y平动, z平动, 绕x转动, 绕y转动, 绕z转动]
    """
    k1, k2, k_prime = lamb_k_factors(a_m, b_m)
    volume_m3 = 4.0 / 3.0 * np.pi * a_m * b_m**2
    i_fluid_unit_rho = volume_m3 * (a_m**2 + b_m**2) / 5.0
    return np.diag([
        k1 * volume_m3,
        k2 * volume_m3,
        k2 * volume_m3,
        0.0,                       # 绕纵轴:势流解为零
        k_prime * i_fluid_unit_rho,
        k_prime * i_fluid_unit_rho,
    ])
