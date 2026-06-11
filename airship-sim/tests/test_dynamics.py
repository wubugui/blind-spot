"""动力学守恒性、收敛性、风平衡、Munk 力矩。"""
import numpy as np

from airship_sim.atmosphere import make_atmosphere
from airship_sim.config import default_config
from airship_sim.dynamics import (IDX_OMEGA, IDX_QUAT, IDX_VEL,
                                  AirshipDynamics, coriolis_times_nu)
from airship_sim.math3d import quat_from_euler, quat_rotate_inv


def make_frictionless_dyn(wind=(0.0, 0.0, 0.0)):
    """无阻力配置(能量守恒测试用)。"""
    cfg = default_config(heaviness_kg=0.0)
    cfg.aero.cd_axial = 0.0
    cfg.aero.cd_lateral = 0.0
    cfg.aero.c_ang_lin_roll_Nms = 0.0
    cfg.aero.c_ang_lin_pitch_Nms = 0.0
    cfg.aero.c_ang_lin_yaw_Nms = 0.0
    cfg.fins.enabled = False   # 尾翼是耗散/恢复项,守恒性测试中关闭
    cfg.atmosphere.wind_ned_m_s = wind
    return AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))


def test_energy_conservation_pendulum():
    # 初始俯仰 20° 静止释放 → 摆动;无阻力/推力/风时机械能守恒(RK4 漂移应极小)
    dyn = make_frictionless_dyn()
    x = dyn.initial_state(q=quat_from_euler(0.0, np.deg2rad(20.0), 0.0))
    e0 = dyn.mechanical_energy_J(x)
    dt = 0.001
    energies = []
    omega_peak = 0.0
    for i in range(5000):   # 5 s,覆盖数个摆动周期
        x = dyn.step_rk4(x, np.zeros(3), i * dt, dt)
        energies.append(dyn.mechanical_energy_J(x))
        omega_peak = max(omega_peak, abs(x[IDX_OMEGA][1]))
    assert omega_peak > 0.1   # 确认确实在摆动(测试自检,角速度峰值 rad/s)
    drift = max(abs(e - e0) for e in energies)
    assert drift < 1e-9   # 总机械能逐步守恒(RK4 数值漂移应在机器精度量级)


def test_pendulum_actually_oscillates():
    # 摆动稳定性来自 CG 低于 CB 的建模,而非人为阻尼:俯仰角应来回过零
    dyn = make_frictionless_dyn()
    x = dyn.initial_state(q=quat_from_euler(0.0, np.deg2rad(15.0), 0.0))
    pitches = []
    dt = 0.001
    for i in range(6000):
        x = dyn.step_rk4(x, np.zeros(3), i * dt, dt)
        if i % 10 == 0:
            from airship_sim.math3d import quat_to_euler
            pitches.append(quat_to_euler(x[IDX_QUAT])[1])
    pitches = np.array(pitches)
    sign_changes = np.sum(np.diff(np.sign(pitches)) != 0)
    assert sign_changes >= 2   # 至少完整摆过去再摆回来
    assert np.max(np.abs(pitches)) < np.deg2rad(16.0)  # 无阻尼也不应发散


def test_rk4_convergence_order():
    # 全局误差 ~ O(dt⁴):dt 减半误差约降 16 倍(容差 8~30)
    def final_state(dt):
        dyn = make_frictionless_dyn()
        x = dyn.initial_state(q=quat_from_euler(0.0, np.deg2rad(20.0), 0.0))
        n = round(1.0 / dt)
        for i in range(n):
            x = dyn.step_rk4(x, np.zeros(3), i * dt, dt)
        return x

    ref = final_state(0.0005)
    e1 = np.linalg.norm(final_state(0.008) - ref)
    e2 = np.linalg.norm(final_state(0.004) - ref)
    ratio = e1 / e2
    assert 8.0 < ratio < 30.0


def test_constant_wind_equilibrium():
    # 与常值风同速漂移 = 平衡点:气流相对速度为零,所有加速度为零
    wind_ned = np.array([0.8, -0.3, 0.0])
    dyn = make_frictionless_dyn(wind=tuple(wind_ned))
    q = quat_from_euler(0.0, 0.0, np.deg2rad(30.0))
    x = dyn.initial_state(q=q)
    x[IDX_VEL] = quat_rotate_inv(q, wind_ned)
    dx = dyn.derivatives(x, np.zeros(3), 0.0)
    assert np.allclose(dx[IDX_VEL], 0.0, atol=1e-12)
    assert np.allclose(dx[IDX_OMEGA], 0.0, atol=1e-12)


def test_munk_moment_destabilizing():
    # 附加质量科氏项应给出 Munk 力矩:正攻角(u>0, w>0)→ 抬头力矩(失稳方向)
    cfg = default_config()
    dyn = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    rho = 1.225
    m_a = rho * dyn.props.M_A_unit_rho
    u, w = 0.5, 0.1
    nu_r = np.array([u, 0.0, w, 0.0, 0.0, 0.0])
    c_term = coriolis_times_nu(m_a, nu_r, nu_r)
    # 动力学方程中该项以 -C_A(ν_r)ν_r 进入 → 俯仰力矩 = -c_term[4]
    munk_pitch_Nm = -c_term[4]
    # 解析值:(k2-k1)·ρV·u·w
    from airship_sim.added_mass import lamb_k_factors
    k1, k2, _ = lamb_k_factors(cfg.hull.a_m, cfg.hull.b_m)
    expect = (k2 - k1) * rho * cfg.hull.volume_m3 * u * w
    assert munk_pitch_Nm > 0.0
    assert np.isclose(munk_pitch_Nm, expect, rtol=1e-9)


def test_fin_restoring_opposes_munk():
    """尾翼恢复力矩方向与 Munk 失稳力矩相反,且量值为部分抵消(真实飞艇的边际静稳定)。"""
    from airship_sim.aero import AeroModel
    from airship_sim.added_mass import lamb_k_factors
    cfg = default_config()
    rho, u, v = 1.225, 0.5, 0.1   # 正侧滑
    aero_fin = AeroModel(cfg.hull, cfg.aero, cfg.fins)
    cfg2 = default_config()
    cfg2.fins.enabled = False
    aero_bare = AeroModel(cfg2.hull, cfg2.aero, cfg2.fins)
    nu_r = np.array([u, v, 0.0, 0.0, 0.0, 0.0])
    _, m_fin = aero_fin.forces_moments(nu_r, rho)
    _, m_bare = aero_bare.forces_moments(nu_r, rho)
    fin_yaw_Nm = m_fin[2] - m_bare[2]
    k1, k2, _ = lamb_k_factors(cfg.hull.a_m, cfg.hull.b_m)
    munk_yaw_Nm = -(k2 - k1) * rho * cfg.hull.volume_m3 * u * v   # 偏航面 Munk(指向增大侧滑)
    assert fin_yaw_Nm * munk_yaw_Nm < 0           # 方向相反
    ratio = abs(fin_yaw_Nm / munk_yaw_Nm)
    assert 0.2 < ratio < 1.0                       # 部分抵消(完全抵消需不现实的尾翼面积)


def test_fin_adds_yaw_damping():
    # 有前向速度时,偏航角速度应受尾翼阻尼(局部来流角含 r·x_f 项)
    from airship_sim.aero import AeroModel
    cfg = default_config()
    aero = AeroModel(cfg.hull, cfg.aero, cfg.fins)
    nu_r = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.3])   # 前飞 + 正偏航角速度
    _, m = aero.forces_moments(nu_r, rho_kg_m3=1.225)
    cfg.fins.enabled = False
    aero_bare = AeroModel(cfg.hull, cfg.aero, cfg.fins)
    _, m_bare = aero_bare.forces_moments(nu_r, rho_kg_m3=1.225)
    assert m[2] < m_bare[2] < 0.0   # 尾翼显著增强偏航阻尼
