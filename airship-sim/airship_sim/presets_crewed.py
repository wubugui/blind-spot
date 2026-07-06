"""载人级飞艇参数集 + 按物理参数自动整定增益(回路整形)。

量级推导(设计文档同源):
  囊体 25m×8m 长椭球:V≈838 m³,海平面排开空气≈1026 kg,氦(含杂质
  0.18 kg/m³)≈151 kg → 毛浮力≈875 kg。蒙皮+尾翼≈160 kg;吊舱集总
  (结构/龙骨 250 + 飞行员 80 + 动力 + 电池 + 压舱/货)由配平得出≈720 kg。
  巡航 10 m/s 轴向阻力≈230 N → 双侧桨各 150 N(可换 300 N);
  20 kWh 电池 ≈ 5 h 巡航。

retune_gains() 用当前 hull/mass/actuator 的物理量按回路整形公式整定串级 PID
(与 config.py 手工整定同一方法,只是公式化),因此换囊体/电机/尾翼后调用
即可得到匹配的自动驾驶——这也是游戏"航电自动整定"的实现。

[假设] 增益整定按线性化单通道回路整形(ωn/ζ 选型见代码) / 与手工整定的
模型级参数集吻合(同方法) / 极端质量分布(重心远偏)时需重新选 ωn。
"""
from __future__ import annotations

import numpy as np

from .added_mass import lamb_k_factors
from .config import GRAVITY_M_S2, PidGains, SimConfig

RHO_REF_KG_M3 = 1.225


def retune_gains(cfg: SimConfig, heaviness_kg: float | None = None) -> SimConfig:
    """按当前机身物理量整定三通道 PID 与定点外环(回路整形,详见模块 docstring)。"""
    hull = cfg.hull
    a, b = hull.a_m, hull.b_m
    v_m3 = hull.volume_m3
    k1, k2, k_prime = lamb_k_factors(a, b)
    m_total = (cfg.mass.m_envelope_kg + cfg.mass.rho_helium_kg_m3 * v_m3
               + cfg.mass.m_gondola_kg)
    t_max = cfg.actuator.thrust_max_N
    tau_max = 2.0 * t_max * cfg.actuator.motor_sep_y_m

    # 高度通道:垂直等效质量;ωn=0.25 rad/s,ζ=1(气压计噪声下的稳妥带宽)
    m_eff_z = m_total + k2 * RHO_REF_KG_M3 * v_m3
    wn = 0.25
    cfg.control.alt = PidGains(
        kp=m_eff_z * wn * wn, ki=m_eff_z * wn * wn * 0.06,
        kd=2.0 * 1.0 * wn * m_eff_z * 0.6,      # 0.6 折衷:重滤波下微分打折
        tau_d_s=1.2, tau_meas_s=0.5, i_band=3.0 * b / 0.4 * 0.4,
        out_min=-t_max, out_max=t_max)

    # 偏航通道:等效惯量(刚体+附加);ωn 由可用力矩反推(0.5rad 误差时恰饱和),
    # 并限制在 [0.08, 0.5] rad/s
    i_fluid = RHO_REF_KG_M3 * v_m3 * (a * a + b * b) / 5.0
    # 刚体 I_zz 近似:蒙皮壳 + 氦 + 吊舱质点(与 build_mass_properties 同式,免循环依赖)
    m_env, m_he = cfg.mass.m_envelope_kg, cfg.mass.rho_helium_kg_m3 * v_m3
    m_gon = cfg.mass.m_gondola_kg
    r_gon = np.array(cfg.mass.r_gondola_b_m)
    i_zz = (m_env * (a * a + b * b) / 3.0 + m_he * (a * a + b * b) / 5.0
            + m_gon * (r_gon[0] ** 2 + r_gon[1] ** 2))
    i_eff = i_zz + k_prime * i_fluid
    wn_y = float(np.clip(np.sqrt(tau_max / (0.5 * i_eff)), 0.08, 0.5))
    cfg.control.yaw = PidGains(
        kp=i_eff * wn_y * wn_y, ki=i_eff * wn_y * wn_y * 0.05,
        kd=2.0 * 0.9 * wn_y * i_eff,
        tau_d_s=0.4, i_band=0.9, out_min=-tau_max, out_max=tau_max)

    # 前速通道:轴向等效质量,一阶带宽 0.35 rad/s
    m_eff_x = m_total + k1 * RHO_REF_KG_M3 * v_m3
    cfg.control.speed = PidGains(
        kp=m_eff_x * 0.35, ki=m_eff_x * 0.35 * 0.1, kd=0.0,
        out_min=-2.0 * t_max, out_max=2.0 * t_max)

    # 定点外环:速度上限 = 推力平衡速度的 60%;死区 ≈ 1.2 倍艇长(守半径)
    v_bal = np.sqrt(2.0 * t_max / (0.5 * RHO_REF_KG_M3
                                   * cfg.aero.cd_axial * hull.cross_area_m2))
    cfg.control.pos_hold_kp_1_s = 0.25
    cfg.control.pos_hold_max_speed_m_s = float(0.7 * v_bal)
    cfg.control.pos_hold_yaw_rate_max_rad_s = 0.2
    cfg.control.pos_hold_deadband_m = float(1.2 * hull.length_m)

    if heaviness_kg is not None:
        cfg.control.alt_ff_N = heaviness_kg * GRAVITY_M_S2
    return cfg


def crewed_config(heaviness_kg: float = 5.0) -> SimConfig:
    """载人级基准艇(小囊体+巡航桨+小尾翼),海平面配平为略重 heaviness_kg。"""
    cfg = SimConfig()
    cfg.hull.length_m = 25.0
    cfg.hull.diameter_m = 8.0
    cfg.mass.m_envelope_kg = 160.0          # 蒙皮 ≈513m²×250g/m² + 尾翼结构
    cfg.mass.rho_helium_kg_m3 = 0.18
    cfg.mass.r_gondola_b_m = (0.0, 0.0, 4.6)  # 吊舱贴囊体下缘(b=4m + 0.6m)
    # 尾翼/横距选型依据:侧风下 Munk 残余力矩须在小侧滑内被差速力矩覆盖,否则
    # 航向发散→横对风(实测 S=8/sep=3 在 6m/s 发散);且带前进速度大角度转弯时
    # 侧滑引起的 Munk 力矩会"broach"(甩艏),S=30 配合转弯协调(转弯先收油)
    # 后 90° 转弯干净收敛。全静稳定需 S≈45m²(大型尾翼组,升级件)。
    cfg.fins.s_plane_m2 = 30.0
    cfg.fins.x_fin_m = -10.0
    act = cfg.actuator
    act.thrust_max_N = 150.0
    act.tau_s = 0.5                          # 大桨响应慢
    act.motor_sep_y_m = 4.0
    act.motor_z_b_m = 4.6
    act.density_scale_thrust = True
    cfg.energy.enabled = True
    cfg.energy.capacity_Wh = 20000.0
    cfg.energy.battery_mass_kg = 100.0
    cfg.energy.prop_disc_area_m2 = 3.14
    # 海平面配平:吊舱 = 排开空气 + heaviness − 蒙皮 − 氦
    v = cfg.hull.volume_m3
    cfg.mass.m_gondola_kg = (RHO_REF_KG_M3 * v + heaviness_kg
                             - cfg.mass.m_envelope_kg
                             - cfg.mass.rho_helium_kg_m3 * v)
    retune_gains(cfg, heaviness_kg)
    return cfg
