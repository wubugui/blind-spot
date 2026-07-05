"""6DOF 刚体动力学(含附加质量),RK4 积分。

公式体系(Fossen, Handbook of Marine Craft Hydrodynamics, 2011, ch.3/6-8;
该体系是浮空器/水下航行器的标准建模方法,附加质量与刚体项统一处理):

- 动力学参考点 = 浮心 CB(椭球几何中心):附加质量张量在该点为对角阵。
- 广义速度 ν = [u,v,w,p,q,r](体轴系,CB 点线速度 + 角速度)。
- 刚体质量矩阵(重心偏移 r_g 用 Steiner 耦合项表达):
      M_RB = [[ m·I₃,      -m·S(r_g)],
              [ m·S(r_g),   I_cb    ]]
- 附加质量 M_A = ρ_air · diag(Lamb)(见 added_mass.py),随当地密度实时缩放。
- 科氏/向心矩阵取对称矩阵 M 的标准参数化(Fossen eq.3.46):
      C(ν) = [[0₃,     -S(M₁₁v₁+M₁₂v₂)],
              [-S(M₁₁v₁+M₁₂v₂), -S(M₂₁v₁+M₂₂v₂)]]
  注:C_A(ν_r) 的交叉项自动给出 Munk 失稳力矩 (k2-k1)·ρV·u·w 等,无需单独建模。
- 风(等效海流处理,Fossen ch.8):令体轴系风速 ν_c = [Rᵀ·w_ned; 0],
  气流相对速度 ν_r = ν - ν_c。动力学方程:
      M_RB·ν̇ + C_RB(ν)·ν + M_A·ν̇_r + C_A(ν_r)·ν_r = τ_grav + τ_buoy + τ_thrust + τ_aero(ν_r)
  其中世界系常值风满足 ν̇_c = [-ω×(Rᵀw_ned); 0],移项后对 ν̇ 求解。
  [假设] 计算 ν̇_c 时忽略风场自身的时间/空间变化率(∂w/∂t≈0)
  / 常值风下严格成立;湍流频率 <1Hz、幅值 <1m/s 时该项 ≪ 阻力项
  / 强阵风前沿(风速秒级突变)时低估瞬态载荷。

状态向量 x (16,):
  [0:3]   p_ned_m        位置(世界系 NED)
  [3:7]   q (w,x,y,z)    姿态四元数(体→世界)
  [7:10]  v_body_m_s     CB 点线速度(体轴系)
  [10:13] omega_rad_s    角速度(体轴系)
  [13:16] thrust_N       电机实际推力 [左, 右, 垂直](一阶执行器状态,随 RK4 一起积分)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .added_mass import added_mass_matrix_unit_rho
from .aero import AeroModel
from .atmosphere import Atmosphere
from .config import GRAVITY_M_S2, SimConfig
from .math3d import (cross3, quat_derivative, quat_normalize,
                     quat_to_rotmat, skew)

STATE_DIM = 16
IDX_POS = slice(0, 3)
IDX_QUAT = slice(3, 7)
IDX_VEL = slice(7, 10)
IDX_OMEGA = slice(10, 13)
IDX_THRUST = slice(13, 16)


@dataclass
class MassProperties:
    m_total_kg: float
    m_helium_kg: float
    r_g_b_m: np.ndarray          # 重心相对浮心位置(体轴系);默认配置下 ≈ (0,0,0.40)
    inertia_cb_kg_m2: np.ndarray  # 刚体惯量张量(关于浮心)
    M_RB: np.ndarray             # 6×6 刚体质量矩阵(关于浮心)
    M_A_unit_rho: np.ndarray     # 6×6 附加质量矩阵 / ρ


def build_mass_properties(cfg: SimConfig,
                          m_helium_kg: float | None = None) -> MassProperties:
    """m_helium_kg 缺省由配置密度×体积得出;氦气热力学开启时由外部传入实时值。"""
    hull, mass = cfg.hull, cfg.mass
    a, b = hull.a_m, hull.b_m
    v_m3 = hull.volume_m3
    m_env = mass.m_envelope_kg
    m_he = m_helium_kg if m_helium_kg is not None else mass.rho_helium_kg_m3 * v_m3
    m_gon = mass.m_gondola_kg
    m_total = m_env + m_he + m_gon
    r_gon = np.array(mass.r_gondola_b_m)

    # 重心(蒙皮与氦气质心在浮心,见 MassConfig 的 [假设])
    r_g = m_gon * r_gon / m_total

    # 惯量(关于浮心):
    # 薄椭球壳 ≈ 实心椭球公式 × 5/3(球的壳/实心比 (2/3)/(2/5),推广到椭球)
    i_env = np.diag([2.0 / 3.0 * m_env * b**2,
                     1.0 / 3.0 * m_env * (a**2 + b**2),
                     1.0 / 3.0 * m_env * (a**2 + b**2)])
    # 氦气:均匀实心椭球
    i_he = np.diag([2.0 / 5.0 * m_he * b**2,
                    1.0 / 5.0 * m_he * (a**2 + b**2),
                    1.0 / 5.0 * m_he * (a**2 + b**2)])
    # 吊舱:质点,平行轴定理
    i_gon = m_gon * (np.dot(r_gon, r_gon) * np.eye(3) - np.outer(r_gon, r_gon))
    i_cb = i_env + i_he + i_gon

    m_rb = np.zeros((6, 6))
    m_rb[0:3, 0:3] = m_total * np.eye(3)
    m_rb[0:3, 3:6] = -m_total * skew(r_g)
    m_rb[3:6, 0:3] = m_total * skew(r_g)
    m_rb[3:6, 3:6] = i_cb

    return MassProperties(
        m_total_kg=m_total,
        m_helium_kg=m_he,
        r_g_b_m=r_g,
        inertia_cb_kg_m2=i_cb,
        M_RB=m_rb,
        M_A_unit_rho=added_mass_matrix_unit_rho(a, b),
    )


def coriolis_times_nu(M: np.ndarray, nu: np.ndarray, nu_in: np.ndarray) -> np.ndarray:
    """计算 C(ν)·ν_in,C 由对称质量矩阵 M 与速度 ν 按 Fossen eq.3.46 构造。

    常规使用时 nu_in = nu;分开传参便于复用。
    """
    v1, v2 = nu[0:3], nu[3:6]
    s_a = skew(M[0:3, 0:3] @ v1 + M[0:3, 3:6] @ v2)
    s_b = skew(M[3:6, 0:3] @ v1 + M[3:6, 3:6] @ v2)
    out = np.empty(6)
    out[0:3] = -s_a @ nu_in[3:6]
    out[3:6] = -s_a @ nu_in[0:3] - s_b @ nu_in[3:6]
    return out


class AirshipDynamics:
    """连续动力学 f(x, u, t) 与 RK4 步进。无任何渲染/控制逻辑。"""

    def __init__(self, cfg: SimConfig, atmosphere: Atmosphere):
        self.cfg = cfg
        self.props = build_mass_properties(cfg)
        self.aero = AeroModel(cfg.hull, cfg.aero, cfg.fins)
        self.atmosphere = atmosphere
        act = cfg.actuator
        # 电机位置(体轴系,相对浮心):左、右、垂直
        self._r_motors = np.array([
            [0.0, -act.motor_sep_y_m, act.motor_z_b_m],
            [0.0, +act.motor_sep_y_m, act.motor_z_b_m],
            [0.0, 0.0, act.motor_z_b_m],
        ])
        self._thrust_min_N = -act.thrust_max_N if act.reversible else 0.0
        self._thrust_max_N = act.thrust_max_N
        self._tau_s = act.tau_s
        self._density_scale_thrust = act.density_scale_thrust
        self._thrust_ref_rho = act.thrust_ref_rho_kg_m3
        self._volume_m3 = cfg.hull.volume_m3
        self._minv_rho_key: float | None = None
        self._minv: np.ndarray | None = None

    def set_helium_mass(self, m_helium_kg: float) -> None:
        """氦气热力学(superheat 排气)实时更新质量属性。"""
        self.props = build_mass_properties(self.cfg, m_helium_kg=m_helium_kg)
        self._minv_rho_key = None   # 质量变化 → 缓存失效

    def _mass_matrix_inv(self, rho_kg_m3: float) -> np.ndarray:
        """(M_RB + ρ·M_A/ρ)⁻¹,按密度缓存。

        密度量化到 4 位有效数字(相对 1e-4):悬停高度变化引起的 ρ 微变不触发重算,
        引入的质量矩阵误差 ≤0.01%,远小于气动系数 ±30% 的模型不确定度;
        浮力等力项仍用精确 ρ。
        """
        key = float(f"{rho_kg_m3:.4e}")
        if key != self._minv_rho_key:
            self._minv_rho_key = key
            self._minv = np.linalg.inv(self.props.M_RB + key * self.props.M_A_unit_rho)
        return self._minv

    def initial_state(self, p_ned_m=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0)) -> np.ndarray:
        x = np.zeros(STATE_DIM)
        x[IDX_POS] = p_ned_m
        x[IDX_QUAT] = q
        return x

    def clamp_command(self, cmd_N: np.ndarray) -> np.ndarray:
        return np.clip(cmd_N, self._thrust_min_N, self._thrust_max_N)

    def derivatives(self, x: np.ndarray, cmd_N: np.ndarray, t_s: float) -> np.ndarray:
        p = x[IDX_POS]
        q = quat_normalize(x[IDX_QUAT])
        nu = np.concatenate([x[IDX_VEL], x[IDX_OMEGA]])
        thrust_N = x[IDX_THRUST]

        atm = self.atmosphere.get_state(p, t_s)
        rho = atm.rho_kg_m3
        R = quat_to_rotmat(q)
        pr = self.props

        # 风:世界系 → 体轴系,气流相对速度
        wind_b = R.T @ atm.wind_ned_m_s
        nu_c = np.concatenate([wind_b, np.zeros(3)])
        nu_r = nu - nu_c

        # ---- 广义外力(关于浮心,体轴系) ----
        tau = np.zeros(6)
        # 重力(作用在重心)
        f_grav_b = R.T @ np.array([0.0, 0.0, pr.m_total_kg * GRAVITY_M_S2])
        tau[0:3] += f_grav_b
        tau[3:6] += cross3(pr.r_g_b_m, f_grav_b)
        # 浮力(作用在浮心,当地密度实时计算)
        f_buoy_b = R.T @ np.array([0.0, 0.0, -rho * self._volume_m3 * GRAVITY_M_S2])
        tau[0:3] += f_buoy_b
        # 推力:左右电机沿 +x,垂直电机推力为正时沿 -z(向上)。
        # 螺旋桨推力随当地空气密度缩放 T∝ρ(见 ActuatorConfig):高原稀薄空气推力下降。
        thr = thrust_N
        if self._density_scale_thrust:
            thr = thrust_N * (rho / self._thrust_ref_rho)
        f_motors_b = np.array([
            [thr[0], 0.0, 0.0],
            [thr[1], 0.0, 0.0],
            [0.0, 0.0, -thr[2]],
        ])
        for r_m, f_m in zip(self._r_motors, f_motors_b):
            tau[0:3] += f_m
            tau[3:6] += cross3(r_m, f_m)
        # 气动阻力与转动阻尼(用气流相对速度)
        f_aero, m_aero = self.aero.forces_moments(nu_r, rho)
        tau[0:3] += f_aero
        tau[3:6] += m_aero

        # ---- 质量矩阵与科氏项 ----
        M_A = rho * pr.M_A_unit_rho
        rhs = tau - coriolis_times_nu(pr.M_RB, nu, nu) - coriolis_times_nu(M_A, nu_r, nu_r)
        # 常值世界风的 ν̇_c 修正项(见模块 docstring)
        dnu_c = np.concatenate([-cross3(nu[3:6], wind_b), np.zeros(3)])
        rhs += M_A @ dnu_c
        nu_dot = self._mass_matrix_inv(rho) @ rhs

        # ---- 运动学与执行器 ----
        dx = np.empty(STATE_DIM)
        dx[IDX_POS] = R @ nu[0:3]
        dx[IDX_QUAT] = quat_derivative(q, nu[3:6])
        dx[IDX_VEL] = nu_dot[0:3]
        dx[IDX_OMEGA] = nu_dot[3:6]
        # 电机一阶响应:τ·Ḟ = F_cmd - F(指令先经饱和限幅)
        dx[IDX_THRUST] = (self.clamp_command(cmd_N) - thrust_N) / self._tau_s
        return dx

    def step_rk4(self, x: np.ndarray, cmd_N: np.ndarray, t_s: float, dt_s: float) -> np.ndarray:
        """单步 RK4(指令在步内保持零阶保持,与 50Hz 控制/1ms 物理的时序一致)。"""
        k1 = self.derivatives(x, cmd_N, t_s)
        k2 = self.derivatives(x + 0.5 * dt_s * k1, cmd_N, t_s + 0.5 * dt_s)
        k3 = self.derivatives(x + 0.5 * dt_s * k2, cmd_N, t_s + 0.5 * dt_s)
        k4 = self.derivatives(x + dt_s * k3, cmd_N, t_s + dt_s)
        x_next = x + (dt_s / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        x_next[IDX_QUAT] = quat_normalize(x_next[IDX_QUAT])
        return x_next

    # ---- 能量(单元测试用:无阻力/无推力/无风时应守恒) ----
    def mechanical_energy_J(self, x: np.ndarray) -> float:
        """动能(含附加质量) + 重力势能 + 浮力势能。

        z 向下:重力 F_z=+mg → U_grav = -m·g·z_cg;浮力 F_z=-ρVg → U_buoy = +ρVg·z_cb。
        仅在常密度大气下守恒(无阻力/推力/风时,单元测试用)。
        """
        q = quat_normalize(x[IDX_QUAT])
        nu = np.concatenate([x[IDX_VEL], x[IDX_OMEGA]])
        atm = self.atmosphere.get_state(x[IDX_POS], 0.0)
        M = self.props.M_RB + atm.rho_kg_m3 * self.props.M_A_unit_rho
        ke = 0.5 * nu @ M @ nu
        R = quat_to_rotmat(q)
        z_cg = x[IDX_POS][2] + (R @ self.props.r_g_b_m)[2]
        pe_grav = -self.props.m_total_kg * GRAVITY_M_S2 * z_cg
        pe_buoy = atm.rho_kg_m3 * self._volume_m3 * GRAVITY_M_S2 * x[IDX_POS][2]
        return float(ke + pe_grav + pe_buoy)
