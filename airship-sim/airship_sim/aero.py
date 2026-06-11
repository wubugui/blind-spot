"""气动阻力与转动阻尼(体轴系,作用点/参考点 = 浮心)。

包含:
1. 轴向阻力:  Fx = -½ρ·Cd_ax·A_cross·|u_r|·u_r   (A_cross = πb², 横截面积)
2. 侧向阻力:  横流合速度 v_yz = √(v_r²+w_r²),
              F_yz = -½ρ·Cd_lat·A_side·|v_yz|·(v_r, w_r)   (A_side = πab, 侧投影面积)
3. 俯仰/偏航横流转动阻尼(条带理论推导):
   绕浮心以角速度 q 旋转时,纵向位置 x 处局部横流速度 = q·x,局部宽度 w(x)=2b√(1-x²/a²),
   dM = ½ρ·Cd_lat·w(x)·|qx|·qx·x dx,对 x∈[-a,a] 积分:
       M = -(4/15)·ρ·Cd_lat·b·a⁴·q|q|
   (∫₀^a √(1-x²/a²)·x³ dx = (2/15)a⁴)
4. 小线性角阻尼(粘性/蒙皮摩擦),防止纯二次阻尼在小角速度下欠阻尼。

[假设] 侧向阻力合力作用于浮心(忽略横流压心偏移产生的力矩)
/ 不稳定的 Munk 力矩已由附加质量 Coriolis 项 C_A(ν_r) 精确给出,此处不重复;
  压心偏移残差相对 Munk 力矩为小量
/ 安装大尾翼(显著改变压心)时需补充尾翼气动模型。

[假设] 平动阻力与转动阻尼解耦(平动用合速度、转动用纯旋转条带积分,交叉项忽略)
/ 误差在平动+旋转耦合剧烈时(快速转弯)~20% / 精细机动仿真需完整条带积分。
"""
from __future__ import annotations

import numpy as np

from .config import AeroConfig, FinConfig, HullConfig


class AeroModel:
    def __init__(self, hull: HullConfig, cfg: AeroConfig, fins: FinConfig | None = None):
        self._cd_axial = cfg.cd_axial
        self._cd_lateral = cfg.cd_lateral
        self._a_cross_m2 = hull.cross_area_m2
        self._a_side_m2 = hull.side_area_m2
        # 条带理论横流转动阻尼系数(乘 ρ·q|q| 即为力矩)
        self._c_ang_quad_m5 = (4.0 / 15.0) * cfg.cd_lateral * hull.b_m * hull.a_m**4
        self._c_lin_Nms = np.array([
            cfg.c_ang_lin_roll_Nms, cfg.c_ang_lin_pitch_Nms, cfg.c_ang_lin_yaw_Nms])
        self._fins = fins if (fins is not None and fins.enabled) else None

    def forces_moments(self, nu_r: np.ndarray, rho_kg_m3: float) -> tuple[np.ndarray, np.ndarray]:
        """输入气流相对速度 ν_r = [u,v,w,p,q,r](体轴),返回 (F_body_N, M_body_Nm)。"""
        u, v, w = nu_r[0:3]
        omega = nu_r[3:6]

        f = np.empty(3)
        f[0] = -0.5 * rho_kg_m3 * self._cd_axial * self._a_cross_m2 * abs(u) * u
        v_yz = np.hypot(v, w)
        c_lat = -0.5 * rho_kg_m3 * self._cd_lateral * self._a_side_m2 * v_yz
        f[1] = c_lat * v
        f[2] = c_lat * w

        m = -rho_kg_m3 * self._c_ang_quad_m5 * np.abs(omega) * omega
        m[0] = 0.0  # 横流条带阻尼对滚转无贡献(旋成体),滚转只有下面的线性项
        m -= self._c_lin_Nms * omega

        # 尾翼(见 FinConfig):线性升力线,局部来流角含转动分量(自然提供俯仰/偏航阻尼)
        if self._fins is not None:
            fc = self._fins
            x_f = fc.x_fin_m
            q_rate, r_rate = omega[1], omega[2]
            # 尾翼处局部横向气流速度:v_loc = v + r·x_f(偏航面), w_loc = w - q·x_f(俯仰面)
            v_loc = v + r_rate * x_f
            w_loc = w - q_rate * x_f
            c_fin = 0.5 * rho_kg_m3 * fc.cl_alpha_per_rad * fc.s_plane_m2 * abs(u)
            f_y_fin = -c_fin * v_loc   # 垂直尾翼对侧滑的恢复侧力
            f_z_fin = -c_fin * w_loc   # 水平尾翼对攻角的恢复法向力
            f[1] += f_y_fin
            f[2] += f_z_fin
            # 力矩 r×F,r = (x_f, 0, 0)
            m[1] += -x_f * f_z_fin
            m[2] += x_f * f_y_fin
        return f, m
