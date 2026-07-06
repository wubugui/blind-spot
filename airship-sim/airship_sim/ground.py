"""地形与地面接触(可选模块;不挂地形时动力学路径零改动)。

地形约定:height_m(n, e) 返回该水平位置的地面海拔相对参考面 z=0 的高度,
即地面位于 NED z = -height_m 处。z=0 取放飞点地面(与 atmosphere.ground_alt_m
一致:该点的海拔 = ground_alt_m)。

接触模型(GroundContact):
  在艇体上取若干采样点(吊舱底 + 艇身前后底部)。某点侵入地面深度 pen 时,
  施加世界系竖直支持力  F_up = k·pen + c·v_down  (拉力钳位为 0),
  水平摩擦力  F_h = -μ·F_up · v_h/(|v_h|+ε)  (库仑摩擦的光滑化)。
  k、c 按总质量自动定标:静态下沉 5cm(k = m·g/0.05/N点),临界阻尼比 0.7。

[假设] 支持力恒竖直(忽略坡面法向倾斜) / 坡度 <15° 时误差 <4% / 陡坡碰撞
  (撞山)由上层按水平撞击速度判定,不依赖法向精度。
[假设] 点接触弹簧-阻尼,无翻滚/结构变形 / 起降与轻度触地量级正确
  / 硬撞毁伤由上层按撞击速度阈值裁决(见 Simulation.last_impact)。
[假设] 1ms RK4 下 k 的接触固有频率 ~2Hz,积分稳定(远离步长稳定边界)。
"""
from __future__ import annotations

import math

import numpy as np

from .config import GRAVITY_M_S2, SimConfig
from .math3d import cross3


class Terrain:
    """地形接口。height_m(n,e) → 地面高度(z=0 参考面以上,m)。"""

    def height_m(self, n_m: float, e_m: float) -> float:
        raise NotImplementedError

    def is_water(self, n_m: float, e_m: float) -> bool:
        """该点是否水面(上层用于迫降结局判定/渲染)。"""
        return False


class FlatTerrain(Terrain):
    def __init__(self, height_m: float = 0.0, water: bool = False):
        self._h = float(height_m)
        self._water = bool(water)

    def height_m(self, n_m: float, e_m: float) -> float:
        return self._h

    def is_water(self, n_m: float, e_m: float) -> bool:
        return self._water


class AnalyticTerrain(Terrain):
    """解析组合地形:基础坡 + 高斯山脊 + 正弦丘陵 + 水面(低于水位线)。

    与风场同源约定:冰川谷的 slope 下坡方向 = katabatic downhill 方位;
    山脊 axis = 风门方向。参数字典可 JSON 往返(游戏航段规格用)。

    参数(均可省略):
      base_m            基准高度
      slope             {"azimuth_deg": 下坡方位, "grade": 坡度(高差/水平距)}
                        高度随下坡方向线性下降(上坡方向升高)
      ridges            [{"center_n","center_e","axis_deg"(脊线走向),
                          "height_m","halfwidth_m"}] 高斯截面山脊
      hills             {"amp_m","wavelength_m","seed"} 双向正弦丘陵(确定性)
      water_level_m     低于该高度处为水面,高度钳位到水位(湖)
    """

    def __init__(self, spec: dict | None = None):
        s = spec or {}
        self._base = float(s.get("base_m", 0.0))
        slope = s.get("slope")
        if slope:
            az = math.radians(float(slope["azimuth_deg"]))
            g = float(slope["grade"])
            # 下坡方向单位矢量 d:沿 d 走高度下降 → 梯度 = -g·d
            self._slope_gn = -g * math.cos(az)
            self._slope_ge = -g * math.sin(az)
        else:
            self._slope_gn = self._slope_ge = 0.0
        self._ridges = []
        for r in s.get("ridges", []):
            ax = math.radians(float(r["axis_deg"]))
            self._ridges.append((
                float(r["center_n"]), float(r["center_e"]),
                # across = 垂直脊线的水平单位矢量
                -math.sin(ax), math.cos(ax),
                float(r["height_m"]), max(float(r["halfwidth_m"]), 1e-3)))
        h = s.get("hills")
        if h:
            self._hill_amp = float(h["amp_m"])
            self._hill_wl = max(float(h["wavelength_m"]), 1.0)
            seed = int(h.get("seed", 0))
            self._hill_ph1 = (seed % 97) * 0.1
            self._hill_ph2 = (seed % 89) * 0.13
        else:
            self._hill_amp = 0.0
            self._hill_wl = 1.0
            self._hill_ph1 = self._hill_ph2 = 0.0
        self._water = s.get("water_level_m")
        self._spec = s

    @property
    def spec(self) -> dict:
        return self._spec

    def height_m(self, n_m: float, e_m: float) -> float:
        h = self._base + self._slope_gn * n_m + self._slope_ge * e_m
        for cn, ce, axn, axe, hh, hw in self._ridges:
            xi = (n_m - cn) * axn + (e_m - ce) * axe
            h += hh * math.exp(-(xi / hw) ** 2)
        if self._hill_amp > 0.0:
            k = 2.0 * math.pi / self._hill_wl
            h += self._hill_amp * (math.sin(k * n_m + self._hill_ph1)
                                   * math.cos(k * e_m * 0.73 + self._hill_ph2))
        if self._water is not None and h < float(self._water):
            h = float(self._water)
        return h

    def is_water(self, n_m: float, e_m: float) -> bool:
        if self._water is None:
            return False
        # 原始地形低于水位处即水面(height_m 已钳位到水位)
        h = self._base + self._slope_gn * n_m + self._slope_ge * e_m
        for cn, ce, axn, axe, hh, hw in self._ridges:
            xi = (n_m - cn) * axn + (e_m - ce) * axe
            h += hh * math.exp(-(xi / hw) ** 2)
        if self._hill_amp > 0.0:
            k = 2.0 * math.pi / self._hill_wl
            h += self._hill_amp * (math.sin(k * n_m + self._hill_ph1)
                                   * math.cos(k * e_m * 0.73 + self._hill_ph2))
        return h < float(self._water)


class GroundContact:
    """地面接触力(挂在 AirshipDynamics 上;terrain=None 时不构造本对象)。"""

    MU_FRICTION = 0.6

    def __init__(self, cfg: SimConfig, terrain: Terrain, m_total_kg: float):
        self.terrain = terrain
        a, b = cfg.hull.a_m, cfg.hull.b_m
        z_gondola = cfg.actuator.motor_z_b_m + 0.1 * b   # 吊舱/起落架底
        # 采样点(体轴系):吊舱底、艇身前/后下缘
        self._pts_b = np.array([
            [0.0, 0.0, z_gondola],
            [0.6 * a, 0.0, 0.8 * b],
            [-0.6 * a, 0.0, 0.8 * b],
        ])
        n = len(self._pts_b)
        # 自动定标:静态下沉 5cm,阻尼比 0.7(见模块 docstring)
        self._k = m_total_kg * GRAVITY_M_S2 / 0.05 / n
        self._c = 2.0 * 0.7 * math.sqrt(self._k * m_total_kg / n)

    def forces(self, p_ned_m: np.ndarray, R: np.ndarray,
               nu: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """返回体轴系 (F_N, M_N·m)。无接触时为零(热点路径,先做廉价高度粗判)。"""
        f_b = np.zeros(3)
        m_b = np.zeros(3)
        # 粗判:参考点离地明显高于艇体尺度 → 必无接触
        h_ref = self.terrain.height_m(p_ned_m[0], p_ned_m[1])
        if -p_ned_m[2] - h_ref > 3.0 * max(abs(self._pts_b[:, 2].max()), 1.0):
            return f_b, m_b
        v_b, omega_b = nu[0:3], nu[3:6]
        for pt in self._pts_b:
            pw = p_ned_m + R @ pt
            h = self.terrain.height_m(pw[0], pw[1])
            pen = pw[2] - (-h)              # z 向下:pw[2] > -h 即入地
            if pen <= 0.0:
                continue
            vw = R @ (v_b + cross3(omega_b, pt))
            f_up = self._k * pen + self._c * max(vw[2], 0.0)
            if f_up <= 0.0:
                continue
            vh = np.hypot(vw[0], vw[1])
            fric = -self.MU_FRICTION * f_up / (vh + 0.1)
            fw = np.array([fric * vw[0], fric * vw[1], -f_up])
            fb = R.T @ fw
            f_b += fb
            m_b += cross3(pt, fb)
        return f_b, m_b

    def probe(self, p_ned_m: np.ndarray, R: np.ndarray,
              nu: np.ndarray) -> dict:
        """接触状态探针(上层判定着陆/坠毁/落水;不施力)。"""
        v_b, omega_b = nu[0:3], nu[3:6]
        max_pen = 0.0
        v_down = 0.0
        v_horiz = 0.0
        water = False
        for pt in self._pts_b:
            pw = p_ned_m + R @ pt
            h = self.terrain.height_m(float(pw[0]), float(pw[1]))
            pen = float(pw[2]) - (-h)
            if pen > max_pen:
                max_pen = pen
                vw = R @ (v_b + cross3(omega_b, pt))
                v_down = float(vw[2])
                v_horiz = float(np.hypot(vw[0], vw[1]))
                water = bool(self.terrain.is_water(float(pw[0]), float(pw[1])))
        return {"contact": bool(max_pen > 0.0), "pen_m": float(max_pen),
                "v_down_m_s": v_down, "v_horiz_m_s": v_horiz, "water": water}
