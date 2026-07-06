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


# ================================================================
# WorldTerrain:烘焙式程序化景观(游戏用;物理与渲染同源)
# ================================================================
def _np_vnoise(X, Y, seed):
    """向量化值噪声(numpy),哈希与 AnalyticTerrain 同族,确定性。"""
    xi = np.floor(X); yi = np.floor(Y)
    xf = X - xi; yf = Y - yi

    def h(a, b):
        n = np.sin(a * 127.1 + b * 311.7 + seed * 74.7) * 43758.5453
        return n - np.floor(n)

    sx = xf * xf * (3.0 - 2.0 * xf)
    sy = yf * yf * (3.0 - 2.0 * yf)
    return ((h(xi, yi) * (1 - sx) + h(xi + 1, yi) * sx) * (1 - sy)
            + (h(xi, yi + 1) * (1 - sx) + h(xi + 1, yi + 1) * sx) * sy)


def _np_fbm(X, Y, octaves, seed, ridged=False):
    v = np.zeros_like(X); amp = 1.0; f = 1.0; tot = 0.0
    for o in range(octaves):
        n = _np_vnoise(X * f, Y * f, seed + o * 13)
        if ridged:
            n = 1.0 - np.abs(2.0 * n - 1.0)
            n = n * n
        v += amp * n; tot += amp
        amp *= 0.52; f *= 2.05
    return v / tot


class WorldTerrain(Terrain):
    """程序化景观地形(烘焙网格 + O(1) 双线性查询)。

    组成:
      基底坡(沿 leg 下坡向) + 域扭曲脊状分形(山脉真实起伏)
      + 航线走廊阻尼(走廊内起伏受控可飞,走廊外放大成高耸雪山)
      + 山脊 bump(风门,与风场同源) + 河流(源点顺坡追踪、刻蚀河谷、
        水面沿程单调下降,汇入湖/海) + 湖/海水面。

    [假设] 地形烘焙为 grid_n×grid_n 网格后双线性插值(默认 512²,~16m 格距)
    / 物理接触与渲染完全同源;查询 O(1),比解析式逐项求和更快
    / 需要 <16m 的微地形(沟坎)时提高分辨率。
    [假设] 河流由"raw 场贪婪下坡走"生成 / 走向天然沿谷线、终于水体或图边,
    水系自洽 / 不模拟流量/侵蚀动力学。
    """

    HALF_EXTENT_FACTOR = 1.15   # 烘焙范围相对航段包围盒的放大

    def __init__(self, spec: dict):
        self.spec = spec or {}
        s = self.spec
        self._seed = int(s.get("seed", 0))
        self._biome = s.get("biome", "plains")
        route = s.get("route", {"start_n": 0.0, "start_e": 0.0,
                                "dest_n": 1000.0, "dest_e": 0.0})
        self._a = np.array([float(route["start_n"]), float(route["start_e"])])
        self._b = np.array([float(route["dest_n"]), float(route["dest_e"])])
        c = 0.5 * (self._a + self._b)
        half = max(float(np.linalg.norm(self._b - self._a)) * 0.75, 1600.0) \
            * self.HALF_EXTENT_FACTOR
        self._c = c; self._half = half
        n_grid = int(s.get("grid_n", 512))
        self._n = n_grid
        self._cell = 2.0 * half / (n_grid - 1)
        self._bake()

    # ---- 烘焙 ----
    def _bake(self) -> None:
        s = self.spec
        n = self._n
        axis = np.linspace(-self._half, self._half, n)
        N, E = np.meshgrid(axis + self._c[0], axis + self._c[1], indexing="ij")

        # 基底坡(与风场同源:slope 键与 AnalyticTerrain 一致)
        H = np.zeros_like(N)
        slope = s.get("slope")
        if slope:
            az = math.radians(float(slope["azimuth_deg"]))
            g = float(slope["grade"])
            H += -g * (np.cos(az) * N + np.sin(az) * E)
        H += float(s.get("base_m", 0.0))

        # 走廊阻尼因子:到航线线段的距离
        ab = self._b - self._a
        L2 = float(ab @ ab) + 1e-9
        t = np.clip(((N - self._a[0]) * ab[0] + (E - self._a[1]) * ab[1]) / L2, 0, 1)
        dn = N - (self._a[0] + t * ab[0]); de = E - (self._a[1] + t * ab[1])
        d_corr = np.sqrt(dn * dn + de * de)
        f_far = np.clip((d_corr - 700.0) / 2600.0, 0.0, 1.0)
        f_far = f_far * f_far * (3 - 2 * f_far)

        # 群系起伏参数(近走廊 amp_near 保证可飞,远处 amp_far 造雪山)
        B = self._biome
        prm = {
            "plains":       (18.0, 130.0, 0.25, 3),
            "lake":         (8.0,  90.0,  0.2,  3),
            "lake_open":    (4.0,  60.0,  0.15, 3),
            "foothill":     (45.0, 650.0, 0.55, 4),
            "ridge":        (55.0, 1250.0, 0.75, 4),
            "ridge_high":   (60.0, 1500.0, 0.8,  4),
            "plateau_climb": (40.0, 900.0, 0.6,  4),
            "plateau":      (30.0, 750.0, 0.5,  4),
            "glacier_app":  (45.0, 1500.0, 0.7,  4),
            "glacier":      (50.0, 1800.0, 0.75, 4),
        }.get(B, (20.0, 150.0, 0.3, 3))
        amp_near, amp_far, ridge_mix, octs = prm

        # 域扭曲脊状分形
        scale = 1.0 / 2600.0
        WX = _np_fbm(N * scale * 0.6, E * scale * 0.6, 2, self._seed + 91) * 2.2
        WY = _np_fbm(N * scale * 0.6 + 5.2, E * scale * 0.6 + 1.3, 2, self._seed + 57) * 2.2
        base_n = _np_fbm(N * scale + WX, E * scale + WY, octs, self._seed)
        ridge_n = _np_fbm(N * scale * 1.4 + WY, E * scale * 1.4 + WX, octs,
                          self._seed + 7, ridged=True)
        relief = (1 - ridge_mix) * (base_n - 0.5) * 2.0 + ridge_mix * (ridge_n - 0.35) * 2.2
        amp = amp_near + (amp_far - amp_near) * f_far
        H += relief * amp

        # 山脊 bump(风门,与风场共用参数)
        for r in s.get("ridges", []):
            axr = math.radians(float(r["axis_deg"]))
            xi = (N - float(r["center_n"])) * (-math.sin(axr)) \
               + (E - float(r["center_e"])) * math.cos(axr)
            H += float(r["height_m"]) * np.exp(-(xi / float(r["halfwidth_m"])) ** 2)

        # 热侵蚀式平滑(去尖刺,山形更真实)
        for _ in range(2):
            Hs = H.copy()
            Hs[1:-1, 1:-1] = (H[1:-1, 1:-1] * 0.6
                              + 0.1 * (H[:-2, 1:-1] + H[2:, 1:-1]
                                       + H[1:-1, :-2] + H[1:-1, 2:]))
            H = Hs

        # ---- 水系 ----
        WL = np.full_like(H, np.nan)         # 每格水面高度(NaN=无水)
        water_level = s.get("water_level_m")
        if water_level is not None:
            wl = float(water_level)
            H = np.maximum(H, np.where(H < wl, H, H))   # 保留湖床
            mask = H < wl
            WL[mask] = wl
        # 河流:上游源点顺坡追踪 → 刻蚀 + 沿程水面
        n_riv = {"plains": 2, "lake": 2, "foothill": 2, "ridge": 1, "ridge_high": 1,
                 "plateau_climb": 1, "plateau": 1, "glacier_app": 1, "glacier": 1,
                 "lake_open": 0}.get(B, 1)
        rng = np.random.default_rng(self._seed + 1234)
        for k in range(n_riv):
            self._trace_river(H, WL, rng)

        # 起降场坪:航段两端各平整一块(驿站),去水
        for pt in (self._a, self._b):
            xi = (pt[0] - (self._c[0] - self._half)) / self._cell
            yi = (pt[1] - (self._c[1] - self._half)) / self._cell
            i0, j0 = int(round(xi)), int(round(yi))
            if 0 <= i0 < n and 0 <= j0 < n:
                pad = int(170.0 / self._cell)
                lo_i, hi_i = max(0, i0 - pad), min(n, i0 + pad + 1)
                lo_j, hi_j = max(0, j0 - pad), min(n, j0 + pad + 1)
                ii, jj = np.meshgrid(np.arange(lo_i, hi_i), np.arange(lo_j, hi_j),
                                     indexing="ij")
                d = np.sqrt(((ii - i0) ** 2 + (jj - j0) ** 2)) * self._cell
                w = np.clip(1.0 - d / (pad * self._cell), 0.0, 1.0)
                w = w * w * (3 - 2 * w)
                h0 = H[i0, j0]
                H[lo_i:hi_i, lo_j:hi_j] = (H[lo_i:hi_i, lo_j:hi_j] * (1 - w) + h0 * w)
                WL[lo_i:hi_i, lo_j:hi_j] = np.where(w > 0.35, np.nan,
                                                    WL[lo_i:hi_i, lo_j:hi_j])
        self._H = H
        self._WL = WL

    def _trace_river(self, H, WL, rng) -> None:
        n = self._n
        # 源点:随机取高处样本中的最高者
        cand = rng.integers(6, n - 6, size=(40, 2))
        heights = H[cand[:, 0], cand[:, 1]]
        i, j = cand[int(np.argmax(heights))]
        path = []
        for _ in range(n * 2):
            path.append((i, j))
            if not np.isnan(WL[i, j]):
                break                          # 汇入湖/海
            # 8 邻域最陡下降(带轻微噪声防直线)
            best = None; bh = H[i, j]
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ii, jj = i + di, j + dj
                    if 0 <= ii < n and 0 <= jj < n:
                        hh = H[ii, jj] + rng.normal(0, 0.05)
                        if hh < bh:
                            bh = hh; best = (ii, jj)
            if best is None:
                break                          # 洼地(小湖)
            i, j = best
            if i < 2 or j < 2 or i > n - 3 or j > n - 3:
                break                          # 出图
        if len(path) < 12:
            return
        # 刻蚀 + 沿程水面(单调不升)
        depth, width_cells = 7.0, max(2, int(28.0 / self._cell))
        lvl = None
        for (i, j) in path:
            bed = H[i, j] - depth
            lo_i, hi_i = max(0, i - width_cells), min(n, i + width_cells + 1)
            lo_j, hi_j = max(0, j - width_cells), min(n, j + width_cells + 1)
            sub = H[lo_i:hi_i, lo_j:hi_j]
            ii, jj = np.meshgrid(np.arange(lo_i, hi_i), np.arange(lo_j, hi_j),
                                 indexing="ij")
            d2 = ((ii - i) ** 2 + (jj - j) ** 2) * (self._cell ** 2)
            carve = depth * np.exp(-d2 / (2 * (width_cells * self._cell * 0.55) ** 2))
            H[lo_i:hi_i, lo_j:hi_j] = np.minimum(sub, np.maximum(bed, sub - carve))
            w = bed + 2.2
            lvl = w if lvl is None else min(lvl, w)     # 水面沿程单调下降
            riv_w = max(1, int(width_cells * 0.45))
            for di in range(-riv_w, riv_w + 1):
                for dj in range(-riv_w, riv_w + 1):
                    ii2, jj2 = i + di, j + dj
                    if 0 <= ii2 < n and 0 <= jj2 < n and H[ii2, jj2] < lvl:
                        if np.isnan(WL[ii2, jj2]) or WL[ii2, jj2] < lvl:
                            WL[ii2, jj2] = lvl

    # ---- 查询(物理热点:O(1) 双线性) ----
    def _idx(self, n_m: float, e_m: float):
        x = (n_m - (self._c[0] - self._half)) / self._cell
        y = (e_m - (self._c[1] - self._half)) / self._cell
        x = min(max(x, 0.0), self._n - 1.001)
        y = min(max(y, 0.0), self._n - 1.001)
        return x, y

    def height_m(self, n_m: float, e_m: float) -> float:
        x, y = self._idx(float(n_m), float(e_m))
        i, j = int(x), int(y)
        fx, fy = x - i, y - j
        H = self._H
        return float(H[i, j] * (1 - fx) * (1 - fy) + H[i + 1, j] * fx * (1 - fy)
                     + H[i, j + 1] * (1 - fx) * fy + H[i + 1, j + 1] * fx * fy)

    def water_h(self, n_m: float, e_m: float) -> float | None:
        x, y = self._idx(float(n_m), float(e_m))
        w = self._WL[int(round(x)), int(round(y))]
        return None if np.isnan(w) else float(w)

    def is_water(self, n_m: float, e_m: float) -> bool:
        return self.water_h(n_m, e_m) is not None

    # ---- 导出(渲染) ----
    def export_grids(self, near_res: int = 160, far_res: int = 96,
                     far_half_m: float = 22000.0) -> dict:
        """近景精细网格(高度/水面/植被密度) + 远景粗网格(雪山)。"""
        n = self._n
        idx = np.linspace(0, n - 1, near_res).astype(int)
        Hn = self._H[np.ix_(idx, idx)]
        Wn = self._WL[np.ix_(idx, idx)]
        # 植被密度(生态规则):噪声林斑 × 坡度惩罚 × 海拔(林线) × 水源加成
        axis = np.linspace(-self._half, self._half, near_res)
        N, E = np.meshgrid(axis + self._c[0], axis + self._c[1], indexing="ij")
        forest = _np_fbm(N / 900.0, E / 900.0, 3, self._seed + 400)
        gy, gx = np.gradient(Hn, 2 * self._half / (near_res - 1))
        slope = np.hypot(gx, gy)
        hmin, hmax = float(Hn.min()), float(Hn.max())
        span = max(hmax - hmin, 1.0)
        treeline = hmin + span * {"plains": 1.2, "lake": 1.2, "lake_open": 1.2,
                                  "foothill": 0.75, "ridge": 0.5, "ridge_high": 0.4,
                                  "plateau_climb": 0.55, "plateau": 0.6,
                                  "glacier_app": 0.25, "glacier": 0.15}.get(self._biome, 0.8)
        dens = np.clip((forest - 0.42) * 2.6, 0, 1)
        dens *= np.clip(1.0 - slope / 0.5, 0, 1)
        dens *= np.clip((treeline - Hn) / (span * 0.18), 0, 1)
        near_water = ~np.isnan(Wn)
        # 水源加成:靠水 3 格内密度提升
        wsoft = near_water.astype(float)
        for _ in range(3):
            wsoft[1:-1, 1:-1] = np.maximum(
                wsoft[1:-1, 1:-1],
                0.75 * np.maximum.reduce([wsoft[:-2, 1:-1], wsoft[2:, 1:-1],
                                          wsoft[1:-1, :-2], wsoft[1:-1, 2:]]))
        dens = np.clip(dens + wsoft * 0.35 * (dens > 0.02), 0, 1)
        dens[near_water] = 0.0

        # 远景网格(同一函数,含走廊阻尼→远处才高耸)
        fx = np.linspace(-far_half_m, far_half_m, far_res)
        FN, FE = np.meshgrid(fx + self._c[0], fx + self._c[1], indexing="ij")
        FH = self._far_height(FN, FE)

        rnd2 = lambda A: [[round(float(v), 1) for v in row] for row in A]
        wl_out = [[(round(float(v), 1) if not np.isnan(v) else None) for v in row]
                  for row in Wn]
        return {"center_n": float(self._c[0]), "center_e": float(self._c[1]),
                "half_n": self._half, "half_e": self._half, "res": near_res,
                "heights": rnd2(Hn), "water_h": wl_out,
                "flora": [[round(float(v), 2) for v in row] for row in dens],
                "far": {"half_m": far_half_m, "res": far_res, "heights": rnd2(FH)},
                "start": [float(self._a[0]), float(self._a[1])],
                "dest": [float(self._b[0]), float(self._b[1])]}

    def _far_height(self, N, E):
        """远景:与烘焙同参数的解析计算(粗网格一次性,无需精度)。"""
        s = self.spec
        H = np.zeros_like(N)
        slope = s.get("slope")
        if slope:
            az = math.radians(float(slope["azimuth_deg"]))
            H += -float(slope["grade"]) * (np.cos(az) * N + np.sin(az) * E)
        H += float(s.get("base_m", 0.0))
        ab = self._b - self._a
        L2 = float(ab @ ab) + 1e-9
        t = np.clip(((N - self._a[0]) * ab[0] + (E - self._a[1]) * ab[1]) / L2, 0, 1)
        dn = N - (self._a[0] + t * ab[0]); de = E - (self._a[1] + t * ab[1])
        f_far = np.clip((np.sqrt(dn * dn + de * de) - 700.0) / 2600.0, 0, 1)
        f_far = f_far * f_far * (3 - 2 * f_far)
        prm = {
            "plains": (18.0, 130.0, 0.25, 3), "lake": (8.0, 90.0, 0.2, 3),
            "lake_open": (4.0, 60.0, 0.15, 3), "foothill": (45.0, 650.0, 0.55, 4),
            "ridge": (55.0, 1250.0, 0.75, 4), "ridge_high": (60.0, 1500.0, 0.8, 4),
            "plateau_climb": (40.0, 900.0, 0.6, 4), "plateau": (30.0, 750.0, 0.5, 4),
            "glacier_app": (45.0, 1500.0, 0.7, 4), "glacier": (50.0, 1800.0, 0.75, 4),
        }.get(self._biome, (20.0, 150.0, 0.3, 3))
        amp_near, amp_far, ridge_mix, octs = prm
        scale = 1.0 / 2600.0
        WX = _np_fbm(N * scale * 0.6, E * scale * 0.6, 2, self._seed + 91) * 2.2
        WY = _np_fbm(N * scale * 0.6 + 5.2, E * scale * 0.6 + 1.3, 2, self._seed + 57) * 2.2
        base_n = _np_fbm(N * scale + WX, E * scale + WY, octs, self._seed)
        ridge_n = _np_fbm(N * scale * 1.4 + WY, E * scale * 1.4 + WX, octs,
                          self._seed + 7, ridged=True)
        relief = (1 - ridge_mix) * (base_n - 0.5) * 2.0 + ridge_mix * (ridge_n - 0.35) * 2.2
        return H + relief * (amp_near + (amp_far - amp_near) * f_far)
