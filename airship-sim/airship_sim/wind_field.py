"""环境风场 — 空间结构化的平均风(位置/时间的函数)。

物理引擎只通过 Atmosphere.get_state(position_ned_m, time_s) 取风,风本就是
位置与时间的函数,因此可在此加入空间结构化风场,动力学代码零改动。

面向低空(2~50m)工程验证的解析/运动学风场:
  UniformWind        常值风(调试/对照)
  LogLawWind         中性边界层对数风廓线,粗糙度 z0 随下垫面
  PrandtlSlopeWind   冰川/坡地下降风(katabatic)/上坡风(anabatic),含沿坡 fetch 增强
  TabulatedProfileWind 高度-风廓线查表(接入实测探空/塔站/再分析数据)
  LakeBreezeFront    湖陆风 + 内陆推进锋面(辐合+上升)
  ThermalField       对流热泡阵列(上升/补偿下沉,含垂直分量)
  RidgeTerrain       山脊加速 + 背风回流(对基础风的地形修正,装饰器)
  CompositeWind      分量线性叠加

坐标:NED;水平风矢量为"去向"(0=吹向北 +x,90=吹向东 +y),垂直分量 D 向下
(上升气流 D<0)。高度用离地高度 AGL(= −z,z=0 为放飞点地面)。

[假设] 各分量线性叠加、互不作用 / 弱风缓坡下合理 / 强耦合时需数值风场(RANS/LES 或实测)。
所有模型均为解析/运动学参数化,便于快速验证 case;定量任务需用实测标定参数。
"""
from __future__ import annotations

import math

import numpy as np

_KARMAN = 0.4
_PRANDTL_PEAK = math.exp(-math.pi / 4.0) * math.sin(math.pi / 4.0)  # ≈0.32239,Prandtl 廓线峰值


def _azimuth_unit(dir_to_deg: float) -> np.ndarray:
    """去向方位角 → NED 水平单位矢量 (N=+x, E=+y)。"""
    a = math.radians(dir_to_deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


class WindField:
    """环境风场接口:给定位置/离地高度/时间,返回平均风矢量(NED, m/s)。"""

    def wind_ned(self, pos_ned_m: np.ndarray, height_agl_m: float,
                 time_s: float) -> np.ndarray:
        raise NotImplementedError


class UniformWind(WindField):
    """常值均匀风(去向方位角 + 风速)。"""

    def __init__(self, speed_m_s: float, dir_to_deg: float):
        self._w = float(speed_m_s) * _azimuth_unit(dir_to_deg)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        return self._w.copy()


class LogLawWind(WindField):
    """中性层结对数边界层:u(z) = u_ref · ln(z/z0) / ln(z_ref/z0),方向常值。

    z0 粗糙长度(m):雪/冰 ~2e-4,水面 ~1e-3,草地 ~0.03,灌木/碎石 ~0.1。
    [假设] 中性层结、平坦均匀下垫面、定常 / 近地面 10~100m 内良好
    / 强稳定/强对流时应叠加坡风/热泡分量。
    """

    def __init__(self, u_ref_m_s: float, dir_to_deg: float,
                 z0_m: float = 1e-3, z_ref_m: float = 10.0):
        self._z0 = max(float(z0_m), 1e-6)
        self._zref = max(float(z_ref_m), self._z0 * 2.0)
        self._uref = float(u_ref_m_s)
        self._dir = _azimuth_unit(dir_to_deg)
        self._denom = math.log(self._zref / self._z0)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        z = max(float(height_agl_m), self._z0)
        u = self._uref * math.log(z / self._z0) / self._denom
        return max(u, 0.0) * self._dir

    def u_star_m_s(self) -> float:
        return _KARMAN * self._uref / self._denom


class PrandtlSlopeWind(WindField):
    """冰川/坡地热力驱动的沿坡风(Prandtl 1942 稳态坡风解析解)。

    冷却坡面(冰川/夜间)→ 顺坡下泄(katabatic);受热坡面(白天)→ 上坡(anabatic)。
    沿坡速度随离地高度呈低空急流:u(z)=U_peak·e^{-z/λ}sin(z/λ)/f_max,峰在 z=π·λ/4,
    之上衰减并出现弱反向回流(补偿流)。λ=4·z_peak/π。

    沿坡 fetch 增强(可选):峰速随下坡累积距离线性增长
        U_peak_eff = U_peak · (1 + clip(fetch_gain·d_downslope, 0, fetch_max))
    模拟冷空气顺坡越积越强。

    [假设] 无限均匀坡、稳态、常扩散系数;fetch 增强为线性经验修正
    / 冰川/缓坡近地面一阶结构正确,峰高/峰速/增益需实测标定
    / 复杂地形、非稳态爆发(katabatic surge)时失效。
    """

    def __init__(self, downhill_azimuth_deg: float, peak_speed_m_s: float,
                 jet_height_m: float, katabatic: bool = True,
                 fetch_gain_per_m: float = 0.0, fetch_max: float = 1.0):
        self._peak = float(peak_speed_m_s)
        self._lambda = max(4.0 * float(jet_height_m) / math.pi, 1e-3)
        sign = 1.0 if katabatic else -1.0
        self._dir = sign * _azimuth_unit(downhill_azimuth_deg)
        self._downhill = _azimuth_unit(downhill_azimuth_deg)   # 下坡水平单位矢量
        self._fetch_gain = float(fetch_gain_per_m)
        self._fetch_max = float(fetch_max)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        z = max(float(height_agl_m), 0.0)
        s = z / self._lambda
        speed = self._peak * math.exp(-s) * math.sin(s) / _PRANDTL_PEAK
        if self._fetch_gain > 0.0:
            d_down = float(np.dot(np.asarray(pos_ned_m, dtype=float)[:2], self._downhill[:2]))
            gain = min(max(self._fetch_gain * max(d_down, 0.0), 0.0), self._fetch_max)
            speed *= (1.0 + gain)
        return speed * self._dir

    @property
    def jet_lambda_m(self) -> float:
        return self._lambda


class TabulatedProfileWind(WindField):
    """高度-风廓线查表(接入实测探空/塔站/ERA5 再分析的入口)。

    给定升序 heights_m 与每层 NED 水平分量 (N,E),按高度 AGL 线性插值(N/E 分量,
    避免方向角在 ±180° 处绕错)。超界钳位到端点。
    """

    def __init__(self, heights_m, ne_components):
        h = np.asarray(heights_m, dtype=float)
        ne = np.asarray(ne_components, dtype=float).reshape(len(h), 2)
        order = np.argsort(h)
        self._h = h[order]
        self._n = ne[order, 0]
        self._e = ne[order, 1]

    @classmethod
    def from_speed_dir(cls, heights_m, speeds_m_s, dirs_to_deg):
        ne = []
        for spd, d in zip(speeds_m_s, dirs_to_deg):
            a = math.radians(d)
            ne.append((spd * math.cos(a), spd * math.sin(a)))
        return cls(heights_m, ne)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        z = float(height_agl_m)
        n = float(np.interp(z, self._h, self._n))
        e = float(np.interp(z, self._h, self._e))
        return np.array([n, e, 0.0])


class LakeBreezeFront(WindField):
    """湖陆风 + 向内陆推进的锋面(白天:湖→陆 onshore;锋面处辐合+上升)。

    岸线法向 onshore 方向 = onshore_dir_deg(风吹向陆地的方位)。沿该轴坐标 s = 位置·n:
    s < shore_offset 为湖面,s > shore_offset 为陆地。锋面向内陆推进:
        s_front(t) = shore_offset + front_speed·(t − start_time)
    锋后(shore < s < front):onshore 入流,表面最大、depth 内线性减到 0。
    锋面带(|s−front| < front_width/2):水平减速 + 垂直上升 updraft(辐合)。
    锋前(s > front):该分量为 0(海风未到)。湖面(s < shore):仍有 onshore 表层入流。

    [假设] 运动学参数化(非解方程),抓住"随离岸距离与高度变化的 onshore 风 +
      推进的辐合上升锋面"这三个对低空飞行最相关的特征
    / 定性/相对 case 验证够用 / 精细锋面结构需中尺度模式或实测。
    """

    def __init__(self, onshore_dir_deg: float, shore_offset_m: float = 0.0,
                 inflow_speed_m_s: float = 3.0, depth_m: float = 200.0,
                 front_speed_m_s: float = 2.0, front_width_m: float = 120.0,
                 updraft_m_s: float = 1.0, start_time_s: float = 0.0):
        self._n = _azimuth_unit(onshore_dir_deg)       # onshore 水平单位矢量
        self._shore = float(shore_offset_m)
        self._inflow = float(inflow_speed_m_s)
        self._depth = max(float(depth_m), 1.0)
        self._front_speed = float(front_speed_m_s)
        self._front_w = max(float(front_width_m), 1.0)
        self._updraft = float(updraft_m_s)
        self._t0 = float(start_time_s)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        pos = np.asarray(pos_ned_m, dtype=float)
        s = float(np.dot(pos[:2], self._n[:2]))
        z = max(float(height_agl_m), 0.0)
        front = self._shore + self._front_speed * max(time_s - self._t0, 0.0)
        if s > front or z > self._depth:
            return np.zeros(3)
        # onshore 入流,随高度线性减弱
        u = self._inflow * (1.0 - z / self._depth)
        w = np.array([u * self._n[0], u * self._n[1], 0.0])
        # 锋面带:水平减速 + 上升(辐合)
        d = abs(s - front)
        if d < self._front_w * 0.5:
            frac = d / (self._front_w * 0.5)          # 0(锋心)→1(带边)
            w[0] *= frac
            w[1] *= frac
            w[2] += -self._updraft * (1.0 - frac) * (1.0 - z / self._depth)  # D<0 上升
        return w


class ThermalField(WindField):
    """对流热泡阵列(高原/受热地表白天):规则网格上的上升气流单元 + 补偿下沉。

    每个单元中心处上升,周边环状下沉:w(r)=w_peak·(1−0.5(r/R)²)·e^{−(r/R)²}(核心正、外环负,
    面积积分近零 → 近似质量守恒)。垂直分量随高度包络 exp(−z/z_top) 衰减。单元可随平均风平移。
    单元中心可加种子抖动打破规则性。垂直为主,水平忽略。

    [假设] 简化 glider 热泡模型,不解浮力方程 / 定性抖振/垂直扰动验证够用
    / 精确对流需 LES。
    """

    def __init__(self, spacing_m: float = 150.0, core_radius_m: float = 40.0,
                 peak_updraft_m_s: float = 2.0, top_m: float = 400.0,
                 drift_ned_m_s=(0.0, 0.0, 0.0), jitter_m: float = 0.0, seed: int = 0):
        self._spacing = max(float(spacing_m), 1.0)
        self._R = max(float(core_radius_m), 1e-3)
        self._peak = float(peak_updraft_m_s)
        self._top = max(float(top_m), 1.0)
        self._drift = np.asarray(drift_ned_m_s, dtype=float)
        self._jitter = float(jitter_m)
        self._seed = int(seed)

    def _center_jitter(self, ix: int, iy: int) -> np.ndarray:
        if self._jitter <= 0.0:
            return np.zeros(2)
        # 由 (ix,iy,seed) 确定性派生的抖动,保证可复现
        rng = np.random.default_rng((self._seed & 0xFFFF) * 2654435761
                                    ^ (ix & 0xFFFF) * 40503 ^ (iy & 0xFFFF) * 12289)
        return (rng.random(2) - 0.5) * 2.0 * self._jitter

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        pos = np.asarray(pos_ned_m, dtype=float)
        z = max(float(height_agl_m), 0.0)
        # 网格随平均风平移
        x = pos[0] - self._drift[0] * time_s
        y = pos[1] - self._drift[1] * time_s
        env = math.exp(-z / self._top)
        w_up = 0.0
        ix0, iy0 = round(x / self._spacing), round(y / self._spacing)
        for ix in (ix0 - 1, ix0, ix0 + 1):
            for iy in (iy0 - 1, iy0, iy0 + 1):
                c = np.array([ix, iy]) * self._spacing + self._center_jitter(ix, iy)
                r2 = (x - c[0]) ** 2 + (y - c[1]) ** 2
                g = r2 / (self._R ** 2)
                w_up += self._peak * (1.0 - 0.5 * g) * math.exp(-g)
        return np.array([0.0, 0.0, -w_up * env])   # D<0 上升


class RidgeTerrain(WindField):
    """山脊地形对基础风的修正:脊顶加速 + 背风近地面回流(装饰器,包住基础风)。

    横脊坐标 ξ = (位置 − 脊顶)·across(across = 垂直脊走向的水平单位矢量)。
    脊顶加速:factor = 1 + Δs·exp(−(ξ/L)²)·exp(−z/z_infl),低矮山 Δs≈2H/L(Jackson-Hunt 量级)。
    背风回流:下风侧一段近地面反向流(分离),幅度 lee_frac×基础风,随高度快速衰减。
    下风侧由基础风水平方向判定。

    [假设] 低矮缓坡线性加速理论 + 经验背风回流 / 定性地形效应够用
    / 陡坡强分离需 CFD。
    """

    def __init__(self, base: WindField, crest_ne_m=(0.0, 0.0), ridge_axis_deg: float = 0.0,
                 height_m: float = 30.0, halfwidth_m: float = 100.0,
                 z_influence_m: float = 60.0, lee_recirc_frac: float = 0.0):
        self._base = base
        self._crest = np.asarray(crest_ne_m, dtype=float)
        # across = 垂直于脊走向(ridge_axis 为脊线走向方位)
        self._across = _azimuth_unit(ridge_axis_deg + 90.0)
        self._dsmax = 2.0 * float(height_m) / max(float(halfwidth_m), 1e-3)
        self._L = max(float(halfwidth_m), 1e-3)
        self._zinfl = max(float(z_influence_m), 1e-3)
        self._lee = float(lee_recirc_frac)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        pos = np.asarray(pos_ned_m, dtype=float)
        base = np.asarray(self._base.wind_ned(pos, height_agl_m, time_s), dtype=float)
        z = max(float(height_agl_m), 0.0)
        xi = float(np.dot(pos[:2] - self._crest, self._across[:2]))
        # 脊顶加速(乘性,随高度衰减)
        factor = 1.0 + self._dsmax * math.exp(-(xi / self._L) ** 2) * math.exp(-z / self._zinfl)
        w = base.copy()
        w[0] *= factor
        w[1] *= factor
        # 背风回流:下风侧(基础风指向的一侧)一段近地面反向
        if self._lee > 0.0:
            bh = base[:2]
            bspd = float(np.hypot(bh[0], bh[1]))
            if bspd > 1e-6:
                downwind = float(np.dot(pos[:2] - self._crest, bh / bspd))
                if downwind > 0.0:  # 位于下风侧
                    zone = math.exp(-((downwind - 1.5 * self._L) / self._L) ** 2)
                    rev = -self._lee * bspd * zone * math.exp(-z / (0.5 * self._zinfl))
                    w[0] += rev * bh[0] / bspd
                    w[1] += rev * bh[1] / bspd
        return w


class CompositeWind(WindField):
    """分量线性叠加(边界层 + 坡风 + 湖陆风 + 热泡 …)。"""

    def __init__(self, components):
        self._c = list(components)

    def wind_ned(self, pos_ned_m, height_agl_m, time_s):
        w = np.zeros(3)
        for c in self._c:
            w = w + np.asarray(c.wind_ned(pos_ned_m, height_agl_m, time_s), dtype=float)
        return w


# ---------- 环境预设 ----------
def glacier_katabatic(downhill_azimuth_deg: float = 180.0, peak_speed_m_s: float = 4.0,
                      jet_height_m: float = 6.0, base_wind_m_s: float = 1.0,
                      z0_m: float = 2e-4, fetch_gain_per_m: float = 0.0) -> WindField:
    """冰川下降风:光滑冰面对数边界层 + 顺坡 Prandtl 急流(可选 fetch 增强)。"""
    return CompositeWind([
        LogLawWind(base_wind_m_s, downhill_azimuth_deg, z0_m=z0_m),
        PrandtlSlopeWind(downhill_azimuth_deg, peak_speed_m_s, jet_height_m,
                         katabatic=True, fetch_gain_per_m=fetch_gain_per_m),
    ])


def plateau_slope(downhill_azimuth_deg: float = 180.0, peak_speed_m_s: float = 3.0,
                  jet_height_m: float = 10.0, daytime: bool = False,
                  base_wind_m_s: float = 2.0, z0_m: float = 0.03,
                  thermals: bool = False, thermal_updraft_m_s: float = 2.0) -> WindField:
    """高原坡地风:白天上坡(anabatic,可叠加热泡)、夜间下坡(katabatic);z0≈0.03。"""
    comps = [
        LogLawWind(base_wind_m_s, downhill_azimuth_deg, z0_m=z0_m),
        PrandtlSlopeWind(downhill_azimuth_deg, peak_speed_m_s, jet_height_m,
                         katabatic=not daytime),
    ]
    if daytime and thermals:
        comps.append(ThermalField(spacing_m=150.0, core_radius_m=40.0,
                                  peak_updraft_m_s=thermal_updraft_m_s,
                                  drift_ned_m_s=(base_wind_m_s, 0.0, 0.0)))
    return CompositeWind(comps)


def lake_env(onshore_dir_deg: float = 0.0, u10_m_s: float = 3.0,
             breeze: bool = True, inflow_speed_m_s: float = 3.0,
             front_speed_m_s: float = 2.0, z0_m: float = 1e-3) -> WindField:
    """湖泊:水面对数边界层(低粗糙度)+ 可选湖陆风推进锋面。"""
    comps = [LogLawWind(u10_m_s, onshore_dir_deg, z0_m=z0_m, z_ref_m=10.0)]
    if breeze:
        comps.append(LakeBreezeFront(onshore_dir_deg, inflow_speed_m_s=inflow_speed_m_s,
                                     front_speed_m_s=front_speed_m_s))
    return CompositeWind(comps)


def lake_surface(u10_m_s: float = 3.0, dir_to_deg: float = 90.0,
                 z0_m: float = 1e-3) -> WindField:
    """湖面对数边界层(不含锋面)。"""
    return LogLawWind(u10_m_s, dir_to_deg, z0_m=z0_m, z_ref_m=10.0)


# ---------- 声明式构建(JSON 可序列化,供 config / 网页调用) ----------
def build_wind_field(spec):
    """从 {"kind":..., "params":{...}} 声明式规格构建 WindField。

    支持 kind: uniform, loglaw, slope, tabulated, lake_breeze, thermals,
    glacier_katabatic, plateau_slope, lake_env, lake_surface, composite, ridge。
    composite: {"kind":"composite","components":[spec,...]}
    ridge:     {"kind":"ridge","base":spec,"params":{...}}
    """
    if spec is None:
        return None
    if isinstance(spec, WindField):
        return spec
    kind = spec["kind"]
    p = spec.get("params", {})
    if kind == "composite":
        return CompositeWind([build_wind_field(s) for s in spec["components"]])
    if kind == "ridge":
        return RidgeTerrain(build_wind_field(spec["base"]), **p)
    simple = {
        "uniform": UniformWind, "loglaw": LogLawWind, "slope": PrandtlSlopeWind,
        "lake_breeze": LakeBreezeFront, "thermals": ThermalField,
        "glacier_katabatic": glacier_katabatic, "plateau_slope": plateau_slope,
        "lake_env": lake_env, "lake_surface": lake_surface,
    }
    if kind == "tabulated":
        return TabulatedProfileWind.from_speed_dir(
            p["heights_m"], p["speeds_m_s"], p["dirs_to_deg"])
    return simple[kind](**p)
