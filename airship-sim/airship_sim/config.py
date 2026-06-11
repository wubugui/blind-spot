"""集中配置:所有物理参数(SI 单位,变量名带单位后缀)。

支持 JSON 序列化保存/加载(可复现性要求:同一配置 + 同一种子 → 逐位一致轨迹)。
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass, field

import numpy as np

GRAVITY_M_S2 = 9.80665  # 标准重力加速度


@dataclass
class HullConfig:
    """囊体几何:长椭球,x 轴为纵轴。"""
    length_m: float = 2.0          # 总长(= 2a)
    diameter_m: float = 0.8        # 最大直径(= 2b)

    @property
    def a_m(self) -> float:
        return self.length_m / 2.0

    @property
    def b_m(self) -> float:
        return self.diameter_m / 2.0

    @property
    def volume_m3(self) -> float:
        # 长椭球体积 V = 4/3·π·a·b² ≈ 0.670 m³
        return 4.0 / 3.0 * np.pi * self.a_m * self.b_m**2

    @property
    def cross_area_m2(self) -> float:
        """横截面积 πb²(轴向阻力参考面积)。"""
        return np.pi * self.b_m**2

    @property
    def side_area_m2(self) -> float:
        """侧投影面积 πab(侧向阻力参考面积,椭圆)。"""
        return np.pi * self.a_m * self.b_m


@dataclass
class MassConfig:
    """质量分布。动力学参考点取浮心 CB(=椭球几何中心),体轴系 z 向下。

    [假设] 囊体蒙皮质量近似为薄椭球壳、氦气为均匀实心椭球、吊舱为质点
    / 转动惯量误差约 ±20%,对摆动周期影响 ~10%
    / 需要精确姿态动力学(如挂载云台)时应实测惯量。
    """
    m_envelope_kg: float = 0.108        # 囊体蒙皮+尾翼等结构(≈4.0m² 表面积 × 27g/m² 薄膜)
    m_gondola_kg: float = 0.600716      # 吊舱(电机+电池+电子设备),默认值由 default_config() 配平得出
    rho_helium_kg_m3: float = 0.175     # 囊内气体密度 = 地面空气1.225 - 净浮力1.05(含杂质氦,纯氦≈0.169)
    r_gondola_b_m: tuple[float, float, float] = (0.0, 0.0, 0.55)
    # 吊舱质心在体轴系中相对浮心的位置(z 向下,0.55m = 囊体半径0.4 + 吊挂0.15)


@dataclass
class AeroConfig:
    """气动阻力系数。

    [假设] 椭球轴向 Cd 取 0.075(文献典型范围 0.05~0.1,长细比2.5、层流为主),
    侧向(横流)Cd 取 0.9(圆柱横流典型 0.8~1.0)
    / Cd 不随 Re 与攻角变化,误差约 ±30% / 速度接近 2m/s 或大攻角时需风洞或CFD修正。
    """
    cd_axial: float = 0.075        # 轴向阻力系数,参考面积 = 横截面积
    cd_lateral: float = 0.9        # 侧向阻力系数,参考面积 = 侧投影面积
    # 转动阻尼:俯仰/偏航主项为条带理论横流阻尼(系数由 cd_lateral 推出,见 aero.py),
    # 此处为附加的小线性阻尼(粘性/蒙皮摩擦),量级估计:
    c_ang_lin_roll_Nms: float = 2e-4    # [假设] 滚转仅有蒙皮摩擦阻尼,取小值 / 滚转收敛慢 / 不影响其他通道
    c_ang_lin_pitch_Nms: float = 1e-3
    c_ang_lin_yaw_Nms: float = 1e-3


@dataclass
class ActuatorConfig:
    """执行器:左右差速电机(体轴 +x 推力)+ 垂直电机(体轴 -z 推力为正=向上)。"""
    thrust_max_N: float = 0.05       # 单个电机推力上限
    tau_s: float = 0.1               # 电机一阶响应时间常数
    motor_sep_y_m: float = 0.10      # 左右电机距中线的横向半距
    # 电机安装在吊舱上(z 与吊舱一致):推力线不过重心,会产生俯仰力矩——按真实几何建模,不简化掉
    motor_z_b_m: float = 0.55
    # [假设] 电机可双向出力(±thrust_max),正反转推力对称
    # / 实际反转效率低 ~30% / 选定不可反转螺旋桨时需把下限改为 0
    reversible: bool = True


@dataclass
class SensorConfig:
    """传感器模型(模拟 ESP32 + IMU + BMP390)。"""
    imu_rate_hz: float = 100.0
    baro_rate_hz: float = 10.0
    baro_noise_std_m: float = 0.05       # BMP390 高度噪声,σ=0.05m → 2σ≈±0.1m(规格给定)
    gyro_noise_std_rad_s: float = 0.002  # 典型 MEMS 陀螺白噪声(≈0.1°/s)
    # [假设] 姿态角与前向速度由机载融合/外部定位直接给出,等效为白噪声测量
    # / 跳过姿态估计器的收敛与漂移动态 / 验证估计算法本身时需替换为原始 IMU + 融合器
    att_noise_std_rad: float = 0.005     # ≈0.3°
    vel_noise_std_m_s: float = 0.02


@dataclass
class PidGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    tau_d_s: float = 0.2       # 微分项一阶低通时间常数
    tau_meas_s: float = 0.0    # 测量值一阶低通(0=不滤波)。用于低采样率/高噪声测量
    #   (如 10Hz 保持采样的气压计):避免保持跳变被微分放大、噪声经 Kp 在
    #   推力上限处单边削顶(整流)压低平均推力。控制器侧滤波,不触碰物理状态。
    out_min: float = -1.0
    out_max: float = 1.0


@dataclass
class ControlConfig:
    """串级 PID。增益初值由回路整形估算(见各注释),由阶跃/抗风场景验证修正。

    通道结构:
      高度:气压计高度 → 垂直推力 [N]
      偏航:航向角     → 偏航力矩指令 [N·m] → 差速
      前速:体轴前向速度 → 总前向推力 [N]
      位置保持(外环,可选):水平位置误差 → 航向设定 + 前速设定
    """
    rate_hz: float = 50.0
    # 高度通道:垂直等效质量 m+k2ρV ≈ 1.45 kg;取 ωn≈0.26 rad/s, ζ≈1
    #   Kp = m·ωn² ≈ 0.10 N/m, Kd = 2ζωn·m ≈ 0.75(气压计 10Hz/σ0.05m,微分须重滤波,
    #   实取 0.4 + τ_d=1.5s 折衷);积分项只负担前馈标定残差
    alt: PidGains = field(default_factory=lambda: PidGains(
        kp=0.10, ki=0.01, kd=0.4, tau_d_s=1.5, tau_meas_s=0.6,
        out_min=-0.05, out_max=0.05))
    # 悬停油门前馈 [N]:抵消配重的稳态推力,默认由 default_config() 置为 heaviness·g。
    # [假设] 悬停油门已在实机试飞中标定(标准做法),仿真中直接取配平值
    # / 免去积分器分钟级爬升 / 标定误差大或浮力随环境变化快时由积分项兜底
    alt_ff_N: float = 0.0
    # 偏航通道:偏航等效惯量 I+I_a ≈ 0.14 kg·m²;ωn≈0.35 rad/s, ζ≈0.9
    #   Kp = I·ωn² ≈ 0.017 N·m/rad, Kd = 2ζωn·I ≈ 0.087;最大差速力矩 ±0.01 N·m
    yaw: PidGains = field(default_factory=lambda: PidGains(
        kp=0.017, ki=0.001, kd=0.087, tau_d_s=0.3, out_min=-0.01, out_max=0.01))
    # 前速通道:轴向等效质量 m+k1ρV ≈ 0.95 kg;一阶目标带宽 ω≈0.3 rad/s → Kp = m·ω ≈ 0.29
    speed: PidGains = field(default_factory=lambda: PidGains(
        kp=0.29, ki=0.03, kd=0.0, out_min=-0.1, out_max=0.1))
    # 位置保持外环(P):距离 → 前速设定,饱和于 max_speed
    pos_hold_kp_1_s: float = 0.15
    pos_hold_max_speed_m_s: float = 0.5
    pos_hold_deadband_m: float = 0.15   # 距目标小于该值时不再调头追位置,避免原地振荡


@dataclass
class AtmosphereConfig:
    """大气模块配置(阶段1只用 constant 模型;阶段5扩展 ISA/分层风/湍流/阵风)。"""
    model: str = "constant"              # constant | isa(阶段5)
    rho_const_kg_m3: float = 1.225       # constant 模型空气密度(ISA 海平面)
    temperature_const_K: float = 288.15
    pressure_const_Pa: float = 101325.0
    wind_ned_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)  # 常值风(世界系 NED)


@dataclass
class SimConfig:
    """顶层配置。"""
    dt_physics_s: float = 0.001          # 物理步长 1ms,RK4
    seed: int = 20260611                 # 全局随机种子(传感器噪声/湍流由此派生)
    hull: HullConfig = field(default_factory=HullConfig)
    mass: MassConfig = field(default_factory=MassConfig)
    aero: AeroConfig = field(default_factory=AeroConfig)
    actuator: ActuatorConfig = field(default_factory=ActuatorConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    atmosphere: AtmosphereConfig = field(default_factory=AtmosphereConfig)

    # ---- 序列化(可复现性要求) ----
    def to_json(self) -> str:
        return json.dumps(dataclasses.asdict(self), indent=2)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            f.write(self.to_json())

    @classmethod
    def from_json(cls, text: str) -> "SimConfig":
        return _from_dict(cls, json.loads(text))

    @classmethod
    def load(cls, path: str) -> "SimConfig":
        with open(path) as f:
            return cls.from_json(f.read())


def _from_dict(cls, d):
    """按 dataclass 字段类型递归还原(list → tuple)。"""
    kwargs = {}
    for f in dataclasses.fields(cls):
        if f.name not in d:
            continue
        v = d[f.name]
        if dataclasses.is_dataclass(f.type) if isinstance(f.type, type) else False:
            kwargs[f.name] = _from_dict(f.type, v)
        elif isinstance(v, dict):
            sub = f.default_factory() if f.default_factory is not dataclasses.MISSING else None
            kwargs[f.name] = _from_dict(type(sub), v) if sub is not None else v
        elif isinstance(v, list):
            kwargs[f.name] = tuple(v)
        else:
            kwargs[f.name] = v
    return cls(**kwargs)


def default_config(heaviness_kg: float = 0.005) -> SimConfig:
    """默认配置:在地面密度下把吊舱质量配平为总重 = 排开空气质量 + heaviness_kg。

    heaviness_kg 默认 5g(略负浮力,近中性)。
    """
    cfg = SimConfig()
    v = cfg.hull.volume_m3
    m_helium = cfg.mass.rho_helium_kg_m3 * v
    m_displaced = cfg.atmosphere.rho_const_kg_m3 * v
    cfg.mass.m_gondola_kg = m_displaced + heaviness_kg - cfg.mass.m_envelope_kg - m_helium
    if cfg.mass.m_gondola_kg <= 0:
        raise ValueError("envelope + helium already heavier than buoyancy + heaviness")
    # 悬停油门前馈 = 配重 × g(视为实机试飞标定值,见 ControlConfig 注释)
    cfg.control.alt_ff_N = heaviness_kg * GRAVITY_M_S2
    return cfg
