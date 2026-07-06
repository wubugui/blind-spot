# 室内遥控飞艇 6DOF 工程仿真

工程仿真工具，**不是游戏**。用于在制造真实飞艇之前验证控制算法、估算性能边界、
辅助硬件选型。物理保真优先：所有参数有公式推导、文献典型值或显式标注的工程假设；
固定种子下轨迹逐位可复现；渲染层零物理逻辑、不插值物理状态。

## 快速开始

```bash
pip install -r requirements.txt

# 单元测试(56 个:能量守恒、Lamb 解析解、ISA 对表、逐位复现等)
python -m pytest tests/ -q

# 验证场景(输出指标 + output/*.png 响应曲线)
python scenarios/hover.py             # 无风定点悬停 60s
python scenarios/step_response.py     # 前进2m / 转90° / 爬升1m
python scenarios/wind_rejection.py    # 0.5 / 1.5 m/s 常值风悬停

# 实时可视化(浏览器开 http://localhost:8000/)
python scenarios/run_realtime.py                       # 室内,无风
python scenarios/run_realtime.py --wind 0.5            # 室内 + 常值风
python scenarios/run_realtime.py --atmo low --turb 1   # ISA + 边界层风 + 湍流
python scenarios/run_realtime.py --atmo high --helium  # 高空剖面 + 氦气热力学
```

前端功能:3D 姿态/轨迹/风与推力箭头、30s 滚动遥测曲线(设定值 vs 实际值)、
PID 增益滑块(实时生效)、暂停/单步/重置/时间倍率、WASD+QE 手动模式、
大气面板(风层滑块、湍流、ΔT、阵风按钮、太阳辐照、垂直剖面图)。

## 物理模型摘要

| 模块 | 模型 | 依据 |
| --- | --- | --- |
| 刚体动力学 | 6DOF,浮心参考点,四元数姿态,RK4 @ 1ms | Fossen (2011) 船舶/浮空器标准体系 |
| 附加质量 | 长椭球 Lamb 系数 k1=0.156, k2=0.762, k'=0.365 | Lamb (1932) §114-115 解析式,单元测试对表 |
| Munk 力矩 | 由 C_A(ν_r) 科氏项自动给出 | 同上,无需单独建模 |
| 浮力/重力 | 浮力在浮心(当地密度实时计算),重心低 0.4m 提供摆动恢复力矩 | 几何配置推导 |
| 气动阻力 | 轴向 Cd 0.075 / 侧向 0.9,各自参考面积;转动阻尼条带理论积分 | 文献典型值区间中值 |
| 尾翼 | 十字尾翼线性升力线(S·CLα·x_fin),部分抵消 Munk + 提供阻尼 | 见下"关键工程结论" #2 |
| 执行器 | 一阶响应 τ=0.1s,±0.05N 上限,差速布局真实几何(含俯仰耦合力矩) | 规格给定 |
| 传感器 | IMU 100Hz、气压计 10Hz σ=0.05m(2σ=±0.1m),独立种子 RNG | ESP32+BMP390 量级 |
| 控制 | 串级 PID 50Hz:高度(+悬停油门前馈)/偏航/前速 + 位置保持外环 | 增益由回路整形推导,注释在 config.py |
| ISA 大气 | 0~32km 三层剖面(位势高度),11km 密度误差 <0.1%;ΔT 偏移 | ISO 2533 / USSA 1976 |
| 分层风 | 高度层线性插值(N/E 分量);低空幂律边界层 α=0.14 / 高空 0~30km 预置 | 中性层结幂律 |
| 湍流 | 一阶 Gauss-Markov 近似 Dryden 谱,σ/L 随高度变化,σ_w=0.7σ_u | MIL-HDBK-1797 量级 |
| 阵风 | 1-cosine 脉冲,方向/幅值/时长可配,可叠加 | 标准阵风模型 |
| 氦气热力学 | 一阶集总热模型,superheat→定容排气→净浮力;平衡 superheat≈20K | 平衡值与实艇 10~30K 一致 |

每个简化在代码注释中以 `[假设] 内容 / 影响 / 何时失效` 格式声明(全文检索 `[假设]`)。
**未建模**:地面/天花板碰撞、囊体柔性变形、电池电压跌落、螺旋桨非线性。

## 关键工程结论(默认参数:2.0m×0.8m 囊体,5g 偏重,单电机 0.05N)

1. **垂直电机选型不足**。5g 配重的稳态悬停推力 0.049N ≈ 电机上限 0.05N:
   悬停在 99% 油门、81% 时间饱和;向上控制余量仅 0.001N → 最大爬升率 0.028 m/s,
   爬升 1m 需 ~40s。建议:垂直电机上限 ≥0.15N,或配平到 1~2g。
   (无风悬停本身可用:60s 高度误差 RMS 0.053m,姿态 RMS <0.2°。)
2. **裸椭球方向不稳定**。无尾翼时 Munk 力矩使 0.5m/s 风下航向 10s 内发散到横对风
   (差速力矩 ±0.01N·m 比 Munk 斜率小一个量级)——这正是真实飞艇必须装尾翼的原因。
   默认配置含十字尾翼(单平面 0.15m²,浮心后 0.85m),完全抵消 Munk 需
   S·CLα·|x_fin| > 0.81 m³(不现实的尾翼面积),剩余靠主动控制。
3. **抗风边界由偏航力矩决定,而非前向推力**。前向推力理论可顶 ~1.9 m/s,
   但实测:0.5 m/s 可保持(均值偏差 0.58m);0.6 临界;≥0.7 发散——
   Munk 力矩随风速平方增长,差速 ±0.01N·m 先饱和(此时前向推力仅用 25%)。
   建议:增大尾翼、加大电机横向间距、或加独立尾舵电机。
4. 阶跃性能(力矩/推力饱和主导):前进 2m 上升 8s 超调 6%;原地转 90° 上升 13s
   超调 6%(积分分离解决了反向饱和刹车段的积分累积超调)。
5. 氦气热力学:太阳辐照下平衡 superheat ≈20K → 净浮力增益 ≈8g,**超过 5g 配重**,
   昼夜切换会翻转配平符号——高空/户外任务必须主动配平或控制阀排气。

## 游戏《风程》(基于同一物理引擎)

`game/` 下是一个完整的飞艇旅途游戏(载人级,浏览器运行):驾驶真实物理驱动的
飞艇横跨 12 驿站 5 区域的大陆,在驿站改装(气囊/动力/尾翼/电池/太阳能/涂层,
机库按物理实时评级),穿越平原/湖泊/山脊/高原/冰川的真实风场抵达极光城。
货运经济、时段天气(热泡/湖陆风/下降风昼夜节律)、压舱与放氦(ballonet 模型)、
长存档。引擎保持工程纯净:游戏只 import `airship_sim`,新增的地形碰撞
(ground.py)与能量模型(energy.py)按工程标准带 `[假设]` 与测试,可独立复用。

```bash
python game/build_game.py                # 生成自包含 game/index.html
PYTHONPATH=.:game python3 game/verify_world.py   # 世界完备性:每条航线可飞完
```
设计文档见 `game/DESIGN.md`。

## 目录结构

```
airship_sim/          物理引擎与服务(模块化,互相独立可替换)
  config.py           全部物理参数(SI 单位,带单位后缀),JSON 序列化
  math3d.py           四元数/坐标变换(NED)
  added_mass.py       Lamb 附加质量
  aero.py             阻力/转动阻尼/尾翼
  dynamics.py         6DOF 动力学 + RK4
  sensors.py          IMU/气压计噪声模型
  controller.py       串级 PID + 位置保持外环
  atmosphere.py       统一接口 get_state(pos,t)→密度/温压/风;ISA/风层/湍流/阵风
  wind_field.py       空间结构化风场:Prandtl 坡风/对数边界层/湖陆风锋面/热泡/山脊地形/查表
  environments.py     极端环境预设:冰川/高原/湖泊(密度配平+空间风场+太阳负荷一键配好)
  simulation.py       主循环(1ms 物理,50Hz 控制,整数 tick 调度) + 氦气热模型
  server.py           WebSocket 50Hz 广播 + 指令 + NED→Three.js 协议层转换
web/index.html        单文件 Three.js 前端(纯渲染)
scenarios/            可直接运行的验证场景(含 env_cases.py 三环境 case 跑批)
tests/                77 个单元/集成测试
```

## 极端环境与空间风场(低空 2~50m)

物理引擎按位置/时间逐点取风,因此空间风场无需改动力学。`environments.py` 一键配好:

```python
from airship_sim.environments import glacier, plateau, lake
from airship_sim.simulation import Simulation
cfg = glacier(ground_alt_m=4000, air_temp_C=-12, katabatic_peak_m_s=4, jet_height_m=6)
sim = Simulation(cfg)           # 密度已按 4000m 配平,推力随密度缩放,挂上下降风急流
```

或跑批:`python scenarios/env_cases.py`(冰川/高原/湖泊定点抗风,出漂移/饱和指标+PNG)。

**抗风重型机型(能在三环境定点悬停)**:默认机型(0.05N 电机、~0.5m/s 抗风)会被吹走;
`airframe="heavy"` 是标定出来的重型机型(2.8×1.3m 囊体、3N 电机、0.6m 横距、1.5m² 尾翼、
重整定增益、3m 定点死区),配合**迎风起始朝向**(`heading_into_wind_quat`),实测定点漂移
冰川≈2.7m、高原≈0.9m、湖泊≈2.0m:

```python
from airship_sim.environments import glacier, heading_into_wind_quat
cfg = glacier(katabatic_peak_m_s=4, airframe="heavy")
sim = Simulation(cfg)
q = heading_into_wind_quat(sim.atmosphere, (0,0,-10), 0.0)   # 机头迎风
sim.reset(p_ned_m=(0,0,-10), q=tuple(q))
```
跑批加 `--airframe heavy`。网页"极端环境"面板勾选"抗风重型"即可。**关键工程结论:低空
2~4m/s 定点需要比默认大得多的机型**;且欠驱动(无侧推)只能守半径(死区)不能守点——
起飞须迎风,否则背风掉头期间被吹散。

| 环境 | 主导物理(已建模) |
| --- | --- |
| 冰川 | 低温高密度 + **Prandtl 下降风急流**(低空 jet,峰在 πλ/4)+ 冰面反照太阳负荷 |
| 高原 | **低密度**(浮力↓、气动↓、**螺旋桨推力 T∝ρ ↓**)+ 上坡风/**热泡上升气流** + 强日照超温 |
| 湖泊 | 水面低粗糙度对数层 + **湖陆风推进锋面**(辐合+上升) |

风场模型(`wind_field.py`,均带 `[假设]` 声明,可 `CompositeWind` 叠加、JSON 声明式构建):
Prandtl 坡风(含沿坡 fetch 增强)、对数边界层(z0 随下垫面)、湖陆风锋面、对流热泡阵列、
山脊加速+背风回流、高度-风廓线查表(接入实测探空/ERA5)。网页端"极端环境"面板可交互切换。

> **准确性提示**:以上均为解析/运动学参数化,抓住对低空飞行最相关的一阶结构,便于快速验证
> case;定量任务须用实测(风洞/推力台/探空/飞行辨识)标定参数——见接口下文。

## 时序与可复现性

- 物理 1ms(RK4)、控制 50Hz、IMU 100Hz、气压计 10Hz,统一整数 tick 调度,无浮点漂移
- 同一 `SimConfig`(含 seed)→ 逐位一致轨迹(`test_determinism.py`);
  配置可 `SimConfig.save/load` JSON 往返
- 实时服务按墙钟配速;时间加速为尽力而为(纯 Python 物理 ≈0.5~0.65s/仿真秒,
  约可 ×1.5;落后时不追赶,事件循环周期性让出保证指令响应)

## 大气接口(替换为真实数据的入口)

物理引擎只调用 `Atmosphere.get_state(position_ned_m, time_s) → AtmosphereState
(rho_kg_m3, temperature_K, pressure_Pa, wind_ned_m_s)`。
接入 ERA5 等再分析数据时实现该接口即可,动力学代码零改动。
