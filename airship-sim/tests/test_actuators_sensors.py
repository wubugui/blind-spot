"""执行器一阶响应/饱和,传感器噪声统计与采样率。"""
import numpy as np

from airship_sim.atmosphere import make_atmosphere
from airship_sim.config import default_config
from airship_sim.dynamics import IDX_THRUST, AirshipDynamics
from airship_sim.sensors import SensorSuite
from airship_sim.simulation import Simulation


def test_motor_first_order_response():
    cfg = default_config()
    dyn = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    x = dyn.initial_state()
    cmd = np.array([0.05, 0.0, 0.0])
    dt = 0.001
    tau = cfg.actuator.tau_s
    for i in range(round(tau / dt)):       # 推进一个时间常数
        x = dyn.step_rk4(x, cmd, i * dt, dt)
    f_at_tau = x[IDX_THRUST][0]
    assert np.isclose(f_at_tau, 0.05 * (1 - np.exp(-1)), rtol=1e-3)
    for i in range(round(5 * tau / dt)):   # 共 6τ
        x = dyn.step_rk4(x, cmd, i * dt, dt)
    assert np.isclose(x[IDX_THRUST][0], 0.05, rtol=5e-3)


def test_motor_saturation():
    cfg = default_config()
    dyn = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    assert np.allclose(dyn.clamp_command(np.array([1.0, -1.0, 0.03])),
                       [0.05, -0.05, 0.03])
    cfg.actuator.reversible = False
    dyn2 = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    assert np.allclose(dyn2.clamp_command(np.array([-1.0, 0.02, 0.1])),
                       [0.0, 0.02, 0.05])


def test_baro_noise_statistics():
    cfg = default_config()
    suite = SensorSuite(cfg.sensor, np.random.default_rng(42))
    dyn = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    x = dyn.initial_state(p_ned_m=(0, 0, -2.0))   # 真实高度 2.0 m
    samples = []
    for i in range(3000):
        suite.sample_baro(x, i * 0.1)
        samples.append(suite.meas.altitude_m)
    samples = np.array(samples)
    assert np.isclose(samples.mean(), 2.0, atol=0.005)
    sigma = cfg.sensor.baro_noise_std_m
    assert 0.9 * sigma < samples.std() < 1.1 * sigma


def test_sampling_rates_in_simulation():
    sim = Simulation(default_config())
    sim.run(1.0, record=False)
    # 1s 内:气压计最后一次采样在 t=0.9(10Hz),IMU 在 t=0.99(100Hz)
    assert np.isclose(sim.sensors.meas.t_baro_s, 0.9)
    assert np.isclose(sim.sensors.meas.t_imu_s, 0.99)


def test_sensor_streams_independent():
    # 不同传感器使用独立 RNG 子流:同种子两套 suite 序列一致(可复现)
    cfg = default_config()
    s1 = SensorSuite(cfg.sensor, np.random.default_rng(7))
    s2 = SensorSuite(cfg.sensor, np.random.default_rng(7))
    dyn = AirshipDynamics(cfg, make_atmosphere(cfg.atmosphere))
    x = dyn.initial_state(p_ned_m=(0, 0, -1.0))
    for i in range(10):
        s1.sample_imu(x, i * 0.01)
        s2.sample_imu(x, i * 0.01)
        s1.sample_baro(x, i * 0.01)
        s2.sample_baro(x, i * 0.01)
    assert s1.meas.altitude_m == s2.meas.altitude_m
    assert np.array_equal(s1.meas.gyro_rad_s, s2.meas.gyro_rad_s)
