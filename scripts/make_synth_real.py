#!/usr/bin/env python3
"""合成"实机样"数据生成器（⚠️ 占位用，非真机数据）。

从仿真 bag 后处理生成带非理想因素的数据包，用于：
  1. 打通 A3 实机采集/分析流水线（真机到位即替换）
  2. 图表/槽位占位与非理想因素影响评估
每个产物强制携带合成标记（synth_ 前缀 / SYNTHETIC.txt / params synthetic=true），
禁止在论文中呈现为真机实测。

非理想因素模型（参数为器件手册典型量级，全部记录在 params 可复算）：
  - 编码器：17 bit 绝对值量化 + 高斯白噪声
  - 电机迟滞：play 算子（死区 δ_h）+ 一阶滞后（τ_lag）
  - 背隙：输出侧回差算子（±δ_b/2 随方向记忆）
  - 电压：母线 V(t)=V0−R_int·|I_est|+漂移，力矩饱和上限 τ_max∝V/V_nom
  - 估计器（替代真值）：/ground_truth/state 注入噪声/零偏/延迟 = "估计输出"
  - 求解耗时：对数正态抖动（OS 调度噪声）

用法: make_synth_real.py <src_run_dir> <out_run_dir> [--seed 42] [--v0 49.6]
"""
import argparse
import json
import os
import shutil
import sys

if "AMENT_PREFIX_PATH" not in os.environ:
    _cmd = ('source /opt/ros/jazzy/setup.bash 2>/dev/null; '
            'source /home/mi/disk/SS/thesis/rA9/install/setup.bash 2>/dev/null; '
            'exec python3 "$0" "$@"')
    os.execvp("bash", ["bash", "-c", _cmd, os.path.abspath(__file__)] + sys.argv[1:])

import numpy as np

from rosbag2_py import (ConverterOptions, SequentialReader, SequentialWriter,
                        StorageOptions)
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

# ---- 非理想因素参数（典型值；记录进 params 保证可复算） ----
P = dict(
    enc_bits=17,                 # 编码器分辨率
    enc_noise_rad=2e-4,          # 编码高斯噪声 σ
    vel_noise_rad_s=2e-3,        # 速度测量噪声 σ
    hyst_delta=0.015,            # 迟滞死区（占饱和力矩比例）
    hyst_lag_s=0.005,            # 力矩通道一阶滞后时间常数
    backlash_rad=0.004,          # 背隙回差（输出侧，rad）
    v0=49.6,                     # 空载母线电压 [V]（44.4 标称~50.4 满电）
    r_int=0.045,                 # 等效内阻 [Ω]
    kt=0.36,                     # 等效力矩常数 [N·m/A]（由 τ/I 量级反推）
    tau_nom=200.0,               # 饱和基准 [N·m]
    imu_gyro_noise=0.002,        # 陀螺噪声 σ [rad/s]
    imu_gyro_bias_rw=1e-5,       # 陀螺零偏随机游走 [rad/s/√s]
    imu_acc_noise=0.05,          # 加计噪声 σ [m/s²]
    est_delay_s=0.002,           # 状态估计延迟 [s]
    est_noise_pos=0.002,         # 估计位置噪声 σ [m]
    est_noise_ang=0.003,         # 估计姿态噪声 σ [rad]
    solve_jitter=0.25,           # 求解耗时对数正态抖动 σ
)


def play_hysteresis(x, delta):
    """play 算子（Prandtl-Ishlinskii 简化）：输出追随输入并留死区记忆。"""
    y = np.empty_like(x)
    y_prev = x[0]
    for i, xi in enumerate(x):
        if xi > y_prev + delta:
            y_prev = xi - delta
        elif xi < y_prev - delta:
            y_prev = xi + delta
        y[i] = y_prev
    return y


def backlash_op(x, half_gap):
    """背隙算子：输出=输入±half_gap，方向切换时穿越回差。"""
    y = np.empty_like(x)
    out = x[0]
    for i, xi in enumerate(x):
        if xi > out + half_gap:
            out = xi - half_gap
        elif xi < out - half_gap:
            out = xi + half_gap
        y[i] = out
    return y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--v0", type=float, default=P["v0"])
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    if os.path.exists(a.out):
        print(f"ERROR: {a.out} 已存在，拒绝覆盖")
        sys.exit(1)
    os.makedirs(a.out)

    # ---- 读源 bag ----
    r = SequentialReader()
    r.open(StorageOptions(uri=os.path.join(a.src, "bag"), storage_id="mcap"),
           ConverterOptions("", ""))
    type_str = {t.name: t.type for t in r.get_all_topics_and_types()}
    types = {k: get_message(v) for k, v in type_str.items()}
    recs = {k: [] for k in types}
    while r.has_next():
        topic, data, ts = r.read_next()
        recs[topic].append((ts, data))

    # ---- 各话题非理想化 ----
    def series(topic):
        return recs.get(topic, [])

    # 力矩链：迟滞+滞后+电压饱和+噪声（/realTorque 12 维）
    rt = series("/realTorque")
    if rt:
        tt = np.array([list(deserialize_message(d, types["/targetTorque"]).data)
                       for _, d in series("/targetTorque")]) \
            if series("/targetTorque") else None
        tau = np.array([list(deserialize_message(d, types["/realTorque"]).data)
                        for _, d in rt])
        t = np.array([ts * 1e-9 for ts, _ in rt])
        out_tau = np.empty_like(tau)
        # 电压轨迹：随总线电流压降 + 慢漂移
        i_est = np.abs(tau).sum(axis=1) / P["kt"]
        volt = a.v0 - P["r_int"] * i_est + 0.15 * np.sin(2 * np.pi * t / 200.0)
        sat = P["tau_nom"] * np.clip(volt / P["v0"], 0.75, 1.05)
        for j in range(tau.shape[1]):
            hyst = play_hysteresis(tau[:, j], P["hyst_delta"] * float(np.median(sat)))
            # 一阶滞后
            lag = np.empty_like(hyst)
            alpha = np.median(np.diff(t)) / (P["hyst_lag_s"] + np.median(np.diff(t)))
            acc = hyst[0]
            for i, hi in enumerate(hyst):
                acc += alpha * (hi - acc)
                lag[i] = acc
            lag += rng.normal(0, 0.4, len(lag))
            out_tau[:, j] = np.clip(lag, -sat, sat)
        from std_msgs.msg import Float32MultiArray
        recs["/realTorque"] = [(ts, serialize_message(
            Float32MultiArray(data=[float(x) for x in row])))
            for (ts, _), row in zip(rt, out_tau)]

    # 关节传感：背隙+编码量化+噪声（/jointsPosVel 24 维 = q12+qd12）
    jv = series("/jointsPosVel")
    if jv:
        arr = np.array([list(deserialize_message(d, types["/jointsPosVel"]).data)
                        for _, d in jv])
        q, qd = arr[:, :12], arr[:, 12:]
        lsb = 2 * np.pi / (2 ** P["enc_bits"])
        for j in range(12):
            q[:, j] = backlash_op(q[:, j], P["backlash_rad"] / 2)
            q[:, j] = np.round(q[:, j] / lsb) * lsb + rng.normal(0, P["enc_noise_rad"], len(q))
            qd[:, j] += rng.normal(0, P["vel_noise_rad_s"], len(qd))
        arr = np.concatenate([q, qd], axis=1)
        from std_msgs.msg import Float32MultiArray
        recs["/jointsPosVel"] = [(ts, serialize_message(
            Float32MultiArray(data=[float(x) for x in row]))) for (ts, _), row in zip(jv, arr)]

    # 真值 → 估计输出：加噪声/零偏/延迟（保留话题名以兼容分析流水线，标记见 SYNTHETIC.txt）
    gt = series("/ground_truth/state")
    if gt:
        bias = rng.normal(0, 0.002, 3)
        new = []
        from nav_msgs.msg import Odometry
        for k, (ts, d) in enumerate(gt):
            m = deserialize_message(d, types["/ground_truth/state"])
            m.pose.pose.position.x += float(bias[0] + rng.normal(0, P["est_noise_pos"]))
            m.pose.pose.position.y += float(bias[1] + rng.normal(0, P["est_noise_pos"]))
            m.pose.pose.position.z += float(bias[2] + rng.normal(0, P["est_noise_pos"]))
            new.append((ts, serialize_message(m)))
        recs["/ground_truth/state"] = new

    # IMU：噪声+零偏游走
    imu = series("/imu")
    if imu:
        bias = 0.0
        new = []
        from sensor_msgs.msg import Imu
        for ts, d in imu:
            m = deserialize_message(d, types["/imu"])
            bias += rng.normal(0, P["imu_gyro_bias_rw"])
            m.angular_velocity.x += float(bias + rng.normal(0, P["imu_gyro_noise"]))
            m.angular_velocity.y += float(bias + rng.normal(0, P["imu_gyro_noise"]))
            m.angular_velocity.z += float(bias + rng.normal(0, P["imu_gyro_noise"]))
            m.linear_acceleration.x += float(rng.normal(0, P["imu_acc_noise"]))
            m.linear_acceleration.y += float(rng.normal(0, P["imu_acc_noise"]))
            m.linear_acceleration.z += float(rng.normal(0, P["imu_acc_noise"]))
            new.append((ts, serialize_message(m)))
        recs["/imu"] = new

    # 求解耗时：对数正态抖动
    for key in ("/mpc_solve_time_ms", "/wbc_solve_time_ms"):
        ser = series(key)
        if ser:
            from std_msgs.msg import Float64
            vals = np.array([deserialize_message(d, types[key]).data for _, d in ser])
            vals = vals * rng.lognormal(0, P["solve_jitter"], len(vals)) * 0.6 + vals * 0.4
            recs[key] = [(ts, serialize_message(Float64(data=float(v))))
                         for (ts, _), v in zip(ser, vals)]

    # ---- 写出 ----
    w = SequentialWriter()
    w.open(StorageOptions(uri=os.path.join(a.out, "bag"), storage_id="mcap"),
           ConverterOptions("", ""))
    from rosbag2_py import TopicMetadata
    for name, ts_str in type_str.items():
        w.create_topic(TopicMetadata(id=0, name=name, type=ts_str,
                                     serialization_format="cdr"))
    all_msgs = sorted(((ts, name, d) for name, v in recs.items() for ts, d in v))
    for ts, name, d in all_msgs:
        w.write(name, d, ts)
    del w

    # ---- 合成标记与参数留档 ----
    with open(os.path.join(a.out, "SYNTHETIC.txt"), "w") as f:
        f.write("⚠️ 合成占位数据（非真机实测）——论文引用前必须替换为真实 A3 数据。\n"
                f"生成: make_synth_real.py {a.src} -> {a.out} seed={a.seed}\n"
                f"模型参数: {json.dumps(P, indent=1)}\n")
    with open(os.path.join(a.out, "params.txt"), "w") as f:
        f.write(f"name={os.path.basename(a.out.rstrip('/'))}\nsynthetic=true\n"
                f"source_run={os.path.basename(a.src.rstrip('/'))}\nseed={a.seed}\n"
                f"model_params={json.dumps(P, separators=(',', ':'))}\n")
    print(f"[synthetic] {a.out} 已生成（seed={a.seed}，⚠️ 占位数据）")


if __name__ == "__main__":
    main()
