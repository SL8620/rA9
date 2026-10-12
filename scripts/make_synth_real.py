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
  【2026-10-12 新增】
  - 延迟链路：指令侧 delay_cmd_ms（/target* 五话题 + /controlInput）与观测侧
    delay_obs_ms（/jointsPosVel /imu /ground_truth/state /realTorque
    /simContactFlag）**分开建模**。按时间域最近邻移位，不按样本数移位
    （录包时间戳突发投递，见 NOTE.md §12.3）。
    /cmd_contactFlag 是**规划**不延迟，/simContactFlag 是**测量**延迟 ——
    两者之差正是 switch_err 要度量的，故延迟它会让 switch_err 反映真实时延。
  - 力矩-转速包络：τ_max(ω) = min(τ_rated, τ_stall·(1−|ω|/ω_nl))，ω 取
    /jointsPosVel 12:24 列，按时间域最近邻对齐到力矩时间戳；与电压跌落项取 min。

**开环近似声明（务必与论文 6.6 局限性一致）**：本工具只叠加**信号特性**，
轨迹仍是理想执行器跑出来的。故产物 = "理想执行器轨迹 + 非理想信号特性"，
**不是**"真实执行器下会产生的轨迹"。指令延迟移位后运动仍是无延迟控制器的输出
（闭环反馈效应缺失）；摩擦/背隙只能作为力矩损失叠加，不能反馈回运动。

用法:
  make_synth_real.py <src_run_dir> <out_run_dir> [--seed 42] [--v0 49.6]
                     [--gaps 名1,名2 | --no-gaps 名1,名2]
  --gaps 只开指定 gap，--no-gaps 关掉指定 gap（其余全开）。缺省全开。
  gap 名见 GAPS 字典。
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
# 逐项标注来源：[估计]=器件手册典型量级估计值（真机参数到位后可替换）
#              [手册]=器件手册值  [实测]=标定/实测值
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
    # ---- 2026-10-12 新增：延迟链路 + 力矩-转速包络 ----
    delay_cmd_ms=5.0,            # 执行侧指令延迟 [ms]（EtherCAT+下发+电流环建立）
    delay_obs_ms=3.0,            # 观测侧延迟 [ms]（滤波+采集+通信）
    tau_stall_ratio=1.35,        # 堵转力矩 / 额定力矩（包络在高速段才起作用）
    omega_nl=8.0,                # 关节侧空载转速 [rad/s]（= 电机空载转速/减速比）
)

P_SRC = dict(
    enc_bits="[手册]", enc_noise_rad="[估计]", vel_noise_rad_s="[估计]",
    hyst_delta="[估计]", hyst_lag_s="[估计]", backlash_rad="[估计]",
    v0="[估计]", r_int="[估计]", kt="[估计]", tau_nom="[实测]task_.info torqueLimitsTask",
    imu_gyro_noise="[估计]", imu_gyro_bias_rw="[估计]", imu_acc_noise="[估计]",
    est_delay_s="[未使用] 由 delay_obs_ms 统一建模（历史遗留参数，勿引用）",
    est_noise_pos="[估计]",
    est_noise_ang="[未使用] 姿态噪声未施加（历史遗留参数，勿引用）",
    solve_jitter="[估计]",
    delay_cmd_ms="[估计] EtherCAT 1ms + 电流环 ~2-3ms + 下发抖动",
    delay_obs_ms="[估计] 互补滤波 2-5ms + 采集/通信",
    tau_stall_ratio="[估计] 电机堵转/额定典型 1.2~1.5",
    omega_nl=("[估计] 由膝峰值 320N·m 反推减速比 i≈40~80，电机空载 3000rpm(314rad/s)"
              " → 关节侧 ω_nl≈4~8 rad/s，取上界 8。注：实测仿真膝 10.4/踝 24.5 rad/s "
              "已超过该值 → 理想执行器跑出真机做不到的速度（本身即 sim2real gap）"),
)

# gap 注册表：--gaps/--no-gaps 用的名字 → 说明。灵敏度分析按此逐项开关。
GAPS = {
    "delay_cmd":    "执行侧指令延迟 delay_cmd_ms",
    "delay_obs":    "观测侧延迟 delay_obs_ms",
    "torque_env":   "力矩-转速包络 τ_max(ω)",
    "hyst_lag":     "电机迟滞(play) + 一阶滞后",
    "backlash":     "背隙",
    "encoder":      "编码器量化 + 噪声",
    "imu":          "IMU 噪声 + 零偏游走",
    "est":          "状态估计噪声（替代真值）",
    "voltage":      "母线电压跌落 + 力矩饱和",
    "solve_jitter": "求解耗时抖动",
}
ALL_GAPS = list(GAPS)


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


def delay_index(t, tau_s):
    """延迟 tau_s 的最近邻索引：输出[i] = 源[idx[i]]，有效样本由 valid 给出。

    按时间域移位，**不按样本数移位** —— 录包时间戳是突发投递（中位间隔
    0.127 ms、均值 0.45 ms、最大间隙 335 ms，见 NOTE.md §12.3），按样本数
    移位的等效延迟会漂。不插值，避免造数据。

    前 tau_s 秒无源数据 → valid=False（延迟缓冲区为空，输出缺这段）。
    """
    t = np.asarray(t, dtype=float)
    if len(t) == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=bool)
    if tau_s == 0:
        return np.arange(len(t)), np.ones(len(t), dtype=bool)
    tgt = t - tau_s
    if len(t) == 1:
        return np.zeros(1, dtype=int), np.array([tgt[0] >= t[0]])
    i = np.clip(np.searchsorted(t, tgt), 1, len(t) - 1)
    left = np.abs(tgt - t[i - 1]) <= np.abs(tgt - t[i])
    return np.where(left, i - 1, i), tgt >= t[0]


def torque_envelope(omega, tau_rated, tau_stall_ratio, omega_nl):
    """力矩-转速包络：τ_max(ω) = min(τ_rated, τ_stall·(1 − |ω|/ω_nl))。

    omega: (..., 12) 关节角速度 [rad/s]；tau_rated: (12,) 额定/限幅力矩。
    包络在 |ω|→ω_nl 时线性降到 0（电压/电流限制的常用近似）。
    """
    omega = np.asarray(omega, dtype=float)
    tau_rated = np.asarray(tau_rated, dtype=float)
    head = tau_stall_ratio * tau_rated * np.clip(
        1.0 - np.abs(omega) / float(omega_nl), 0.0, 1.0)
    return np.minimum(tau_rated, head)


def read_torque_limits():
    """逐关节额定力矩 [N·m]，真源 task_.info torqueLimitsTask（12 维）。"""
    import re
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "src/humanoid_control/humanoid_interface/config/mpc/task_.info")
    try:
        txt = open(p, encoding="utf-8").read()
        blk = re.search(r"torqueLimitsTask\s*\{(.*?)\}", txt, re.S).group(1)
        d = {int(i): float(v) for i, v in
             re.findall(r"\(\s*(\d)\s*,\s*0\s*\)\s+([\d.]+)", blk)}
        return np.array([d[i % 6] for i in range(12)], float)
    except Exception:
        return np.array([200, 200, 320, 320, 200, 200] * 2, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--v0", type=float, default=P["v0"])
    ap.add_argument("--gaps", default="",
                    help=f"只开这些 gap（逗号分隔）。可选: {','.join(ALL_GAPS)}")
    ap.add_argument("--no-gaps", default="",
                    help="关掉这些 gap（其余全开）")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    on = set(ALL_GAPS)
    if a.gaps:
        want = {s.strip() for s in a.gaps.split(",") if s.strip()}
        bad = want - on
        if bad:
            print(f"ERROR: 未知 gap {sorted(bad)}；可选 {ALL_GAPS}")
            sys.exit(2)
        on = want
    if a.no_gaps:
        off = {s.strip() for s in a.no_gaps.split(",") if s.strip()}
        bad = off - on
        if bad:
            print(f"ERROR: 未知 gap {sorted(bad)}；可选 {ALL_GAPS}")
            sys.exit(2)
        on -= off
    P["v0"] = a.v0
    TAU_RATED = read_torque_limits()

    if os.path.exists(a.out):
        print(f"ERROR: {a.out} 已存在，拒绝覆盖")
        sys.exit(1)
    os.makedirs(a.out)
    dropped = {}          # 延迟造成的前缘丢样本数，落盘 params

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

    from std_msgs.msg import Float32MultiArray

    # 先取**未退化**的关节角速度，供力矩-转速包络用（包络由真实速度决定）
    jv = series("/jointsPosVel")
    jv_t = np.array([ts * 1e-9 for ts, _ in jv]) if jv else np.zeros(0)
    jv_arr = (np.array([list(deserialize_message(d, types["/jointsPosVel"]).data)
                        for _, d in jv]) if jv else np.zeros((0, 24)))

    # 力矩链：迟滞+滞后 → 力矩-转速包络 → 电压饱和（/realTorque 12 维）
    rt = series("/realTorque")
    if rt:
        tau = np.array([list(deserialize_message(d, types["/realTorque"]).data)
                        for _, d in rt])
        t = np.array([ts * 1e-9 for ts, _ in rt])
        out_tau = tau.copy()

        if "hyst_lag" in on:
            sat_ref = float(P["tau_nom"])
            for j in range(tau.shape[1]):
                hyst = play_hysteresis(out_tau[:, j], P["hyst_delta"] * sat_ref)
                lag = np.empty_like(hyst)
                alpha = np.median(np.diff(t)) if len(t) > 1 else 1e-3
                alpha = alpha / (P["hyst_lag_s"] + alpha)
                acc = hyst[0]
                for i, hi in enumerate(hyst):
                    acc += alpha * (hi - acc)
                    lag[i] = acc
                out_tau[:, j] = lag + rng.normal(0, 0.4, len(lag))

        # 力矩-转速包络 τ_max(ω) = min(τ_rated, τ_stall·(1−|ω|/ω_nl))
        if "torque_env" in on and len(jv_t):
            def nn(st, sv, dt):
                if len(st) == 0:
                    return np.zeros((len(dt), sv.shape[1]))
                i = np.clip(np.searchsorted(st, dt), 1, len(st) - 1)
                left = np.abs(dt - st[i - 1]) <= np.abs(dt - st[i])
                return sv[np.where(left, i - 1, i)]
            omega = nn(jv_t, jv_arr[:, 12:24], t)      # 时间域最近邻对齐
            env = torque_envelope(omega, TAU_RATED,
                                  P["tau_stall_ratio"], P["omega_nl"])
        else:
            env = np.broadcast_to(TAU_RATED, out_tau.shape).copy()

        # 电压轨迹：随总线电流压降 + 慢漂移 → 与包络取 min
        lim = env
        if "voltage" in on:
            i_est = np.abs(tau).sum(axis=1) / P["kt"]
            volt = P["v0"] - P["r_int"] * i_est + 0.15 * np.sin(2 * np.pi * t / 200.0)
            sat_v = P["tau_nom"] * np.clip(volt / P["v0"], 0.75, 1.05)[:, None]
            lim = np.minimum(lim, sat_v)

        out_tau = np.clip(out_tau, -lim, lim)
        recs["/realTorque"] = [(ts, serialize_message(
            Float32MultiArray(data=[float(x) for x in row])))
            for (ts, _), row in zip(rt, out_tau)]

    # 关节传感：背隙+编码量化+噪声（/jointsPosVel 24 维 = q12+qd12）
    if jv:
        arr = jv_arr.copy()
        q, qd = arr[:, :12], arr[:, 12:]
        if "backlash" in on:
            for j in range(12):
                q[:, j] = backlash_op(q[:, j], P["backlash_rad"] / 2)
        if "encoder" in on:
            lsb = 2 * np.pi / (2 ** P["enc_bits"])
            for j in range(12):
                q[:, j] = (np.round(q[:, j] / lsb) * lsb
                           + rng.normal(0, P["enc_noise_rad"], len(q)))
                qd[:, j] += rng.normal(0, P["vel_noise_rad_s"], len(qd))
        arr = np.concatenate([q, qd], axis=1)
        recs["/jointsPosVel"] = [(ts, serialize_message(
            Float32MultiArray(data=[float(x) for x in row])))
            for (ts, _), row in zip(jv, arr)]

    # 真值 → 估计输出：加噪声/零偏/延迟（保留话题名以兼容分析流水线，标记见 SYNTHETIC.txt）
    gt = series("/ground_truth/state")
    if gt and "est" in on:
        bias = rng.normal(0, 0.002, 3)
        new = []
        from nav_msgs.msg import Odometry
        for ts, d in gt:
            m = deserialize_message(d, types["/ground_truth/state"])
            m.pose.pose.position.x += float(bias[0] + rng.normal(0, P["est_noise_pos"]))
            m.pose.pose.position.y += float(bias[1] + rng.normal(0, P["est_noise_pos"]))
            m.pose.pose.position.z += float(bias[2] + rng.normal(0, P["est_noise_pos"]))
            new.append((ts, serialize_message(m)))
        recs["/ground_truth/state"] = new

    # IMU：噪声+零偏游走
    imu = series("/imu")
    if imu and "imu" in on:
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
    if "solve_jitter" in on:
        for key in ("/mpc_solve_time_ms", "/wbc_solve_time_ms"):
            ser = series(key)
            if ser:
                from std_msgs.msg import Float64
                vals = np.array([deserialize_message(d, types[key]).data for _, d in ser])
                vals = (vals * rng.lognormal(0, P["solve_jitter"], len(vals)) * 0.6
                        + vals * 0.4)
                recs[key] = [(ts, serialize_message(Float64(data=float(v))))
                             for (ts, _), v in zip(ser, vals)]

    # ---- 延迟链路（最后施加：传输效应作用于最终信号）----
    # 时间域最近邻移位，不插值；前 τ 秒无源数据 → 丢弃（延迟缓冲区为空）
    # 分组口径：
    #   delay_cmd = 执行器侧路径，指令**到达**执行器所需时间 → 执行器输入信号
    #   delay_obs = 传感侧路径，测量**到达**记录端所需时间 → 全部测量信号
    # /controlInput 是执行器输入（=data.ctrl）故走 cmd；/realTorque 是执行器
    # 输出的**测量**故走 obs。
    # /simContactFlag 是足底接触的**测量**（真机有开关去抖+传感延迟）故走 obs；
    # /cmd_contactFlag 是**规划**（不经过传感），故不延迟 —— 两者差正是
    # switch_err 要度量的东西，延迟它会让 switch_err 反映真实时延。
    CMD_TOPICS = ["/targetTorque", "/targetPos", "/targetVel",
                  "/targetKp", "/targetKd", "/controlInput"]
    OBS_TOPICS = ["/jointsPosVel", "/imu", "/ground_truth/state",
                  "/realTorque", "/simContactFlag"]
    delay_plan = []
    if "delay_cmd" in on:
        delay_plan += [(k, P["delay_cmd_ms"] * 1e-3) for k in CMD_TOPICS]
    if "delay_obs" in on:
        delay_plan += [(k, P["delay_obs_ms"] * 1e-3) for k in OBS_TOPICS]

    for topic, tau_s in delay_plan:
        ser = series(topic)
        if not ser or tau_s == 0:
            continue
        t = np.array([ts * 1e-9 for ts, _ in ser])
        # 只做**记录级重排**：取 t−τ 处最近邻的原始序列化字节，消息类型
        # 原样保留（/imu 是 sensor_msgs/Imu、/ground_truth/state 是
        # nav_msgs/Odometry、/sim_time 是 Float64，不能一律当 Float32MultiArray）
        idx, valid = delay_index(t, tau_s)
        n_drop = int((~valid).sum())
        if n_drop:
            dropped[f"{topic}@{tau_s*1e3:.1f}ms"] = n_drop
        recs[topic] = [(ts, ser[j][1])
                       for i, ((ts, _), j) in enumerate(zip(ser, idx)) if valid[i]]

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
    gap_desc = {k: GAPS[k] for k in ALL_GAPS if k in on}
    src_desc = {k: P_SRC.get(k, "[未标]") for k in P}
    with open(os.path.join(a.out, "SYNTHETIC.txt"), "w") as f:
        f.write("⚠️ 合成占位数据（非真机实测）——论文引用前必须替换为真实 A3 数据。\n"
                "用途限定：灵敏度分析 / 流水线验证 / 图表占位，**不得作为实测报出**。\n"
                "开环近似：本产物 = 理想执行器轨迹 + 非理想信号特性，\n"
                "         **不是**真实执行器下会产生的轨迹（闭环反馈效应缺失）。\n"
                f"生成: make_synth_real.py {a.src} -> {a.out} seed={a.seed}\n"
                f"启用 gap: {json.dumps(gap_desc, ensure_ascii=False, indent=1)}\n"
                f"模型参数: {json.dumps(P, indent=1, ensure_ascii=False)}\n"
                f"参数来源: {json.dumps(src_desc, indent=1, ensure_ascii=False)}\n"
                f"延迟前缘丢弃样本: {json.dumps(dropped) if dropped else '无'}\n")
    with open(os.path.join(a.out, "params.txt"), "w") as f:
        f.write(f"name={os.path.basename(a.out.rstrip('/'))}\n"
                f"synthetic=true\n"
                f"model=open_loop_approximation\n"
                f"source_run={os.path.basename(a.src.rstrip('/'))}\n"
                f"seed={a.seed}\n"
                f"gaps_on={','.join(sorted(on))}\n"
                f"gaps_off={','.join(sorted(set(ALL_GAPS) - on))}\n"
                f"model_params={json.dumps(P, separators=(',', ':'), ensure_ascii=False)}\n"
                f"param_sources={json.dumps(src_desc, separators=(',', ':'), ensure_ascii=False)}\n"
                f"delay_edge_dropped={json.dumps(dropped, separators=(',', ':')) if dropped else '{}'}\n")
    print(f"[synthetic] {a.out} 已生成（seed={a.seed}，⚠️ 占位数据，开环近似）")
    print(f"[synthetic]   启用 gap: {','.join(sorted(on)) or '（无）'}")
    if dropped:
        print(f"[synthetic]   延迟前缘丢弃: {dropped}")


if __name__ == "__main__":
    main()
