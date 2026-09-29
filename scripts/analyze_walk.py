#!/usr/bin/env python3
"""rosbag → 验证章四类指标 + 图表。

用法: analyze_walk.py <bag_dir> --cmd 0.3 [--out DIR]
指标:
  1. 基座速度跟踪   (/ground_truth/state vs cmd)
  2. 接触时序       (/cmd_contactFlag 规划 vs /simContactFlag 实际)
  3. 力矩跟踪       (/targetTorque vs /realTorque; /targetPos vs /jointsPosVel)
  4. 求解实时性     (/mpc_solve_time_ms, /wbc_solve_time_ms 分布 vs 预算)
产物: <out>/metrics.csv + summary.txt + fig1..fig4.png
"""
import argparse
import csv
import os
import sys

# rosbag2_py/rclpy 的 C 扩展依赖 ROS 共享库（LD_LIBRARY_PATH 在进程启动时
# 固化）：未 source ROS 时用同一解释器自举重入一次
if "AMENT_PREFIX_PATH" not in os.environ:
    _cmd = 'source /opt/ros/jazzy/setup.bash 2>/dev/null; exec python3 "$0" "$@"'
    os.execvp("bash", ["bash", "-c", _cmd, os.path.abspath(__file__)] + sys.argv[1:])

import numpy as np

# rosbag2_py 在 ROS2 的 site-packages 下：source 过 ROS 可直接 import，
# 否则按标准 Jazzy 路径自举（免去必须 source 的前置条件）
try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ModuleNotFoundError:
    import glob
    for p in glob.glob("/opt/ros/*/lib/python3*/site-packages"):
        sys.path.insert(0, p)
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

FOOT_NAMES = ["LTOE", "RTOE", "LHEEL", "RHEEL"]
MPC_BUDGET_MS = 10.0   # task_.info mpcDesiredFrequency 100 Hz
WBC_BUDGET_MS = 2.0    # mrtDesiredFrequency 500 Hz


def read_bag(bag_dir, topics):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("", ""))
    types = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in topics:
            continue
        if topic not in types:
            types[topic] = get_message(reader.get_all_topics_and_types()[
                [t.name for t in reader.get_all_topics_and_types()].index(topic)].type)
        yield topic, deserialize_message(data, types[topic])
    reader.close()


def load(bag_dir):
    want = {"/ground_truth/state", "/cmd_contactFlag", "/simContactFlag",
            "/targetTorque", "/realTorque", "/targetPos", "/jointsPosVel",
            "/mpc_solve_time_ms", "/wbc_solve_time_ms"}
    out = {t: [] for t in want}
    for topic, msg in read_bag(bag_dir, want):
        out[topic].append(msg)
    return out


def base_velocity_series(msgs):
    t, vx, vy, vz, height = [], [], [], [], []
    for m in msgs:
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        v = m.twist.twist.linear          # Odometry twist = body/world? 按 world 使用
        vx.append(v.x); vy.append(v.y); vz.append(v.z)
        height.append(m.pose.pose.position.z)
    return np.array(t), np.array(vx), np.array(vy), np.array(vz), np.array(height)


def flag_series(msgs):
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                  if hasattr(m, "header") else np.nan for m in msgs])
    arr = np.array([list(m.data) for m in msgs], dtype=float)
    return t, arr


def contact_metrics(cmd_msgs, sim_msgs):
    """逐接触点: 占空比、切换时刻误差、一致率。flag 数组长度 4。"""
    # 无 header 的 Int8MultiArray: 以序号对齐（同速率 500Hz 发布）
    if len(cmd_msgs) == 0 or len(sim_msgs) == 0:
        return [], np.zeros((0, 0)), np.zeros((0, 0))
    cmd = np.array([list(m.data) for m in cmd_msgs], dtype=float)
    sim = np.array([list(m.data) for m in sim_msgs], dtype=float)
    n = min(len(cmd), len(sim))
    cmd, sim = cmd[:n], sim[:n]
    rows = []
    for i, name in enumerate(FOOT_NAMES[:cmd.shape[1]]):
        duty_cmd, duty_sim = cmd[:, i].mean(), sim[:, i].mean()
        agree = (cmd[:, i] == sim[:, i]).mean()
        sc = np.flatnonzero(np.diff(cmd[:, i]) != 0) + 1   # 规划切换点(索引)
        ss = np.flatnonzero(np.diff(sim[:, i]) != 0) + 1   # 实际切换点(索引)
        k = min(len(sc), len(ss))
        sw_err = float(np.mean(np.abs(sc[:k] - ss[:k]))) if k else np.nan
        rows.append(dict(foot=name, duty_planned=duty_cmd, duty_actual=duty_sim,
                         agree_pct=100 * agree,
                         switch_err_idx=sw_err, n_switch_planned=len(sc),
                         n_switch_actual=len(ss)))
    return rows, cmd, sim


def torque_metrics(target_msgs, real_msgs):
    if len(target_msgs) == 0 or len(real_msgs) == 0:
        print("[warn] /targetTorque 或 /realTorque 为空（hwSwitch 未生效?）跳过力矩指标",
              file=sys.stderr)
        return dict(rms_per_joint=np.zeros(12), rms_total=np.nan, max_abs=np.nan, n=0)
    t = np.array([list(m.data) for m in target_msgs], dtype=float)
    r = np.array([list(m.data) for m in real_msgs], dtype=float)
    n = min(len(t), len(r))
    err = t[:n] - r[:n]
    return dict(rms_per_joint=np.sqrt((err ** 2).mean(axis=0)),
                rms_total=float(np.sqrt((err ** 2).mean())),
                max_abs=float(np.abs(err).max()), n=n)


def joint_pos_metrics(target_msgs, joints_msgs):
    if len(target_msgs) == 0 or len(joints_msgs) == 0:
        return dict(rms_per_joint=np.zeros(12), rms_total=np.nan, max_abs=np.nan)
    t = np.array([list(m.data)[:12] for m in target_msgs], dtype=float)
    q = np.array([list(m.data)[:12] for m in joints_msgs], dtype=float)
    n = min(len(t), len(q))
    err = t[:n] - q[:n]
    return dict(rms_per_joint=np.sqrt((err ** 2).mean(axis=0)),
                rms_total=float(np.sqrt((err ** 2).mean())),
                max_abs=float(np.abs(err).max()))


def solve_time_metrics(msgs, budget):
    x = np.array([m.data for m in msgs], dtype=float)
    if len(x) == 0:
        return None, x
    return dict(mean=float(x.mean()), p95=float(np.percentile(x, 95)),
                p99=float(np.percentile(x, 99)), max=float(x.max()),
                budget_ms=budget,
                over_budget_pct=100.0 * float((x > budget).mean())), x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--cmd", type=float, default=0.3, help="指令速度 (cmd_vel 归一化值)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out_dir = a.out or os.path.join(os.path.dirname(a.bag.rstrip("/")), "analysis")
    os.makedirs(out_dir, exist_ok=True)

    data = load(a.bag)

    # ---- 1. 基座速度 ----
    t, vx, vy, vz, h = base_velocity_series(data["/ground_truth/state"])
    vx_mean, vx_std = float(vx.mean()), float(vx.std())
    metrics = {"vx_mean": vx_mean, "vx_std": vx_std,
               "vy_mean": float(vy.mean()), "height_mean": float(h.mean()),
               "height_std": float(h.std()), "cmd_vx": a.cmd}

    # ---- 2. 接触时序 ----
    contact_rows, cmd_flags, sim_flags = contact_metrics(
        data["/cmd_contactFlag"], data["/simContactFlag"])

    # ---- 3. 力矩/关节跟踪 ----
    tau = torque_metrics(data["/targetTorque"], data["/realTorque"])
    q = joint_pos_metrics(data["/targetPos"], data["/jointsPosVel"])

    # ---- 4. 求解耗时 ----
    mpc_stats, mpc_ms = solve_time_metrics(data["/mpc_solve_time_ms"], MPC_BUDGET_MS)
    wbc_stats, wbc_ms = solve_time_metrics(data["/wbc_solve_time_ms"], WBC_BUDGET_MS)

    # ---- 输出 metrics.csv ----
    csv_path = os.path.join(out_dir, "metrics.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["category", "item", "value"])
        for k, v in metrics.items():
            w.writerow(["base_tracking", k, v])
        for row in contact_rows:
            for k, v in row.items():
                w.writerow(["contact_timing", f"{row['foot']}.{k}", v])
        for k in ("rms_total", "max_abs"):
            w.writerow(["torque_tracking", k, tau[k]])
        w.writerow(["joint_tracking", "rms_total", q["rms_total"]])
        w.writerow(["joint_tracking", "max_abs", q["max_abs"]])
        for name, st in (("mpc", mpc_stats), ("wbc", wbc_stats)):
            if st:
                for k, v in st.items():
                    w.writerow(["solve_time", f"{name}.{k}", v])

    # ---- summary.txt ----
    lines = [f"bag: {a.bag}", f"cmd_vx: {a.cmd}", "",
             f"[1] base tracking: vx = {vx_mean:.3f} ± {vx_std:.3f} m/s, "
             f"height = {metrics['height_mean']:.3f} ± {metrics['height_std']:.3f} m", "",
             "[2] contact timing:"]
    for row in contact_rows:
        lines.append(f"  {row['foot']}: duty {row['duty_planned']:.2f}->{row['duty_actual']:.2f}, "
                     f"agree {row['agree_pct']:.1f}%, switch_err {row['switch_err_idx']:.1f} idx, "
                     f"switches {row['n_switch_planned']}/{row['n_switch_actual']}")
    lines += ["", f"[3] torque tracking: rms {tau['rms_total']:.2f} Nm, max {tau['max_abs']:.1f} Nm",
              f"    joint pos: rms {q['rms_total']:.4f} rad, max {q['max_abs']:.3f} rad", "",
              "[4] solve time:"]
    for name, st, budget in (("MPC", mpc_stats, MPC_BUDGET_MS), ("WBC", wbc_stats, WBC_BUDGET_MS)):
        if st:
            lines.append(f"  {name}: mean {st['mean']:.2f} ms, p95 {st['p95']:.2f}, p99 {st['p99']:.2f}, "
                         f"max {st['max']:.2f} | budget {budget:.0f} ms, over {st['over_budget_pct']:.2f}%")
        else:
            lines.append(f"  {name}: (no data — 旧 bag 无求解耗时话题，需重录)")
    summary = "\n".join(lines)
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        f.write(summary + "\n")
    print(summary)

    # ---- 图表 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # fig1 基座速度
    fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    if len(t):
        ax[0].plot(t - t[0], vx, label="vx actual", lw=1.2)
        ax[0].axhline(a.cmd, color="r", ls="--", lw=1, label="cmd")
        ax[0].set_ylabel("vx [m/s]"); ax[0].legend(); ax[0].grid(alpha=.3)
        ax[1].plot(t - t[0], h, lw=1.2)
        ax[1].set_ylabel("base height [m]"); ax[1].set_xlabel("time [s]"); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig1_base_tracking.png"), dpi=300)

    # fig2 接触时序
    if len(cmd_flags):
        fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
        n = min(len(cmd_flags), len(sim_flags))
        idx = np.arange(n)
        for i, name in enumerate(FOOT_NAMES[:cmd_flags.shape[1]]):
            ax[0].plot(idx, cmd_flags[:n, i] + 0.12 * i, lw=1, label=name)
            ax[1].plot(idx, sim_flags[:n, i] + 0.12 * i, lw=1, label=name)
        ax[0].set_title("planned contact (/cmd_contactFlag)")
        ax[1].set_title("actual contact (/simContactFlag)")
        for aa in ax:
            aa.set_yticks([]); aa.legend(loc="upper right", fontsize=7); aa.grid(alpha=.3)
        ax[1].set_xlabel("sample index (500 Hz)")
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig2_contact_timing.png"), dpi=300)

    # fig3 力矩跟踪
    if tau["n"]:
        fig, ax = plt.subplots(1, 2, figsize=(8, 3.5))
        ax[0].bar(np.arange(len(tau["rms_per_joint"])), tau["rms_per_joint"])
        ax[0].set_xlabel("joint"); ax[0].set_ylabel("torque RMS err [Nm]"); ax[0].grid(alpha=.3)
        ax[1].bar(np.arange(len(q["rms_per_joint"])), q["rms_per_joint"])
        ax[1].set_xlabel("joint"); ax[1].set_ylabel("joint pos RMS err [rad]"); ax[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig3_tracking_errors.png"), dpi=300)

    # fig4 求解耗时
    fig, ax = plt.subplots(1, 2, figsize=(8, 3.5))
    for aa, x, budget, name in ((ax[0], mpc_ms, MPC_BUDGET_MS, "MPC"),
                                (ax[1], wbc_ms, WBC_BUDGET_MS, "WBC")):
        if len(x):
            aa.hist(x, bins=60)
            aa.axvline(budget, color="r", ls="--", label=f"budget {budget:.0f} ms")
            aa.set_title(f"{name} solve time ({len(x)} samples)")
            aa.set_xlabel("ms"); aa.legend(); aa.grid(alpha=.3)
        else:
            aa.set_title(f"{name}: no data")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig4_solve_time.png"), dpi=300)
    print(f"\n[analyze] outputs -> {out_dir}")


if __name__ == "__main__":
    main()
