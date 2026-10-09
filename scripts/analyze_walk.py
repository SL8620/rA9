#!/usr/bin/env python3
"""rosbag → 验证章四类指标 + 图表。

用法: analyze_walk.py <bag_dir> --cmd 0.3 [--out DIR] [--band 0.05] [--full]
指标:
  1. 基座速度/高度跟踪 (/ground_truth/state vs cmd)  mean±std + RMSE
  2. 接触时序          (/cmd_contactFlag 规划 vs /simContactFlag 实际)
  3. 力矩/关节跟踪     (/targetTorque vs /realTorque; /targetPos vs /jointsPosVel)
  4. 求解实时性        (/mpc_solve_time_ms, /wbc_solve_time_ms 分布 vs 预算)
产物: <out>/metrics.csv + summary.txt + fig1..fig4.png

口径（与论文 6.3.1 一致）:
  * 时间轴一律用 **bag 时间戳**（reader.read_next 第三返回值）。
    Int8MultiArray/Float32MultiArray 无 header，消息内无时间戳；
    各话题速率不同（实测 /cmd_contactFlag 25.5万 vs /simContactFlag 14.2万条），
    早期按序号截断对齐会把站立段当成全程，导致 duty/switch 全错。
  * 稳态窗: 起于「指令后进入目标带并保持 dwell 秒」，止于「录制末 − 0.5 s」。
    计算用原始数据，滤波仅用于图。
  * 单位: m / rad / N·m / ms。--cmd 单位 m/s（/cmd_vel 直接就是目标速度，
    TargetTrajectoriesPublisher 不做缩放）。
  * 含 nan 或缺关键话题的 run 在 summary 顶部标 STATUS: INVALID(<原因>)。
"""
import argparse
import csv
import math
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

FOOT_NAMES = ["L", "R"]                  # 规划侧 2 元素 = [L, R] 整足
POINT_NAMES = ["LTOE", "RTOE", "LHEEL", "RHEEL"]   # 实测侧 4 元素 = 逐接触点
MPC_BUDGET_MS = 10.0   # task_.info mpcDesiredFrequency 100 Hz
WBC_BUDGET_MS = 2.0    # mrtDesiredFrequency 500 Hz
TAIL_TRIM_S = 0.5      # 稳态窗末端回缩
DWELL_S = 0.3          # 进入目标带需保持时长

WANT = {"/ground_truth/state", "/cmd_contactFlag", "/simContactFlag",
        "/targetTorque", "/realTorque", "/targetPos", "/jointsPosVel",
        "/mpc_solve_time_ms", "/wbc_solve_time_ms", "/cmd_vel"}


# ---------------------------------------------------------------- IO
def read_bag(bag_dir, topics):
    """yield (topic, t_sec, msg)。t_sec 是 bag 时间戳（非消息 header）。"""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("", ""))
    all_topics = reader.get_all_topics_and_types()
    types = {t.name: get_message(t.type) for t in all_topics if t.name in topics}
    while reader.has_next():
        topic, data, ts = reader.read_next()      # ts = 錄製時間戳 [ns]
        if topic not in types:
            continue
        yield topic, ts * 1e-9, deserialize_message(data, types[topic])
    reader.close()


def extract(msg):
    """把消息拍平成数值向量。不同话题的消息类型不同：
       * *MultiArray / Float64 / Bool  → msg.data
       * nav_msgs/Odometry             → [v.x,v.y,v.z, p.x,p.y,p.z, roll, pitch]
       * geometry_msgs/Twist           → [l.x,l.y,l.z, a.z]
    """
    d = getattr(msg, "data", None)
    if d is not None:
        return [float(x) for x in d] if hasattr(d, "__iter__") else [float(d)]
    if hasattr(msg, "pose") and hasattr(msg, "twist"):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        q = msg.pose.pose.orientation
        # 四元数 → 横滚/俯仰 [rad]（倒地判定用；退化四元数给 nan，比较时自动跳过）
        n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if n > 0.5:
            roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
            sp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
            pitch = math.asin(sp)
        else:
            roll = pitch = float("nan")
        return [v.x, v.y, v.z, p.x, p.y, p.z, roll, pitch]
    if hasattr(msg, "linear") and hasattr(msg, "angular"):
        return [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
    return None


def load(bag_dir):
    """{topic: (t[N], v[N,D])}，v 为 float 二维。"""
    t, v = {k: [] for k in WANT}, {k: [] for k in WANT}
    for topic, ts, msg in read_bag(bag_dir, WANT):
        row = extract(msg)
        if row is None:
            continue
        t[topic].append(ts)
        v[topic].append(row)
    out = {}
    for k in WANT:
        tt = np.asarray(t[k], dtype=float)
        vv = np.asarray(v[k], dtype=float)
        if vv.size == 0:
            vv = np.zeros((0, 1))
        elif vv.ndim == 1:
            vv = vv.reshape(-1, 1)
        out[k] = (tt, vv)
    return out


# ---------------------------------------------------------------- 时间域工具
def nearest_align(src_t, src_v, dst_t):
    """把 src 序列按 dst 时间戳做最近邻采样 → (len(dst_t), D)。"""
    if len(src_t) == 0 or len(dst_t) == 0:
        return np.zeros((0, src_v.shape[1] if src_v.ndim == 2 else 1))
    idx = np.searchsorted(src_t, dst_t, side="left")
    idx = np.clip(idx, 1, len(src_t) - 1) if len(src_t) > 1 else np.zeros_like(idx)
    if len(src_t) > 1:
        left, right = src_t[idx - 1], src_t[idx]
        idx = np.where(np.abs(dst_t - left) <= np.abs(dst_t - right), idx - 1, idx)
    else:
        idx = np.zeros_like(idx)
    return src_v[idx]


def in_span(src_t, dst_t):
    """dst 时刻是否落在 src 的时间跨度内。超出部分最近邻会「保持末值外推」，
    必须掩掉 —— 例如 /cmd_contactFlag 只发 5 s 而 /simContactFlag 发 23 s。"""
    if len(src_t) == 0 or len(dst_t) == 0:
        return np.zeros(0, dtype=bool)
    return (dst_t >= src_t[0]) & (dst_t <= src_t[-1])


def switch_times(t, v):
    """序列 v 的切换时刻 [s]。"""
    if len(t) < 2:
        return np.zeros(0)
    return t[np.flatnonzero(np.diff(v) != 0) + 1]


def switch_err_ms(t_plan, t_act):
    """计划↔实测切换时刻最近邻配对的平均绝对误差 [ms]。"""
    if len(t_plan) == 0 or len(t_act) == 0:
        return float("nan")
    d = np.abs(t_plan[:, None] - t_act[None, :]).min(axis=1)
    return float(1000.0 * d.mean())


# ---------------------------------------------------------------- 稳态窗
def steady_window(t, vx, cmd, band, t_cmd, use_full):
    """返回 (i0, i1, note)：稳态窗的闭开下标区间。"""
    if len(t) < 10:
        return 0, len(t), "样本过少，用全程"
    t_end = t[-1] - TAIL_TRIM_S
    if use_full or t_end <= t[0]:
        return 0, len(t), "用全程（--full 或录制过短）"
    lo = np.searchsorted(t, t_cmd, side="left")
    inside = np.abs(vx - cmd) <= band
    need = max(1, int(DWELL_S * len(t) / max(t[-1] - t[0], 1e-9)))
    run = 0
    for i in range(lo, len(t)):
        if t[i] > t_end:
            break
        run = run + 1 if inside[i] else 0
        if run >= need:
            return i - run + 1, len(t), f"band={band:.3f} m/s, dwell={DWELL_S}s"
    return lo, len(t), f"未稳定进入目标带(band={band:.3f})，自指令起算"


# ---------------------------------------------------------------- 指标
def contact_metrics(cmd, sim, window, t_ref):
    """逐足: duty(计划/实测)、agree%、切换时刻误差[ms]、切换次数。

    规划侧 /cmd_contactFlag (N,2)=[L,R]；实测侧 /simContactFlag (M,4)=逐接触点。
    足级实测 = toe|heel。两者都对齐到 t_ref（实测时间轴）再比较。
    """
    cmd_t, cmd_v = cmd
    sim_t, sim_v = sim
    if len(cmd_t) == 0 or len(sim_t) == 0:
        return [], None, None, (float("nan"), float("nan"))
    if sim_v.shape[1] < 4 or cmd_v.shape[1] < 2:
        return [], None, None, (float("nan"), float("nan"))

    # 实测足级：L = LTOE|LHEEL, R = RTOE|RHEEL
    sim_foot = np.zeros((len(sim_t), 2))
    sim_foot[:, 0] = np.maximum(sim_v[:, 0], sim_v[:, 2])
    sim_foot[:, 1] = np.maximum(sim_v[:, 1], sim_v[:, 3])
    cmd_al = nearest_align(cmd_t, cmd_v, sim_t)

    # 只在两侧时间跨度的交集内比较：规划侧可能提前停止发布
    # （update() 安全检查提前 return 时跳过 cmd_contactFlag），
    # 此时最近邻会对剩余时段「保持末值外推」，会把 duty/switch 算错。
    # 对齐方向是 cmd→sim，掩码作用在 sim_t 上，只需 sim 时刻落在 cmd 跨度内
    m = ((sim_t >= t_ref[0]) & (sim_t <= t_ref[1]) & in_span(cmd_t, sim_t))
    if not m.any():
        m = in_span(cmd_t, sim_t)
    if not m.any():
        m = np.ones_like(sim_t, dtype=bool)
    ov_lo, ov_hi = float(sim_t[m][0]), float(sim_t[m][-1])
    rows = []
    for i, name in enumerate(FOOT_NAMES):
        cp, cs = cmd_al[m, i], sim_foot[m, i]
        tp, ts = sim_t[m], sim_t[m]
        sp, ss = switch_times(tp, cp), switch_times(ts, cs)
        rows.append(dict(
            foot=name,
            duty_planned=float(cp.mean()) if len(cp) else float("nan"),
            duty_actual=float(cs.mean()) if len(cs) else float("nan"),
            agree_pct=float(100.0 * (cp == cs).mean()) if len(cp) else float("nan"),
            switch_err_ms=switch_err_ms(sp, ss),
            n_switch_planned=int(len(sp)), n_switch_actual=int(len(ss))))
    return rows, (sim_t[m], cmd_al[m]), (sim_t[m], sim_foot[m]), (ov_lo, ov_hi)


def per_point_metrics(sim, window, t_ref):
    """逐接触点 duty，供 fig6_2 明细用。"""
    sim_t, sim_v = sim
    if len(sim_t) == 0 or sim_v.shape[1] < 4:
        return []
    m = (sim_t >= t_ref[0]) & (sim_t <= t_ref[1])
    if not m.any():
        m = np.ones_like(sim_t, dtype=bool)
    return [dict(point=POINT_NAMES[i], duty_actual=float(sim_v[m, i].mean()))
            for i in range(4)]


def track_metrics(target, real, window, ncol):
    """通用跟踪指标：把 real 最近邻对齐到 target 时间轴后求误差。"""
    tt, tv = target
    rt, rv = real
    if len(tt) == 0 or len(rt) == 0:
        return None
    ncol = min(ncol, tv.shape[1], rv.shape[1])
    if ncol == 0:
        return None
    m = (tt >= window[0]) & (tt <= window[1]) & in_span(rt, tt)
    if not m.any():
        m = in_span(rt, tt)
    if not m.any():
        m = np.ones_like(tt, dtype=bool)
    tt, tv = tt[m], tv[m, :ncol]
    ra = nearest_align(rt, rv[:, :ncol], tt)
    err = tv - ra
    return dict(rms_per=np.sqrt((err ** 2).mean(axis=0)),
                max_per=np.abs(err).max(axis=0),
                rms_total=float(np.sqrt((err ** 2).mean())),
                max_abs=float(np.abs(err).max()), n=int(len(err)))


def scalar_metrics(x, window, budget):
    if len(x) == 0 or len(x[0]) == 0:
        return None, np.zeros(0)
    m = (x[0] >= window[0]) & (x[0] <= window[1])
    if not m.any():
        m = np.ones_like(x[0], dtype=bool)
    s = x[1][m, 0]
    if len(s) == 0:            # 话题在窗口内无样本（旧 bag 未埋点）
        return None, np.zeros(0)
    return dict(mean=float(s.mean()), p95=float(np.percentile(s, 95)),
                p99=float(np.percentile(s, 99)), max=float(s.max()),
                budget_ms=budget,
                over_budget_pct=100.0 * float((s > budget).mean())), s


def rmse(y, ref):
    return float(np.sqrt(((y - ref) ** 2).mean())) if len(y) else float("nan")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--cmd", type=float, default=0.3,
                    help="指令速度 vx [m/s]（/cmd_vel 直接就是目标速度，无缩放）")
    ap.add_argument("--out", default=None)
    ap.add_argument("--band", type=float, default=None,
                    help="稳态窗目标带宽 [m/s]，默认 max(0.05, 0.3*|cmd|)")
    ap.add_argument("--full", action="store_true", help="跳过稳态窗，用全程")
    a = ap.parse_args()
    out_dir = a.out or os.path.join(os.path.dirname(a.bag.rstrip("/")), "analysis")
    os.makedirs(out_dir, exist_ok=True)

    try:
        data = load(a.bag)
    except Exception as e:                    # bag 损坏（如 auto_v2 的 read failed）
        summary = f"STATUS: INVALID(bag 无法读取: {type(e).__name__}: {e})\nbag: {a.bag}\n"
        with open(os.path.join(out_dir, "summary.txt"), "w") as f:
            f.write(summary)
        print(summary)
        sys.exit(1)
    problems = []

    # ---- 缺话题判定（缺一即 INVALID，与交付需求 A1 一致）----
    required = ["/ground_truth/state", "/cmd_contactFlag", "/simContactFlag",
                "/targetTorque", "/realTorque", "/targetPos", "/jointsPosVel"]
    missing = [k for k in required if len(data[k][0]) == 0]
    if missing:
        problems.append("缺话题 " + "/".join(missing))

    # ---- 时间轴与稳态窗 ----
    # /ground_truth/state 展平为 [vx,vy,vz, px,py,pz] —— 见 extract()
    gt_t, gt_v = data["/ground_truth/state"]
    if len(gt_t):
        vx = gt_v[:, 0]
        height = gt_v[:, 5] if gt_v.shape[1] > 5 else np.zeros(len(gt_t))
    else:
        vx = height = np.zeros(0)

    cv_t, cv_v = data["/cmd_vel"]
    t_cmd = float(cv_t[np.flatnonzero(np.abs(cv_v[:, 0]) > 1e-6)[0]]) \
        if len(cv_t) and np.any(np.abs(cv_v[:, 0]) > 1e-6) else \
        (float(gt_t[0]) if len(gt_t) else 0.0)

    band = a.band if a.band is not None else max(0.05, 0.3 * abs(a.cmd))
    if len(gt_t):
        i0, i1, win_note = steady_window(gt_t, vx, a.cmd, band, t_cmd, a.full)
        w_lo, w_hi = float(gt_t[i0]), float(gt_t[min(i1, len(gt_t)) - 1])
    else:
        i0, i1, win_note, w_lo, w_hi = 0, 0, "无 /ground_truth/state", 0.0, 0.0

    # 暂停区间界定：/realTorque 只在仿真非暂停时发布（humanoid_sim.py 主循环），
    # /pauseFlag 只在 pause→unpause 沿发一条，不能当时间轴。故用 /realTorque 的
    # 时间跨度作为「已解暂停」区间，并把它与稳态窗求交。
    rt_t, _ = data["/realTorque"]
    if len(rt_t):
        un_lo, un_hi = float(rt_t[0]), float(rt_t[-1])
    elif len(gt_t):
        un_lo, un_hi = float(gt_t[0]), float(gt_t[-1])
    else:
        un_lo = un_hi = 0.0
    if len(gt_t):
        span = float(gt_t[-1] - gt_t[0])
        paused_fraction = float(1.0 - (un_hi - un_lo) / span) if span > 0 else float("nan")
    else:
        paused_fraction = float("nan")
    w_lo, w_hi = max(w_lo, un_lo), min(w_hi, un_hi)
    if w_hi <= w_lo:                      # 交集为空 → 退回未暂停区间
        w_lo, w_hi = un_lo, un_hi
        win_note += "；与未暂停区间无交集，用未暂停区间"
    window = (w_lo, w_hi)
    if np.isfinite(paused_fraction) and paused_fraction > 0.5:
        problems.append(f"录制 {paused_fraction*100:.0f}% 处于暂停态（数据大量缺失）")

    # ---- 1. 基座/高度跟踪 ----
    if len(gt_t):
        m = np.zeros(len(gt_t), dtype=bool); m[i0:i1] = True
        if not m.any():
            m = np.ones_like(m)
        vx_w, h_w = vx[m], height[m]
    else:
        vx_w = h_w = np.zeros(0)
    t0 = float(gt_t[0]) if len(gt_t) else 0.0
    metrics = {"cmd_vx": a.cmd,
               "window_start_s": w_lo - t0, "window_end_s": w_hi - t0,
               "unpaused_start_s": un_lo - t0, "unpaused_end_s": un_hi - t0,
               "paused_fraction": paused_fraction,
               "vx_mean": float(vx_w.mean()) if len(vx_w) else float("nan"),
               "vx_std": float(vx_w.std()) if len(vx_w) else float("nan"),
               "vx_rmse": rmse(vx_w, a.cmd),
               "height_mean": float(h_w.mean()) if len(h_w) else float("nan"),
               "height_std": float(h_w.std()) if len(h_w) else float("nan"),
               # 高度无外部参考（comHeight 0.952 是质心高度，与基座高度不同口径），
               # RMSE 取对窗内均值的偏差，等价于波动幅度
               "height_rmse": rmse(h_w, h_w.mean()) if len(h_w) else float("nan")}
    if len(vx_w) == 0:
        problems.append("基座状态为空")

    # ---- 2. 接触时序 ----
    contact_rows, cmd_plot, sim_plot, ov_span = contact_metrics(
        data["/cmd_contactFlag"], data["/simContactFlag"], window, (w_lo, w_hi))
    points = per_point_metrics(data["/simContactFlag"], window, (w_lo, w_hi))
    if not contact_rows:
        problems.append("接触标志为空/维度不符")

    # ---- 3. 力矩/关节跟踪 ----
    tau = track_metrics(data["/targetTorque"], data["/realTorque"], window, 12)
    q = track_metrics(data["/targetPos"], data["/jointsPosVel"], window, 12)
    if tau is None:
        problems.append("力矩话题为空（/realTorque 只在仿真非暂停时发布）")
    if q is None:
        problems.append("关节话题为空")

    # ---- 4. 求解耗时 ----
    mpc_stats, mpc_ms = scalar_metrics(data["/mpc_solve_time_ms"], window, MPC_BUDGET_MS)
    wbc_stats, wbc_ms = scalar_metrics(data["/wbc_solve_time_ms"], window, WBC_BUDGET_MS)
    if mpc_stats is None:
        problems.append("无 /mpc_solve_time_ms（旧 bag 未埋点）")

    # ---- 倒地判定（交付需求 B2：若倒地则标 INVALID）----
    # 判据与控制器安全检查同源（|roll|/|pitch| > 45° 锁输出），高度 < 0.75×基准
    # 作补充；基准高度取解暂停后前 2 s 中位数（起始为站立姿态）。
    fall_frac = h_nom = float("nan")
    if len(gt_t) and gt_v.shape[1] >= 8:
        g_un = (gt_t >= un_lo) & (gt_t <= un_hi)
        if g_un.any():
            ref = (gt_t >= un_lo) & (gt_t <= un_lo + 2.0)
            h_nom = float(np.median(height[ref])) if ref.any() \
                else float(np.median(height[g_un]))
            ang = np.maximum(np.abs(gt_v[:, 6]), np.abs(gt_v[:, 7]))
            fallen = (height < 0.75 * h_nom) | (ang > np.pi / 4)
            fall_frac = float(fallen[g_un].mean())
        if np.isfinite(fall_frac) and fall_frac > 0.05:
            problems.append(f"倒地/姿态超限({fall_frac*100:.0f}% 样本, 基准高 {h_nom:.2f} m)")
    if np.isfinite(metrics["height_mean"]) and metrics["height_mean"] < 0.70:
        problems.append(f"高度均值 {metrics['height_mean']:.2f} m(<0.70, 疑倒地)")
    if np.isfinite(fall_frac):
        metrics["fall_frac"] = fall_frac
    if np.isfinite(h_nom):
        metrics["height_nominal"] = h_nom

    # ---- RTF（仿真实时率）----
    # /simContactFlag 每 1/500 s **仿真时间**发一条，bag 时间戳是墙钟：
    # RTF = 仿真跨度 / 墙钟跨度。不能用 observation.time 测（那是墙钟，恒 1.0）。
    # RTF<0.5 = 仿真爬行、机器人"冻住"，实验实际未执行（auto_ab_04 假阳性教训）；
    # 0.5~1 之间的时间放大 run 保留 VALID，但 RTF 行随 summary 输出，引用须注明。
    rtf = float("nan")
    sc_t, _ = data["/simContactFlag"]
    if len(sc_t) > 1 and sc_t[-1] > sc_t[0]:
        rtf = ((len(sc_t) - 1) / 500.0) / float(sc_t[-1] - sc_t[0])
        metrics["rtf"] = rtf
        if rtf < 0.5:
            problems.append(f"仿真冻结 RTF={rtf:.3f}（时间基准不一致，实验未实际执行）")

    # ---- nan 判定 ----
    nan_items = [k for k, v in metrics.items() if isinstance(v, float) and np.isnan(v)]
    if tau and np.isnan(tau["rms_total"]):
        nan_items.append("torque_tracking")
    if q and np.isnan(q["rms_total"]):
        nan_items.append("joint_tracking")
    if nan_items:
        problems.append("nan: " + "/".join(nan_items))

    status = "VALID" if not problems else "INVALID(" + "; ".join(problems) + ")"

    # ---- metrics.csv ----
    with open(os.path.join(out_dir, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["category", "item", "value"])
        w.writerow(["status", "status", status])
        for k, v in metrics.items():
            w.writerow(["base_tracking", k, v])
        for row in contact_rows:
            for k, v in row.items():
                w.writerow(["contact_timing", f"{row['foot']}.{k}", v])
        for row in points:
            w.writerow(["contact_point", f"{row['point']}.duty_actual", row["duty_actual"]])
        if tau:
            for i in range(len(tau["rms_per"])):
                w.writerow(["torque_tracking", f"joint{i}.rms", float(tau["rms_per"][i])])
                w.writerow(["torque_tracking", f"joint{i}.max", float(tau["max_per"][i])])
            w.writerow(["torque_tracking", "rms_total", tau["rms_total"]])
            w.writerow(["torque_tracking", "max_abs", tau["max_abs"]])
        if q:
            for i in range(len(q["rms_per"])):
                w.writerow(["joint_tracking", f"joint{i}.rms", float(q["rms_per"][i])])
                w.writerow(["joint_tracking", f"joint{i}.max", float(q["max_per"][i])])
            w.writerow(["joint_tracking", "rms_total", q["rms_total"]])
            w.writerow(["joint_tracking", "max_abs", q["max_abs"]])
        for name, st in (("mpc", mpc_stats), ("wbc", wbc_stats)):
            if st:
                for k, v in st.items():
                    w.writerow(["solve_time", f"{name}.{k}", v])

    # ---- summary.txt ----
    def f2(x, n=3):
        return f"{x:.{n}f}" if np.isfinite(x) else "nan"

    lines = [f"STATUS: {status}",
             f"bag: {a.bag}", f"cmd_vx: {a.cmd} m/s",
             f"RTF: {f2(metrics.get('rtf', float('nan')),3)}"
             f"（1.0=实时；<0.5 判仿真冻结；实机 bag 无 /simContactFlag 为 nan）",
             f"steady window: [{f2(w_lo-t0,2)}, {f2(w_hi-t0,2)}] s  ({win_note})",
             f"unpaused: [{f2(un_lo-t0,2)}, {f2(un_hi-t0,2)}] s, "
             f"paused_fraction = {f2(paused_fraction,3)}", "",
             f"[1] base tracking: vx = {f2(metrics['vx_mean'])} ± {f2(metrics['vx_std'])} m/s "
             f"(RMSE {f2(metrics['vx_rmse'])}), height = {f2(metrics['height_mean'])} ± "
             f"{f2(metrics['height_std'])} m", "",
             "[2] contact timing:",
             f"    compare span: [{f2(ov_span[0]-t0,2)}, {f2(ov_span[1]-t0,2)}] s "
             f"(规划/实测时间跨度交集；超出不外推)"]
    for row in contact_rows:
        lines.append(f"  {row['foot']}: duty {f2(row['duty_planned'],2)}->{f2(row['duty_actual'],2)}, "
                     f"agree {f2(row['agree_pct'],1)}%, switch_err {f2(row['switch_err_ms'],1)} ms, "
                     f"switches {row['n_switch_planned']}/{row['n_switch_actual']}")
    if points:
        lines.append("    per-point duty actual: " +
                     "  ".join(f"{p['point']}={f2(p['duty_actual'],2)}" for p in points))
    lines += ["",
              f"[3] torque tracking: rms {f2(tau['rms_total'],2) if tau else 'nan'} Nm, "
              f"max {f2(tau['max_abs'],1) if tau else 'nan'} Nm",
              f"    joint pos: rms {f2(q['rms_total'],4) if q else 'nan'} rad, "
              f"max {f2(q['max_abs'],3) if q else 'nan'} rad", "",
              "[4] solve time:"]
    for name, st, budget in (("MPC", mpc_stats, MPC_BUDGET_MS), ("WBC", wbc_stats, WBC_BUDGET_MS)):
        if st:
            lines.append(f"  {name}: mean {st['mean']:.2f} ms, p95 {st['p95']:.2f}, "
                         f"p99 {st['p99']:.2f}, max {st['max']:.2f} | budget {budget:.0f} ms, "
                         f"over {st['over_budget_pct']:.2f}%")
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

    # fig1 基座/高度
    fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    if len(gt_t):
        rt = gt_t - gt_t[0]
        ax[0].plot(rt, vx, lw=1.2, label="vx actual")
        ax[0].axhline(a.cmd, color="r", ls="--", lw=1, label="cmd")
        ax[0].axvspan(w_lo - gt_t[0], w_hi - gt_t[0], color="g", alpha=.12, label="steady window")
        ax[0].set_ylabel("vx [m/s]"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
        ax[1].plot(rt, height, lw=1.2)
        ax[1].axvspan(w_lo - gt_t[0], w_hi - gt_t[0], color="g", alpha=.12)
        ax[1].set_ylabel("base height [m]"); ax[1].set_xlabel("time [s]"); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig1_base_tracking.png"), dpi=300)
    plt.close(fig)

    # fig2 接触时序（时间轴）
    if cmd_plot is not None:
        fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
        t0 = cmd_plot[0][0]
        for i, name in enumerate(FOOT_NAMES):
            ax[0].plot(cmd_plot[0] - t0, cmd_plot[1][:, i] + 0.12 * i, lw=1, label=name)
            ax[1].plot(sim_plot[0] - t0, sim_plot[1][:, i] + 0.12 * i, lw=1, label=name)
            bad = cmd_plot[1][:, i] != sim_plot[1][:, i]
            if bad.any():
                ax[1].plot(cmd_plot[0][bad] - t0, sim_plot[1][bad, i] + 0.12 * i,
                           "r.", ms=1.5, label=f"{name} mismatch")
        ax[0].set_title("planned contact (/cmd_contactFlag)")
        ax[1].set_title("actual contact (/simContactFlag) — red = mismatch")
        for aa in ax:
            aa.set_yticks([]); aa.legend(loc="upper right", fontsize=7); aa.grid(alpha=.3)
        ax[1].set_xlabel("time [s]")
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig2_contact_timing.png"), dpi=300)
        plt.close(fig)

    # fig3 力矩/关节
    if tau or q:
        fig, ax = plt.subplots(1, 2, figsize=(8, 3.5))
        if tau:
            ax[0].bar(np.arange(len(tau["rms_per"])), tau["rms_per"])
        ax[0].set_xlabel("joint"); ax[0].set_ylabel("torque RMS err [Nm]"); ax[0].grid(alpha=.3)
        if q:
            ax[1].bar(np.arange(len(q["rms_per"])), q["rms_per"])
        ax[1].set_xlabel("joint"); ax[1].set_ylabel("joint pos RMS err [rad]"); ax[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig3_tracking_errors.png"), dpi=300)
        plt.close(fig)

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
    plt.close(fig)
    print(f"\n[analyze] outputs -> {out_dir}")


if __name__ == "__main__":
    main()
