#!/usr/bin/env python3
"""实机 rosbag → C6 五类指标 + 硬件表征（A3 真机侧，2026-10-15）。

用法: analyze_real.py <run_dir> [--cmd 0.3] [--out DIR] [--full]
输入: run 目录；run 名形如 real_base_v03_21 / real_push_lat_L3_stand_21。
      mcap 位置兼容两种布局：<run>/bag/*.mcap（sim_data）或 <run>/*.mcap（real）。

与仿真侧 analyze_walk.py/analyze_push.py **口径对齐**（同一套稳态窗、接触
折叠、跟踪误差、t_rec 判据），仅数据源换成实机话题：

  状态源   /humanoid_mpc_observation.state（24 维，布局经 sim 真值回验）：
             [0:3]=基座线速度  [3:6]=基座角速度  [6:9]=基座位置
             [9:12]=[yaw,pitch,roll]  [12:24]=12 关节角
           —— sim 侧用 /ground_truth/state 真值，实机侧是**状态估计输出**，
           论文表述须注明估计口径（无外部真值/动捕）。
  z_c      MuJoCo 名义模型正解（zc_lib.com_height），与 6.4.1 同口径。
  接触     /cmd_contactFlag（2=[L,R] 计划）vs /realContactFlag（4=逐点实测，
           足级 = toe|heel 折叠）。
  力矩     /targetTorque（WBC 指令）vs /realTorque（执行侧反馈输出力矩）；
           /targetPos vs /jointsPosVel（前 12 列关节角）。
  实时性   /mpc_solve_time_ms（预算 10ms）、/wbc_solve_time_ms（预算 2ms）。
  扰动     /real_push_fired 给注入时刻 t0；恢复判据沿用 analyze_push 的
           「稳定下来」口径（姿态<eps 且 |平滑位移速率|<eps_rate 持续 thold）。
  硬件     /jointTemperature、/busVoltage、/battery_state、/jointCurrent。

产物: <out>/metrics.csv + summary.txt（+ push_summary.txt 段）
"""
import argparse
import glob
import math
import os
import re
import sys

if "AMENT_PREFIX_PATH" not in os.environ:
    _cmd = ('source /opt/ros/jazzy/setup.bash 2>/dev/null; '
            'source %s/install/setup.bash 2>/dev/null; exec python3 "$0" "$@"'
            % os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    os.execvp("bash", ["bash", "-c", _cmd, os.path.abspath(__file__)] + sys.argv[1:])

import numpy as np

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ModuleNotFoundError:
    for p in glob.glob("/opt/ros/*/lib/python3*/site-packages"):
        sys.path.insert(0, p)
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FOOT_NAMES = ["L", "R"]
POINT_NAMES = ["LTOE", "RTOE", "LHEEL", "RHEEL"]
MPC_BUDGET_MS = 10.0
WBC_BUDGET_MS = 2.0
TAIL_TRIM_S = 0.5
DWELL_S = 0.3
GAIT_SMOOTH_S = 1.0
GAIT_TOL = 0.05

WANT = {"/humanoid_mpc_observation", "/cmd_contactFlag", "/realContactFlag",
        "/targetTorque", "/realTorque", "/targetPos", "/jointsPosVel",
        "/controlInput", "/jointCurrent", "/mpc_solve_time_ms",
        "/wbc_solve_time_ms", "/cmd_vel", "/real_push_fired",
        "/jointTemperature", "/busVoltage", "/battery_state",
        "/foot_vel_estimate", "/hwswitch", "/diagnostics"}


def _read_torque_limits():
    """力矩界（task_.info torqueLimitsTask），12 维。饱和计数用。"""
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "src/humanoid_control/humanoid_interface/config/mpc/task_.info")
    try:
        txt = open(p, encoding="utf-8").read()
        blk = re.search(r"torqueLimitsTask\s*\{(.*?)\}", txt, re.S).group(1)
        d = {int(i): float(v) for i, v in
             re.findall(r"\(\s*(\d)\s*,\s*0\)\s+([\d.]+)", blk)}
        return np.array([d[i % 6] for i in range(12)], float)
    except Exception:
        return np.array([200, 200, 320, 320, 200, 200] * 2, float)


TORQUE_LIMITS = _read_torque_limits()


# ---------------------------------------------------------------- IO
def read_bag(bag_dir, topics):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in topics}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in types:
            continue
        yield topic, ts * 1e-9, deserialize_message(data, types[topic])
    reader.close()


def flat(x):
    """MpcState/MpcInput 消息或数值列表 → 一维 float 数组。"""
    if hasattr(x, "value"):
        return np.array(x.value, dtype=float).ravel()
    if isinstance(x, (list, tuple)):
        out = []
        for e in x:
            out.extend(flat(e).tolist())
        return np.array(out, dtype=float)
    return np.array([x], dtype=float)


def extract(msg):
    d = getattr(msg, "data", None)
    if d is not None:
        return [float(x) for x in d] if hasattr(d, "__iter__") else [float(d)]
    if hasattr(msg, "state"):                      # ocs2 MpcObservation
        st = flat(msg.state)
        # 拼上 base 线/角速度占位使向量定长（布局见文件头）
        return st.tolist()
    if hasattr(msg, "voltage") and hasattr(msg, "current"):   # BatteryState
        return [float(msg.voltage), float(msg.current),
                float(msg.temperature), float(msg.charge), float(msg.percentage)]
    return None


def load(bag_dir):
    t, v = {}, {}
    rates = {}
    first_last = {}
    for topic, ts, msg in read_bag(bag_dir, WANT):
        row = extract(msg)
        if row is None:
            continue
        t.setdefault(topic, []).append(ts)
        v.setdefault(topic, []).append(row)
    out = {}
    for k in set(list(t) + list(WANT)):
        tt = np.asarray(t.get(k, []), dtype=float)
        vv = np.asarray(v.get(k, []), dtype=float)
        if vv.size == 0:
            vv = np.zeros((0, 1))
        elif vv.ndim == 1:
            vv = vv.reshape(-1, 1)
        out[k] = (tt, vv)
        if len(tt) > 1:
            rates[k] = (len(tt) - 1) / (tt[-1] - tt[0])
            first_last[k] = (tt[0], tt[-1])
    return out, rates


# ---------------------------------------------------------------- 时间域工具
def nearest_align(src_t, src_v, dst_t):
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
    if len(src_t) == 0 or len(dst_t) == 0:
        return np.zeros(0, dtype=bool)
    return (dst_t >= src_t[0]) & (dst_t <= src_t[-1])


def switch_times(t, v):
    if len(t) < 2:
        return np.zeros(0)
    return t[np.flatnonzero(np.diff(v) != 0) + 1]


def switch_err_ms(t_plan, t_act):
    if len(t_plan) == 0 or len(t_act) == 0:
        return float("nan")
    d = np.abs(t_plan[:, None] - t_act[None, :]).min(axis=1)
    return float(1000.0 * d.mean())


# ---------------------------------------------------------------- 稳态窗（口径同 analyze_walk）
def steady_window(t, vx, cmd, band, t_cmd, use_full, t_gait=None):
    if len(t) < 10:
        return 0, len(t), "样本过少，用全程"
    t_end = t[-1] - TAIL_TRIM_S
    if use_full or t_end <= t[0]:
        return 0, len(t), "用全程（--full 或录制过短）"
    lo = int(np.searchsorted(t, t_cmd, side="left"))
    note_gait = ""
    if t_gait is not None:
        lo_g = int(np.searchsorted(t, t_gait, side="left"))
        lo_g = min(lo_g, len(t) - 1)
        if lo_g > lo:
            lo = lo_g
            note_gait = f"，gait收敛下界@{t_gait - t[0]:.1f}s"
    inside = np.abs(vx - cmd) <= band
    need = max(1, int(DWELL_S * len(t) / max(t[-1] - t[0], 1e-9)))
    run = 0
    for i in range(lo, len(t)):
        if t[i] > t_end:
            break
        run = run + 1 if inside[i] else 0
        if run >= need:
            return i - run + 1, len(t), f"band={band:.3f} m/s, dwell={DWELL_S}s{note_gait}"
    return lo, len(t), f"未稳定进入目标带(band={band:.3f})，自{'步态起点' if t_gait is not None else '指令'}起算{note_gait}"


def gait_convergence_time(cmd_t, cmd_v):
    """计划 duty 滑动均值进入终值 ±GAIT_TOL 并保持的时刻（cumsum 实现）。"""
    if len(cmd_t) < 10 or cmd_v.shape[1] < 2:
        return None
    duty = cmd_v[:, :2].mean(axis=1)
    dt = float(np.median(np.diff(cmd_t)))
    k = max(1, int(GAIT_SMOOTH_S / max(dt, 1e-6)))
    if len(duty) <= k:
        return None
    cs = np.concatenate(([0.0], np.cumsum(duty)))
    sm = (cs[k:] - cs[:-k]) / k
    tt = cmd_t[k - 1:]
    final = float(np.median(sm[-max(1, len(sm) // 10):]))
    ok = np.abs(sm - final) <= GAIT_TOL
    for i in range(len(ok) - 1, -1, -1):
        if not ok[i]:
            return float(tt[min(i + 1, len(tt) - 1)])
    return float(tt[0])


# ---------------------------------------------------------------- 指标
def contact_metrics(cmd, act, t_ref):
    """逐足 duty/agree%/切换误差/切换数。计划 (N,2) vs 实测 (M,4)→足级折叠。"""
    cmd_t, cmd_v = cmd
    act_t, act_v = act
    if len(cmd_t) == 0 or len(act_t) == 0:
        return [], None, None, (float("nan"), float("nan"))
    if act_v.shape[1] < 4 or cmd_v.shape[1] < 2:
        return [], None, None, (float("nan"), float("nan"))
    act_foot = np.zeros((len(act_t), 2))
    act_foot[:, 0] = np.maximum(act_v[:, 0], act_v[:, 2])
    act_foot[:, 1] = np.maximum(act_v[:, 1], act_v[:, 3])
    cmd_al = nearest_align(cmd_t, cmd_v, act_t)
    m = ((act_t >= t_ref[0]) & (act_t <= t_ref[1]) & in_span(cmd_t, act_t))
    if not m.any():
        m = in_span(cmd_t, act_t)
    if not m.any():
        m = np.ones_like(act_t, dtype=bool)
    ov_lo, ov_hi = float(act_t[m][0]), float(act_t[m][-1])
    rows = []
    for i, name in enumerate(FOOT_NAMES):
        cp, cs = cmd_al[m, i], act_foot[m, i]
        tp, ts = act_t[m], act_t[m]
        sp, ss = switch_times(tp, cp), switch_times(ts, cs)
        rows.append(dict(
            foot=name,
            duty_planned=float(cp.mean()) if len(cp) else float("nan"),
            duty_actual=float(cs.mean()) if len(cs) else float("nan"),
            agree_pct=float(100.0 * (cp == cs).mean()) if len(cp) else float("nan"),
            switch_err_ms=switch_err_ms(sp, ss),
            n_switch_planned=int(len(sp)), n_switch_actual=int(len(ss))))
    return rows, (act_t[m], cmd_al[m]), (act_t[m], act_foot[m]), (ov_lo, ov_hi)


def track_metrics(target, real, window, ncol):
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
    if len(s) == 0:
        return None, np.zeros(0)
    return dict(mean=float(s.mean()), p95=float(np.percentile(s, 95)),
                p99=float(np.percentile(s, 99)), max=float(s.max()),
                budget_ms=budget,
                over_budget_pct=100.0 * float((s > budget).mean())), s


def rmse(y, ref):
    return float(np.sqrt(((y - ref) ** 2).mean())) if len(y) else float("nan")


def f2(x, nd=3):
    if x is None:
        return "NA"
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return "NA"
    return f"{x:.{nd}f}"


def resolve_bag_dir(run):
    """兼容两种落盘布局：sim_data/<run>/bag/ 与 real/<run>/（mcap 直接在 run 下）。"""
    for cand in (os.path.join(run, "bag"), run):
        if glob.glob(os.path.join(cand, "*.mcap")):
            return cand
    return os.path.join(run, "bag")


def guard_bag_size(bag_dir, limit_gb=2.0):
    total = sum(os.path.getsize(f) for f in glob.glob(os.path.join(bag_dir, "*.mcap")))
    if total > limit_gb * 1e9:
        print(f"ERROR: {bag_dir} 的 bag 达 {total / 1e9:.1f} GB（阈值 {limit_gb} GB）")
        sys.exit(3)


def cmd_from_name(name):
    """real_base_v03_21→0.3；real_push_lat_L3_stand_21→stand=0.0/walk=0.1。"""
    m = re.search(r"_v(\d\d)_", name)
    if m:
        return {0: 0.0, 1: 0.1, 3: 0.3}.get(int(m.group(1)), 0.1)
    if "_stand" in name:
        return 0.0
    if "_walk" in name:
        return 0.1
    return 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--cmd", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--band", type=float, default=0.05)
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--eps-rate", type=float, default=0.02)
    ap.add_argument("--settle-window", type=float, default=1.5)
    ap.add_argument("--thold", type=float, default=1.0)
    a = ap.parse_args()
    run = a.run.rstrip("/")
    name = os.path.basename(run)
    cmd = a.cmd if a.cmd is not None else cmd_from_name(name)
    bag_dir = resolve_bag_dir(run)
    guard_bag_size(bag_dir)
    out_dir = a.out or os.path.join(run, "analysis")
    os.makedirs(out_dir, exist_ok=True)

    data, rates = load(bag_dir)
    problems = []

    # ---- 数据完备性 ----
    need = ["/humanoid_mpc_observation", "/cmd_contactFlag", "/realContactFlag",
            "/targetTorque", "/realTorque", "/jointsPosVel", "/mpc_solve_time_ms"]
    missing = [k for k in need if len(data[k][0]) == 0]
    if missing:
        problems.append("缺话题 " + "/".join(missing))

    obs_t, obs_v = data["/humanoid_mpc_observation"]
    if len(obs_t) > 1:
        keep = np.concatenate(([True], np.diff(obs_t) > 0))
        obs_t, obs_v = obs_t[keep], obs_v[keep]
    if obs_v.size and obs_v.shape[1] < 24:
        problems.append(f"observation 状态维数 {obs_v.shape[1]} < 24")
    nan_hits = {k: int(np.isnan(v[1]).sum()) for k, v in data.items()
                if v[1].size and np.isnan(v[1]).any()}
    if nan_hits:
        problems.append("含 NaN: " + ",".join(f"{k}x{n}" for k, n in nan_hits.items()))

    # ---- 状态量（估计口径）----
    if obs_v.size and obs_v.shape[1] >= 24:
        vx_all = obs_v[:, 0]
        base_z = obs_v[:, 8]
        yaw_o, pitch_o, roll_o = obs_v[:, 9], obs_v[:, 10], obs_v[:, 11]
        joints = obs_v[:, 12:24]
    else:
        vx_all = np.zeros(0); base_z = np.zeros(0)
        yaw_o = pitch_o = roll_o = np.zeros(0)
        joints = np.zeros((0, 12))

    # 指令时刻 = /cmd_vel 首次出现目标速度，或首条 hwswitch 使能
    cv_t, cv_v = data["/cmd_vel"]
    t_cmd = float(obs_t[0]) if len(obs_t) else 0.0
    if len(cv_t):
        hit = np.flatnonzero(np.abs(cv_v[:, 0] - cmd) < 0.02)
        t_cmd = float(cv_t[hit[0]]) if len(hit) else float(cv_t[0])

    t_gait = gait_convergence_time(*data["/cmd_contactFlag"])
    band = a.band if a.band else (0.09 if abs(cmd) > 0.2 else 0.05)
    if len(obs_t):
        i0, i1, wnote = steady_window(obs_t, vx_all, cmd, band, t_cmd, a.full, t_gait)
        win = (obs_t[i0], obs_t[min(i1, len(obs_t)) - 1])
    else:
        win = (0.0, 1.0); wnote = "无状态数据"
    t0_rec = float(obs_t[0]) if len(obs_t) else 0.0

    # 扰动 run：稳态窗不含注入后的恢复段（跟踪指标与扰动指标分开算）
    fired_t = data["/real_push_fired"][0]
    if len(fired_t):
        t_push_wall = float(fired_t[0])
        if win[0] >= t_push_wall:
            # 注入早于稳态窗起点（gait 收敛判据在注入后才满足）→
            # 跟踪指标取注入前全程，窗口口径记入 wnote
            win = (float(obs_t[0]), t_push_wall)
            wnote += "，注入早于稳态窗→取注入前全程(扰动段另计)"
        elif win[1] > t_push_wall:
            win = (win[0], t_push_wall)
            wnote += "，截断于注入时刻(扰动段另计)"

    # ---- [1] 基座/质心跟踪 ----
    track = {}
    if len(obs_t):
        m = (obs_t >= win[0]) & (obs_t <= win[1])
        if not m.any():
            m = np.ones_like(obs_t, dtype=bool)
        vx, bz = vx_all[m], base_z[m]
        track = dict(vx_mean=float(vx.mean()), vx_std=float(vx.std()),
                     vx_rmse=rmse(vx, cmd),
                     base_z_mean=float(bz.mean()), base_z_std=float(bz.std()))
        try:
            from zc_lib import com_height
            zc = com_height(bz, roll_o[m], pitch_o[m], joints[m])
            track.update(zc_mean=float(np.mean(zc)), zc_std=float(np.std(zc)))
        except Exception as e:
            problems.append(f"z_c 计算失败: {e}")

    # ---- [2] 接触时序 ----
    crows, _, _, cspan = contact_metrics(data["/cmd_contactFlag"],
                                         data["/realContactFlag"], win)
    per_point = []
    act_t, act_v = data["/realContactFlag"]
    if len(act_t) and act_v.shape[1] >= 4:
        mm = (act_t >= win[0]) & (act_t <= win[1])
        if not mm.any():
            mm = np.ones_like(act_t, dtype=bool)
        per_point = [(POINT_NAMES[i], float(act_v[mm, i].mean())) for i in range(4)]

    # ---- [3] 力矩/关节跟踪 ----
    tau = track_metrics(data["/targetTorque"], data["/realTorque"], win, 12)
    jp = track_metrics(data["/targetPos"], data["/jointsPosVel"], win, 12)
    # 逐关节峰值与饱和计数（表6.6 力矩饱和行；限幅取 task_.info torqueLimitsTask）
    tau_peaks, sat_counts = {}, {}
    tt_t, tt_v = data["/targetTorque"]
    rt_t, rt_v = data["/realTorque"]
    if len(tt_t):
        m = (tt_t >= win[0]) & (tt_t <= win[1])
        if not m.any():
            m = np.ones_like(tt_t, bool)
        tau_peaks["cmd_per_joint"] = np.abs(tt_v[m]).max(axis=0).round(1).tolist()
        tau_peaks["cmd_peak"] = float(np.abs(tt_v[m]).max())
        sat = np.abs(tt_v[m]) >= (TORQUE_LIMITS[None, :] - 1e-6)
        sat_counts["n_samples_sat"] = int(sat.sum())
        sat_counts["sat_frac_pct"] = 100.0 * float(sat.mean())
        sat_counts["sat_per_joint"] = sat.sum(axis=0).tolist()
    if len(rt_t):
        m = (rt_t >= win[0]) & (rt_t <= win[1])
        if not m.any():
            m = np.ones_like(rt_t, bool)
        tau_peaks["real_per_joint"] = np.abs(rt_v[m]).max(axis=0).round(1).tolist()
        tau_peaks["real_peak"] = float(np.abs(rt_v[m]).max())
    # 足端滑移（稳态窗内、仅接触点）：/foot_vel_estimate [12:16]=各点速度范数
    fv_t, fv_v = data["/foot_vel_estimate"]
    slip_stats = {}
    if len(fv_t) and fv_v.shape[1] >= 16:
        act_t2, act_v2 = data["/realContactFlag"]
        m = (fv_t >= win[0]) & (fv_t <= win[1])
        if m.any():
            ft, fv = fv_t[m], fv_v[m, 12:16]
            al = nearest_align(act_t2, act_v2, ft) if len(act_t2) else None
            if al is not None and al.shape[1] >= 4:
                # 剔除接触切换 ±50 ms 瞬态：切换瞬间足端在动，flag 已判接触，
                # 计入会把「支撑期滑移」污染成切换速度
                stable = np.ones(len(ft), bool)
                foot = np.maximum(al[:, [0, 2]], al[:, [1, 3]])
                sw = np.flatnonzero(np.diff(foot, axis=0).any(axis=1)) + 1
                dt_s = 0.05
                for i in sw:
                    stable &= (np.abs(ft - ft[i]) > dt_s)
                slip = fv[stable[:, None] & (al[:, :4] > 0)]
            else:
                slip = fv.ravel()
            if slip.size:
                slip_stats["slip_peak"] = float(slip.max())
                slip_stats["slip_p99"] = float(np.percentile(slip, 99))
                slip_stats["slip_p50"] = float(np.percentile(slip, 50))
                slip_stats["slip_mean"] = float(slip.mean())

    # ---- [4] 求解实时性 ----
    mpc, _ = scalar_metrics(data["/mpc_solve_time_ms"], win, MPC_BUDGET_MS)
    wbc, _ = scalar_metrics(data["/wbc_solve_time_ms"], win, WBC_BUDGET_MS)

    # ---- [5] 扰动恢复（有 /real_push_fired 才算）----
    push = {}
    fired = data["/real_push_fired"]
    if len(fired[0]):
        t_push = t_push_wall
        push_dir = "lat" if "_lat_" in name else "fwd"
        px, py = obs_v[:, 6], obs_v[:, 7]
        axis = py if push_dir == "lat" else px
        pre = (obs_t >= t_push - 1.0) & (obs_t < t_push)
        if not pre.any():
            problems.append("注入前 1s 无状态数据")
            pre = obs_t < t_push
        post = (obs_t >= t_push) & (obs_t <= t_push + 5.0)
        r0 = float(roll_o[pre].mean()) if pre.any() else 0.0
        p0 = float(pitch_o[pre].mean()) if pre.any() else 0.0
        a0 = float(axis[pre].mean()) if pre.any() else 0.0
        dphi, dth, dax = roll_o - r0, pitch_o - p0, axis - a0
        push["dir"] = push_dir
        push["t_push"] = t_push - t0_rec
        push["dphi_max"] = float(np.abs(dphi[post]).max()) if post.any() else float("nan")
        push["dth_max"] = float(np.abs(dth[post]).max()) if post.any() else float("nan")
        push["dax_max"] = float(np.abs(dax[post]).max()) if post.any() else float("nan")

        dax_sm = np.full_like(dax, np.nan)
        dt = float(np.median(np.diff(obs_t))) if len(obs_t) > 2 else 0.002
        k = max(1, int(a.settle_window / max(dt, 1e-6)))
        if len(dax) > k:
            cs = np.concatenate(([0.0], np.cumsum(dax)))
            dax_sm[k - 1:] = (cs[k:] - cs[:-k]) / k
        # obs 时间戳可能重复（同 ns 多条）→ gradient 除零出 inf，须一并兜底
        if len(obs_t) > 2:
            dax_rate = np.gradient(dax_sm, obs_t)
        else:
            dax_rate = np.zeros_like(dax)
        dax_rate = np.nan_to_num(dax_rate, nan=1e9, posinf=1e9, neginf=1e9)

        def t_rec_for(eps, mode):
            if not post.any():
                return float("nan")
            if mode == "settled":
                ok = ((np.abs(dphi) < eps) & (np.abs(dth) < eps) &
                      (np.abs(dax_rate) < a.eps_rate))
            else:
                ok = (np.abs(dphi) < eps) & (np.abs(dth) < eps)
            tp, okp = obs_t[post], ok[post]
            run_len = 0.0
            for i in range(1, len(tp)):
                run_len = run_len + (tp[i] - tp[i - 1]) if okp[i] else 0.0
                if run_len >= a.thold:
                    return float(tp[i] - run_len - t_push)
            return float("nan")

        push["t_rec"] = t_rec_for(a.eps, "settled")
        push["t_rec_half"] = t_rec_for(a.eps * 0.5, "settled")
        push["t_rec_dbl"] = t_rec_for(a.eps * 2, "settled")
        push["t_rec_att"] = t_rec_for(a.eps, "att")

        # 恢复期髋滚转力矩峰值（/realTorque idx 0,6）
        tt_tau, vv_tau = data["/realTorque"]
        if len(tt_tau):
            mrec = (tt_tau >= t_push) & (tt_tau <= t_push + 5.0)
            if mrec.any():
                push["hiproll_tau_peak"] = float(np.abs(vv_tau[mrec][:, [0, 6]]).max())
            push["tau_peak_5s"] = float(np.abs(vv_tau[mrec]).max()) if mrec.any() else float("nan")
        # 接触切换数 + 足端滑移峰值（恢复期 5s）
        if len(act_t):
            mrec2 = (act_t >= t_push) & (act_t <= t_push + 5.0)
            if mrec2.any():
                seg = act_v[mrec2]
                foot = np.maximum(seg[:, [0, 2]], seg[:, [1, 3]])
                push["contact_switches_5s"] = int(
                    (np.diff(foot, axis=0) != 0).sum())
        # 足端滑移峰值：/foot_vel_estimate [0:12]=4 点×3D 速度，[12:16]=各点范数
        # （humanoidController.cpp foot_vel 填充）；只统计处于接触的点
        fv_t, fv_v = data["/foot_vel_estimate"]
        if len(fv_t) and fv_v.shape[1] >= 16:
            mrec3 = (fv_t >= t_push) & (fv_t <= t_push + 5.0)
            if mrec3.any():
                ft, fv = fv_t[mrec3], fv_v[mrec3, 12:16]
                act_al = nearest_align(act_t, act_v, ft) if len(act_t) else None
                if act_al is not None and act_al.shape[1] >= 4:
                    slip = fv[act_al[:, :4] > 0]
                    push["foot_slip_peak"] = float(slip.max()) if slip.size else float("nan")
                else:
                    push["foot_slip_peak"] = float(fv.max())
        # 注入后 5s 求解耗时
        for key, tag in (("/mpc_solve_time_ms", "mpc"), ("/wbc_solve_time_ms", "wbc")):
            st, sv = data[key]
            if len(st):
                mm = (st >= t_push) & (st <= t_push + 5.0)
                if mm.any():
                    s = sv[mm, 0]
                    push[f"{tag}_mean_5s"] = float(s.mean())
                    push[f"{tag}_max_5s"] = float(s.max())
                    push[f"{tag}_over_5s"] = 100.0 * float(
                        (s > (MPC_BUDGET_MS if tag == "mpc" else WBC_BUDGET_MS)).mean())
        # 响应缺失 = 推力未送达
        if np.isfinite(push["dphi_max"]) and np.isfinite(push["dax_max"]) and \
           push["dphi_max"] < 0.002 and push["dax_max"] < 0.005:
            problems.insert(0, "注入疑似未生效（无姿态/位移响应）")

        # 倒地判定（口径同 analyze_push）。outcome 与 STATUS 分开：
        # FALLEN/NOT_RECOVERED 是**实验结果**（有效证据），不作废数据；
        # 只有注入前已倒（自发故障）或注入未送达才判 INVALID。
        h_nom = float(np.median(base_z[pre])) if pre.any() else float(np.median(base_z))
        bad = (base_z < 0.75 * h_nom) | (np.abs(roll_o) > np.pi / 4) | \
              (np.abs(pitch_o) > np.pi / 4)
        push["fall_frac_5s"] = float(bad[post].mean()) if post.any() else float("nan")
        push["fallen"] = bool(np.isfinite(push["fall_frac_5s"]) and
                              push["fall_frac_5s"] > 0.5)
        pre_fall = float(bad[pre].mean()) if pre.any() else 0.0
        if pre_fall > 0.5:
            problems.insert(0, f"注入前已倒地（自发故障 {pre_fall*100:.0f}%，非扰动数据）")
            push["outcome"] = "PRE_FALL"
        elif push["fallen"]:
            push["outcome"] = "FALLEN"
        elif np.isfinite(push["t_rec"]):
            push["outcome"] = "RECOVERED"
        else:
            push["outcome"] = "NOT_RECOVERED"
        # 姿态带口径的结果（实机上"稳住"判据被持续迈步的基座摆动卡住，
        # 见 summary 注记；论文恢复指标以姿态带为准，两口径并报）
        if push["outcome"] == "PRE_FALL":
            push["outcome_att"] = "PRE_FALL"
        elif push["fallen"]:
            push["outcome_att"] = "FALLEN"
        elif np.isfinite(push["t_rec_att"]):
            push["outcome_att"] = "RECOVERED"
        else:
            push["outcome_att"] = "NOT_RECOVERED"

    # ---- [6] 硬件表征 ----
    hw = {}
    tt_t, tt_v = data["/jointTemperature"]
    if len(tt_t):
        hw["temp_first_mean"] = float(tt_v[0].mean())
        hw["temp_last_mean"] = float(tt_v[-1].mean())
        hw["temp_rise_mean"] = hw["temp_last_mean"] - hw["temp_first_mean"]
        hw["temp_peak_max"] = float(tt_v.max())
        hw["temp_peak_joint"] = int(np.unravel_index(np.argmax(tt_v), tt_v.shape)[1])
        hw["temp_peak_per_joint"] = tt_v.max(axis=0).round(1).tolist()
    bv_t, bv_v = data["/busVoltage"]
    if len(bv_t):
        v = bv_v[:, 0]
        hw["bus_v_first"] = float(v[0]); hw["bus_v_last"] = float(v[-1])
        hw["bus_v_min"] = float(v.min()); hw["bus_v_max"] = float(v.max())
        hw["bus_v_drop"] = float(v[0] - v[-1])
    bs_t, bs_v = data["/battery_state"]
    if len(bs_t):
        hw["bat_v_first"] = float(bs_v[0, 0]); hw["bat_v_last"] = float(bs_v[-1, 0])
        hw["bat_current_mean"] = float(bs_v[:, 1].mean())
        hw["bat_temp_max"] = float(bs_v[:, 2].max())
        hw["bat_soc_first"] = float(bs_v[0, 4]); hw["bat_soc_last"] = float(bs_v[-1, 4])
    hw["duration_s"] = float((max((t[-1] for t, _ in data.values() if len(t)),
                                  default=0.0) - t0_rec))
    hw["rate_realTorque"] = rates.get("/realTorque", float("nan"))
    hw["rate_obs"] = rates.get("/humanoid_mpc_observation", float("nan"))

    # ---- 倒地/稳定性总判 ----
    # 扰动 run 只判注入**前**（注入致倒是实验结果，由 push outcome 记录，
    # 不作废数据）；base run 判全程。
    if len(base_z):
        pre_only = (obs_t < t_push_wall) if len(fired_t) else np.ones_like(obs_t, bool)
        if pre_only.any():
            bz_p, ro_p, po_p = base_z[pre_only], roll_o[pre_only], pitch_o[pre_only]
            h_med = float(np.median(bz_p))
            bad_all = ((bz_p < 0.70) | (np.abs(ro_p) > np.pi / 4) |
                       (np.abs(po_p) > np.pi / 4))
            fall_frac = float(bad_all.mean())
            if fall_frac > 0.3:
                problems.append(f"倒地/姿态超限({fall_frac*100:.0f}% 样本, 高度中位 {h_med:.2f} m)")
        if track.get("base_z_mean", 1.0) < 0.70:
            problems.append(f"高度均值 {track['base_z_mean']:.2f} m(<0.70, 疑倒地)")

    status = "VALID" if not problems else "INVALID(" + "; ".join(problems) + ")"

    # ---------------- 输出 ----------------
    lines = [f"STATUS: {status}",
             f"run: {name}",
             f"cmd_vx: {cmd} m/s（按 run 名推断）",
             f"steady window: [{win[0]-t0_rec:.2f}, {win[1]-t0_rec:.2f}] s  ({wnote})",
             f"话题速率: obs {f2(rates.get('/humanoid_mpc_observation'),1)} Hz, "
             f"realTorque {f2(rates.get('/realTorque'),1)} Hz, "
             f"contact {f2(rates.get('/realContactFlag'),1)} Hz, "
             f"solve {f2(rates.get('/mpc_solve_time_ms'),1)} Hz",
             "",
             "[1] base tracking (状态估计口径): "
             f"vx = {f2(track.get('vx_mean'))} ± {f2(track.get('vx_std'))} m/s "
             f"(RMSE {f2(track.get('vx_rmse'))}), "
             f"base_z = {f2(track.get('base_z_mean'))} ± {f2(track.get('base_z_std'))} m",
             f"    z_c 质心高度（MuJoCo正解, 6.4.1口径）: {f2(track.get('zc_mean'))} ± "
             f"{f2(track.get('zc_std'),4)} m",
             "",
             "[2] contact timing:"]
    for r in crows:
        lines.append(f"  {r['foot']}: duty {f2(r['duty_planned'])}->{f2(r['duty_actual'])}, "
                     f"agree {f2(r['agree_pct'],1)}%, switch_err {f2(r['switch_err_ms'],1)} ms, "
                     f"switches {r['n_switch_planned']}/{r['n_switch_actual']}")
    if per_point:
        lines.append("    per-point duty actual: " +
                     "  ".join(f"{n}={v:.2f}" for n, v in per_point))
    lines.append("")
    lines.append("[3] torque/joint tracking:")
    if tau:
        lines.append(f"  力矩误差(target-real): rms {f2(tau['rms_total'])} Nm, "
                     f"max {f2(tau['max_abs'])} Nm  (n={tau['n']})")
        lines.append("    逐关节 rms: " + np.array2string(tau["rms_per"], precision=2))
        lines.append("    逐关节 max: " + np.array2string(tau["max_per"], precision=1))
        lines.append(f"  实测力矩幅值: peak {f2(float(np.abs(data['/realTorque'][1]).max()),1)} Nm"
                     f" / 指令幅值 peak {f2(float(np.abs(data['/targetTorque'][1]).max()),1)} Nm")
        lines.append(f"  稳态窗内指令峰值 {f2(tau_peaks.get('cmd_peak'),1)} Nm、"
                     f"实测峰值 {f2(tau_peaks.get('real_peak'),1)} Nm；"
                     f"饱和样本 {sat_counts.get('n_samples_sat',0)} "
                     f"({f2(sat_counts.get('sat_frac_pct'),3)}%)")
        lines.append("    逐关节指令峰值: " + str(tau_peaks.get("cmd_per_joint")))
        lines.append("    逐关节实测峰值: " + str(tau_peaks.get("real_per_joint")))
    if jp:
        lines.append(f"  关节角误差: rms {f2(jp['rms_total'],4)} rad, max {f2(jp['max_abs'],4)} rad")
    lines.append("")
    lines.append("[4] solve time:")
    for tag, s in (("MPC", mpc), ("WBC", wbc)):
        if s:
            lines.append(f"  {tag}: mean {f2(s['mean'],2)} ms, p95 {f2(s['p95'],2)}, "
                         f"p99 {f2(s['p99'],2)}, max {f2(s['max'],2)} | "
                         f"budget {s['budget_ms']:.0f} ms, over {f2(s['over_budget_pct'],2)}%")
    if push:
        lines.append("")
        lines.append(f"[5] push recovery ({push['dir']}, t0={push['t_push']:.2f}s): "
                     f"outcome_settled={push.get('outcome','NA')}, "
                     f"outcome_att={push.get('outcome_att','NA')}")
        lines.append("  注记: 实机 trot-in-place 持续迈步使平滑位移速率/速度维持在约 0.04 m/s 量级，"
                     "「稳住」判据(eps_rate=0.02 m/s)无法满足；恢复指标以姿态带口径(t_rec_att)为准")
        lines.append(f"  dphi_max {f2(push['dphi_max'],4)} rad, dth_max {f2(push['dth_max'],4)} rad, "
                     f"dax_max {f2(push['dax_max'],4)} m")
        lines.append(f"  t_rec {f2(push['t_rec'],2)} s (eps/2 {f2(push['t_rec_half'],2)}, "
                     f"eps*2 {f2(push['t_rec_dbl'],2)}, att {f2(push['t_rec_att'],2)})")
        lines.append(f"  hiproll_tau_peak {f2(push.get('hiproll_tau_peak'),1)} Nm, "
                     f"tau_peak_5s {f2(push.get('tau_peak_5s'),1)} Nm")
        lines.append(f"  contact_switches_5s {push.get('contact_switches_5s','NA')}, "
                     f"foot_slip_peak {f2(push.get('foot_slip_peak'),3)} m/s")
        lines.append(f"  solve 5s: mpc mean {f2(push.get('mpc_mean_5s'),2)}/max {f2(push.get('mpc_max_5s'),2)} "
                     f"over {f2(push.get('mpc_over_5s'),2)}%; "
                     f"wbc mean {f2(push.get('wbc_mean_5s'),2)}/max {f2(push.get('wbc_max_5s'),2)} "
                     f"over {f2(push.get('wbc_over_5s'),2)}%")
        lines.append(f"  fall_frac_5s {f2(push.get('fall_frac_5s'),3)}, fallen {push.get('fallen')}")
    lines.append("")
    lines.append("[6] hardware:")
    lines.append(f"  duration {f2(hw.get('duration_s'),1)} s")
    lines.append(f"  关节温度: 首均 {f2(hw.get('temp_first_mean'),1)}°C → 末均 {f2(hw.get('temp_last_mean'),1)}°C "
                 f"(Δ{f2(hw.get('temp_rise_mean'),1)}), 全场峰值 {f2(hw.get('temp_peak_max'),1)}°C "
                 f"(关节 {hw.get('temp_peak_joint','NA')})")
    lines.append(f"  母线电压: {f2(hw.get('bus_v_first'),2)} → {f2(hw.get('bus_v_last'),2)} V "
                 f"(min {f2(hw.get('bus_v_min'),2)}, drop {f2(hw.get('bus_v_drop'),2)} V)")
    lines.append(f"  电池: V {f2(hw.get('bat_v_first'),2)}→{f2(hw.get('bat_v_last'),2)}, "
                 f"电流均值 {f2(hw.get('bat_current_mean'),2)} A, 温度峰值 {f2(hw.get('bat_temp_max'),1)}°C, "
                 f"SOC {f2(hw.get('bat_soc_first'),3)}→{f2(hw.get('bat_soc_last'),3)}")
    summary = "\n".join(lines) + "\n"
    open(os.path.join(out_dir, "summary.txt"), "w").write(summary)
    print(summary)

    # metrics.csv（一行平铺，便于聚合）
    flat_row = dict(run=name, cmd=cmd, status=status,
                    **{f"track_{k}": v for k, v in track.items()},
                    **{f"hw_{k}": v for k, v in hw.items()},
                    **{f"push_{k}": v for k, v in push.items()},
                    **{f"tau_{k}": v for k, v in tau_peaks.items()},
                    **{f"sat_{k}": v for k, v in sat_counts.items()},
                    **{f"slip_{k}": v for k, v in slip_stats.items()})
    for r in crows:
        for k in ("duty_planned", "duty_actual", "agree_pct", "switch_err_ms",
                  "n_switch_planned", "n_switch_actual"):
            flat_row[f"contact_{r['foot']}_{k}"] = r[k]
    if tau:
        flat_row["tau_rms"] = tau["rms_total"]; flat_row["tau_max"] = tau["max_abs"]
    if jp:
        flat_row["joint_rms"] = jp["rms_total"]; flat_row["joint_max"] = jp["max_abs"]
    for tag, s in (("mpc", mpc), ("wbc", wbc)):
        if s:
            for k in ("mean", "p95", "p99", "max", "over_budget_pct"):
                flat_row[f"solve_{tag}_{k}"] = s[k]
    import csv
    cols = list(flat_row)
    with open(os.path.join(out_dir, "metrics.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader(); w.writerow(flat_row)
    print(f"[analyze_real] outputs -> {out_dir}")


if __name__ == "__main__":
    main()
