#!/usr/bin/env python3
"""扰动 run 专用分析（C6交付需求 B1）。

用法: analyze_push.py <run_dir> [--eps 0.05] [--eps-pos 0.02] [--thold 1.0] [--out DIR]
输入: run 目录（bag/ + params.txt，push_* 字段记录注入参数）
输出: <out>/push_summary.txt + fig5_recovery.png

指标（相对注入时刻 t=0）:
  dphi_max/dtheta_max  峰值滚转/俯仰偏离（对注入前 1s 基线）
  dy_max               沿推力方向轴的最大质心偏移（lat=y, fwd=x）
  t_rec                恢复时间：|姿态偏差|<eps 且 |位置偏差|<eps_pos 持续 thold 秒
                       （附 eps×0.5 / ×2 敏感性）；倒地则 t_rec=NA(FALLEN)
  hiproll_tau_peak     恢复期髋滚转力矩峰值（/realTorque idx 0,6；对标 143.89/220 Nm）
  solve_time_push      注入后 5s 内 mpc/wbc 耗时统计
  contact events       恢复期接触切换数；足端滑移峰值（/foot_vel_estimate 范数）
"""
import argparse
import math
import os
import sys

if "AMENT_PREFIX_PATH" not in os.environ:
    _cmd = 'source /opt/ros/jazzy/setup.bash 2>/dev/null; exec python3 "$0" "$@"'
    os.execvp("bash", ["bash", "-c", _cmd, os.path.abspath(__file__)] + sys.argv[1:])

import numpy as np

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ModuleNotFoundError:
    import glob
    for p in glob.glob("/opt/ros/*/lib/python3*/site-packages"):
        sys.path.insert(0, p)
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANT = {"/ground_truth/state", "/sim_time", "/realTorque", "/targetTorque",
        "/mpc_solve_time_ms", "/wbc_solve_time_ms", "/simContactFlag",
        "/foot_vel_estimate"}


def load(bag_dir):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()
             if t.name in WANT}
    out = {k: [] for k in WANT}
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic in types:
            out[topic].append((ts * 1e-9, deserialize_message(data, types[topic])))
    return out


def read_params(run_dir):
    p = {}
    with open(os.path.join(run_dir, "params.txt")) as f:
        for line in f:
            if "=" in line:
                k, v = line.split("=", 1)
                p[k.strip()] = v.strip()
    return p


def quat_to_rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--eps-pos", type=float, default=0.02)
    ap.add_argument("--thold", type=float, default=1.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run = a.run.rstrip("/")
    out_dir = a.out or os.path.join(run, "analysis")
    os.makedirs(out_dir, exist_ok=True)

    params = read_params(run)
    data = load(os.path.join(run, "bag"))
    problems = []

    # ---- 注入时刻（sim 时间 → 墙钟）----
    push_trig_sim = float(params.get("push_time_sim_trigger", "nan").split()[0])
    st = data["/sim_time"]
    if not st:
        problems.append("缺 /sim_time（无法定位注入时刻）")
        summary = "STATUS: INVALID(" + "; ".join(problems) + ")\n"
        open(os.path.join(out_dir, "push_summary.txt"), "w").write(summary)
        print(summary)
        sys.exit(1)
    stt = np.array([t for t, _ in st])
    stv = np.array([m.data for _, m in st])
    if math.isfinite(push_trig_sim):
        t_push_wall = float(np.interp(push_trig_sim, stv, stt))
    else:
        t_push_wall = float(stt[0])

    # ---- 状态量 ----
    gt = data["/ground_truth/state"]
    gt_t = np.array([t for t, _ in gt])
    roll = np.array([quat_to_rpy(m.pose.pose.orientation)[0] for _, m in gt])
    pitch = np.array([quat_to_rpy(m.pose.pose.orientation)[1] for _, m in gt])
    px = np.array([m.pose.pose.position.x for _, m in gt])
    py = np.array([m.pose.pose.position.y for _, m in gt])
    pz = np.array([m.pose.pose.position.z for _, m in gt])
    push_dir = params.get("push_dir", "lat")
    axis = py if push_dir == "lat" else px

    # 基线 = 注入前 1s 均值
    pre = (gt_t >= t_push_wall - 1.0) & (gt_t < t_push_wall)
    if not pre.any():
        problems.append("注入前 1s 无数据")
        pre = gt_t < t_push_wall
    r0, p0, a0 = roll[pre].mean(), pitch[pre].mean(), axis[pre].mean()

    # ---- 峰值偏离（注入后 5s 内）----
    post = (gt_t >= t_push_wall) & (gt_t <= t_push_wall + 5.0)
    if not post.any():
        problems.append("注入后无数据")
    dphi = roll - r0
    dth = pitch - p0
    dax = axis - a0
    dphi_max = float(np.abs(dphi[post]).max()) if post.any() else float("nan")
    dth_max = float(np.abs(dth[post]).max()) if post.any() else float("nan")
    dy_max = float(np.abs(dax[post]).max()) if post.any() else float("nan")

    # ---- 恢复时间 t_rec（带保持时长；敏感性 eps×0.5/×2）----
    def t_rec_for(eps, eps_pos, thold):
        if not post.any():
            return float("nan")
        ok = (np.abs(dphi) < eps) & (np.abs(dax) < eps_pos) & (np.abs(dth) < eps)
        t_post = gt_t[post]
        ok_post = ok[post]
        need = thold
        run_len = 0.0
        for i in range(1, len(t_post)):
            run_len = run_len + (t_post[i] - t_post[i - 1]) if ok_post[i] else 0.0
            if run_len >= need:
                # t_rec = 进入误差带的时刻 − 注入时刻（保持 thold 秒后确认）
                return float(t_post[i] - run_len - t_push_wall)
        return float("nan")

    t_rec = t_rec_for(a.eps, a.eps_pos, a.thold)
    t_rec_half = t_rec_for(a.eps * 0.5, a.eps_pos * 0.5, a.thold)
    t_rec_dbl = t_rec_for(a.eps * 2, a.eps_pos * 2, a.thold)

    # ---- 倒地判定（复用 analyze_walk 口径：姿态>45° 或高度<0.75×基线）----
    h_nom = float(np.median(pz[pre])) if pre.any() else float(np.median(pz))
    fall_frac = float((((pz < 0.75 * h_nom) | (np.abs(roll) > np.pi / 4) |
                        (np.abs(pitch) > np.pi / 4))[post]).mean()) if post.any() else float("nan")
    fallen = np.isfinite(fall_frac) and fall_frac > 0.5
    # 注入前就已倒地 = 自发故障（控制器垃圾关节目标等），不是扰动数据
    pre_fall = float((((pz < 0.75 * h_nom) | (np.abs(roll) > np.pi / 4) |
                       (np.abs(pitch) > np.pi / 4))[pre]).mean()) if pre.any() else 0.0
    if pre_fall > 0.5:
        problems.insert(0, f"注入前已倒地（自发故障 {pre_fall*100:.0f}%，非扰动数据）")
        fallen = True
    elif fallen:
        problems.append(f"倒地未恢复（{fall_frac*100:.0f}% 样本越限）")
    if not np.isfinite(t_rec) and not fallen:
        problems.append("5s 内未恢复（t_rec=NA）")

    # ---- 髋滚转力矩峰值（恢复期 5s；idx0=L,idx6=R）----
    rt = data["/realTorque"]
    rt_t = np.array([t for t, _ in rt]) if rt else np.zeros(0)
    hip = np.array([max(abs(m.data[0]), abs(m.data[6])) for _, m in rt]) \
        if rt else np.zeros(0)
    m5 = (rt_t >= t_push_wall) & (rt_t <= t_push_wall + 5.0)
    hip_peak = float(hip[m5].max()) if m5.any() else float("nan")

    # ---- solve-time（注入后 5s）----
    def st_stats(key):
        v = data[key]
        if not v:
            return None
        t = np.array([x for x, _ in v])
        s = np.array([m.data for _, m in v])
        m = (t >= t_push_wall) & (t <= t_push_wall + 5.0)
        s = s[m] if m.any() else s[:0]
        if len(s) == 0:
            return None
        return dict(mean=float(s.mean()), p99=float(np.percentile(s, 99)),
                    max=float(s.max()))

    mpc_st = st_stats("/mpc_solve_time_ms")
    wbc_st = st_stats("/wbc_solve_time_ms")

    # ---- 接触切换数与足端滑移峰值（恢复期 5s）----
    sc = data["/simContactFlag"]
    sc_t = np.array([t for t, _ in sc]) if sc else np.zeros(0)
    sc_v = np.array([[x for x in m.data] for _, m in sc]) if sc else np.zeros((0, 4))
    m5s = (sc_t >= t_push_wall) & (sc_t <= t_push_wall + 5.0)
    n_switch = int(np.sum(np.abs(np.diff(sc_v[m5s], axis=0)).sum(axis=1) > 0)) \
        if m5s.any() and m5s.sum() > 1 else 0
    fv = data["/foot_vel_estimate"]
    fv_t = np.array([t for t, _ in fv]) if fv else np.zeros(0)
    slip = np.array([max(abs(x) for x in m.data) for _, m in fv]) if fv else np.zeros(0)
    m5f = (fv_t >= t_push_wall) & (fv_t <= t_push_wall + 5.0)
    slip_peak = float(slip[m5f].max()) if m5f.any() else float("nan")

    status = "VALID" if not problems else "INVALID(" + "; ".join(problems) + ")"
    imp = params.get("push_impulse", "?")
    state = "stand" if abs(float(params.get("cmd_vx", "0") or 0)) < 1e-6 else "walk"

    lines = [
        f"STATUS: {status}",
        f"push: dir={push_dir} impulse={imp} N.s state={state} "
        f"duration_ms={params.get('push_duration_ms','?')} "
        f"time_sim={push_trig_sim:.3f} (wall t0={t_push_wall - gt_t[0]:.2f}s)",
        f"dphi_max={dphi_max:.4f} rad, dtheta_max={dth_max:.4f} rad, "
        f"dy_max={dy_max:.4f} m, t_rec={t_rec:.2f} s"
        if np.isfinite(t_rec) else
        f"dphi_max={dphi_max:.4f} rad, dtheta_max={dth_max:.4f} rad, "
        f"dy_max={dy_max:.4f} m, t_rec=NA({'FALLEN' if fallen else 'NOT_RECOVERED'})",
        f"t_rec sensitivity: eps*0.5 -> {t_rec_half if np.isfinite(t_rec_half) else 'NA'} s, "
        f"eps*2 -> {t_rec_dbl if np.isfinite(t_rec_dbl) else 'NA'} s (thold={a.thold}s)",
        f"hiproll_tau_peak={hip_peak:.2f} Nm (table2.4: need 143.89, peak 220)",
        ("solve_time_push: mpc mean/p99/max="
         + "/".join(f"{mpc_st[k]:.2f}" for k in ("mean", "p99", "max"))
         if mpc_st else "solve_time_push: mpc NA")
        + (", wbc " + "/".join(f"{wbc_st[k]:.2f}" for k in ("mean", "p99", "max"))
           if wbc_st else ", wbc NA"),
        f"contact events: switches={n_switch}, slip_peak={slip_peak:.3f} m/s",
        f"fall_frac={fall_frac:.2f} (h_nom={h_nom:.3f} m)",
    ]
    summary = "\n".join(lines)
    with open(os.path.join(out_dir, "push_summary.txt"), "w") as f:
        f.write(summary + "\n")
    print(summary)

    # ---- fig5: 2×2 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t_rel = gt_t - t_push_wall
    fig, ax = plt.subplots(2, 2, figsize=(10, 7))
    w = (t_rel >= -1) & (t_rel <= 6)
    ax[0, 0].plot(t_rel[w], dphi[w], label="roll dev")
    ax[0, 0].axhline(a.eps, color="r", ls="--", lw=0.8)
    ax[0, 0].axhline(-a.eps, color="r", ls="--", lw=0.8)
    if np.isfinite(t_rec):
        ax[0, 0].axvline(t_rec, color="g", ls=":", label=f"t_rec={t_rec:.2f}s")
    ax[0, 0].set_xlabel("t - t_push [s]"); ax[0, 0].set_ylabel("roll dev [rad]")
    ax[0, 0].legend(fontsize=8); ax[0, 0].grid(alpha=.3)
    ax[0, 1].plot(t_rel[w], dax[w], label=f"{push_dir} offset")
    ax[0, 1].axhline(a.eps_pos, color="r", ls="--", lw=0.8)
    ax[0, 1].axhline(-a.eps_pos, color="r", ls="--", lw=0.8)
    ax[0, 1].set_xlabel("t - t_push [s]"); ax[0, 1].set_ylabel("offset [m]")
    ax[0, 1].legend(fontsize=8); ax[0, 1].grid(alpha=.3)
    if m5.any():
        ax[1, 0].plot(rt_t[m5] - t_push_wall, hip[m5], label="|hip roll| peak")
        ax[1, 0].axhline(143.89, color="orange", ls="--", lw=0.8, label="143.89 Nm (需求)")
        ax[1, 0].axhline(220, color="r", ls="--", lw=0.8, label="220 Nm (峰值)")
    ax[1, 0].set_xlabel("t - t_push [s]"); ax[1, 0].set_ylabel("tau [Nm]")
    ax[1, 0].legend(fontsize=8); ax[1, 0].grid(alpha=.3)
    ax[1, 1].bar(["eps*0.5", "eps", "eps*2"],
                 [x if np.isfinite(x) else 0 for x in (t_rec_half, t_rec, t_rec_dbl)])
    ax[1, 1].set_ylabel("t_rec [s]"); ax[1, 1].set_title("t_rec sensitivity")
    ax[1, 1].grid(alpha=.3, axis="y")
    fig.suptitle(f"{os.path.basename(run)}  push {push_dir} {imp} N·s ({state})")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig5_recovery.png"), dpi=300)
    plt.close(fig)
    print(f"\n[analyze_push] outputs -> {out_dir}")


if __name__ == "__main__":
    main()
