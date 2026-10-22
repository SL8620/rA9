#!/usr/bin/env python3
"""C6 回填底稿：A1/A2 聚合统计（交付需求 §D 槽位映射的数字来源）。
输出 deliverables/C6/analysis/summary_stats.md"""
import glob, os, sys
import numpy as np
RA9 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ROS 环境自举：必须在 import make_fig6 之前设好 AMENT_PREFIX_PATH，
# 否则 make_fig6 顶部的 os.execvp 会把本进程整个替换成 make_fig6.py，
# 聚合代码一行都跑不到（2026-10-21 修）。
if "AMENT_PREFIX_PATH" not in os.environ:
    _cmd = ('source /opt/ros/jazzy/setup.bash 2>/dev/null; '
            'source %s/install/setup.bash 2>/dev/null; exec python3 "$0" "$@"' % RA9)
    os.execvp("bash", ["bash", "-c", _cmd, os.path.abspath(__file__)] + sys.argv[1:])

sys.path.insert(0, os.path.join(RA9, "scripts"))
os.chdir(RA9)
import importlib.util
spec = importlib.util.spec_from_file_location("mf6", os.path.join(RA9, "scripts/make_fig6.py"))
mf6 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mf6)
load_run = mf6.load_run

def f(x, n=2): return f"{x:.{n}f}" if np.isfinite(x) else "NA"

lines = ["# C6 回填底稿（A1/A2 聚合统计）", "",
         "数字均可由 `analyze_walk.py`/`analyze_push.py` 从 sim_data/ 重算；",
         "统计窗=各 run 稳态窗；数值取 3 个有效 run 的均值（±run 间 std）。", ""]
lines += ["## 6.4.1 基座跟踪（A1，三速度 × 3 run）", "",
          "| 指标 | v00 (0.0) | v01 (0.1) | v03 (0.3) |", "|---|---|---|---|"]
rows = {"vx_mean": [], "vx_std": [], "vx_rmse": [], "height_mean": [], "height_std": [],
        "zc_mean": [], "zc_std": []}
for v, names in mf6.RUNS.items():
    stat = {k: [] for k in rows}
    for n in names:
        p = os.path.join(mf6.run_dir(n), "analysis", "metrics.csv")
        import csv as _csv
        for r in _csv.DictReader(open(p)):
            if r["item"] in stat and r["value"] not in ("", "nan"):
                stat[r["item"]].append(float(r["value"]))
    for k in rows:
        rows[k].append(np.mean(stat[k]) if stat[k] else float("nan"))
for k, label, n in (("vx_mean", "vx mean [m/s]", 3), ("vx_std", "vx std [m/s]", 3),
                    ("vx_rmse", "vx RMSE [m/s]", 3),
                    ("zc_mean", "z_c 质心高 mean [m]（MuJoCo正解）", 3),
                    ("zc_std", "z_c 质心高 std [m]", 4),
                    ("height_mean", "基座高 mean [m]", 3),
                    ("height_std", "基座高 std [m]", 3)):
    lines.append(f"| {label} | " + " | ".join(f(x, n) for x in rows[k]) + " |")

lines += ["", "## 6.4.2 接触时序（A1）", "",
          "| 指标 | v00 | v01 | v03 |", "|---|---|---|---|"]
agree, serr, dP, dA = [], [], [], []
for v, names in mf6.RUNS.items():
    a, s, dp, da = [], [], [], []
    for n in names:
        import csv as _csv
        for r in _csv.DictReader(open(os.path.join(mf6.run_dir(n), "analysis", "metrics.csv"))):
            it = r["item"]
            try: val = float(r["value"])
            except Exception: continue
            if it.endswith(".agree_pct"): a.append(val)
            if it.endswith(".switch_err_ms"): s.append(val)
            if it.endswith(".duty_planned"): dp.append(val)
            if it.endswith(".duty_actual"): da.append(val)
    agree.append(np.mean(a)); serr.append(np.mean(s))
    dP.append(np.mean(dp)); dA.append(np.mean(da))
for label, arr, n in (("duty 计划", dP, 2), ("duty 实测", dA, 2),
                      ("agree% ", agree, 1), ("switch_err [ms]", serr, 1)):
    lines.append(f"| {label} | " + " | ".join(f(x, n) for x in arr) + " |")

lines += ["", "## 6.4.3 力矩/关节跟踪（A1）", "",
          "| 指标 | v00 | v01 | v03 |", "|---|---|---|---|"]
t_rms, t_max, q_rms, q_max, pd_rms, clip_rms = [], [], [], [], [], []
for v, names in mf6.RUNS.items():
    tr, tm, qr, qm, pr, cr = [], [], [], [], [], []
    for n in names:
        import csv as _csv
        for r in _csv.DictReader(open(os.path.join(mf6.run_dir(n), "analysis", "metrics.csv"))):
            if r["category"] == "torque_tracking" and r["item"] == "rms_total": tr.append(float(r["value"]))
            if r["category"] == "torque_tracking" and r["item"] == "max_abs": tm.append(float(r["value"]))
            if r["category"] == "torque_tracking" and r["item"] == "pd_correction_rms": pr.append(float(r["value"]))
            if r["category"] == "torque_tracking" and r["item"] == "clip_loss_rms": cr.append(float(r["value"]))
            if r["category"] == "joint_tracking" and r["item"] == "rms_total_excl": qr.append(float(r["value"]))
            if r["category"] == "joint_tracking" and r["item"] == "max_abs_excl": qm.append(float(r["value"]))
    t_rms.append(np.mean(tr)); t_max.append(np.max(tm))
    q_rms.append(np.mean(qr)); q_max.append(np.max(qm))
    pd_rms.append(np.mean(pr) if pr else float("nan"))
    clip_rms.append(np.mean(cr) if cr else float("nan"))
for label, arr, n in (("力矩跟踪误差 RMS [N·m]（=PD修正+钳制损失）", t_rms, 2),
                      ("力矩跟踪误差 max [N·m]", t_max, 1),
                      ("其中 PD 修正量 RMS [N·m]", pd_rms, 2),
                      ("其中 钳制损失 RMS [N·m]", clip_rms, 3),
                      ("关节角误差 RMS [rad]（剔除瞬态窗）", q_rms, 4),
                      ("关节角误差 max [rad]（剔除瞬态窗）", q_max, 3)):
    lines.append(f"| {label} | " + " | ".join(f(x, n) for x in arr) + " |")
lines += ["",
          "> ⚠️ **口径**（NOTE.md §12.2/§12.4）：`/realTorque` 语义于 2026-10-12 更正，",
          "> 旧 bag 由恒等式 `qfrc_actuator=clip(data.ctrl,±LIM)` **分析层回填**（残差 0.0000 Nm）。",
          "> 「力矩跟踪误差」= `/targetTorque` − 执行器出力 = **PD 修正量 + 钳制损失**，",
          "> 实测钳制损失≈0，故该指标**实质是 PD 修正量的幅值**，不是执行器动力学跟踪误差",
          "> （仿真执行器为理想力矩源，无动力学可跟踪）。**不可与真机同名指标直接对比。**",
          "> ⚠️ **max 受两处低估**：旧 bag `/realTorque` 仅 29.4 Hz（漏 37~59% 峰值，",
          "> 已改 500 Hz）；且峰值集中在瞬态垃圾窗（§7.5）。报峰值请用 `/targetTorque`。"]

lines += ["", "## 6.4.4 求解耗时（A1 全部 9 run 聚合）", ""]
mpc, wbc = [], []
for names in mf6.RUNS.values():
    for n in names:
        d = load_run(n)
        mpc.append(d["mpc_solve_time_ms"][:, 1]); wbc.append(d["wbc_solve_time_ms"][:, 1])
mpc = np.concatenate(mpc); wbc = np.concatenate(wbc)
for name, a, budget in (("MPC", mpc, 10), ("WBC", wbc, 2)):
    lines.append(f"- **{name}**：mean {f(a.mean())}、p95 {f(np.percentile(a,95))}、"
                 f"p99 {f(np.percentile(a,99))}、max {f(a.max())} ms；"
                 f"超预算率 {100*(a>budget).mean():.2f}%（预算 {budget} ms）")

import re


def _grab(pat, txt):
    m = re.search(pat, txt)
    return m.group(1) if m else "NA"


def _fnum(x):
    try:
        return float(x)
    except Exception:
        return float("nan")


def _ms(txt, label):
    """mean±run间std；空列表回 NA。"""
    ok = [x for x in txt if np.isfinite(x)]
    if not ok:
        return "NA"
    return f"{np.mean(ok):.4f}±{np.std(ok):.4f}" if len(ok) > 1 else f"{np.mean(ok):.4f}"


lines += ["", "## 6.4.5 扰动与恢复（A2，各格有效样本）", "",
          "FALLEN（`STATUS: INVALID(倒地…)`）不进统计，单列于表后；`n` 为该格有效样本数。",
          "",
          "| 工况 | n(REC/FALLEN) | dphi_max [rad] | dth_max [rad] | dy_max [m] | "
          "t_rec [s] | t_rec_att [s] | hiproll_tau_peak [N.m] | slip_peak [m/s] |",
          "|---|---|---|---|---|---|---|---|---|"]
fallen_rows = []
for cell in ("lat_L1_stand", "lat_L2_stand", "lat_L3_stand", "fwd_L1_stand",
             "fwd_L2_stand", "fwd_L3_stand", "lat_L1_walk", "lat_L2_walk", "lat_L3_walk"):
    cols = {k: [] for k in ("dp", "dth", "dy", "tr", "tra", "hp", "sl")}
    n_fall = 0
    for p in sorted(glob.glob(f"sim_data/sim_push_{cell}_*/analysis/push_summary.txt")):
        txt = open(p).read()
        if "注入疑似未生效" in txt or "注入前已倒地" in txt:
            continue
        run = p.split(os.sep)[-3]
        if txt.startswith("STATUS: INVALID"):
            n_fall += 1
            fallen_rows.append((run, _grab(r"dphi_max=([0-9.]+)", txt),
                                _grab(r"dy_max=([0-9.]+)", txt),
                                _grab(r"hiproll_tau_peak=([0-9.]+)", txt)))
            continue
        cols["dp"].append(_fnum(_grab(r"dphi_max=([0-9.]+)", txt)))
        cols["dth"].append(_fnum(_grab(r"dtheta_max=([0-9.]+)", txt)))
        cols["dy"].append(_fnum(_grab(r"dy_max=([0-9.]+)", txt)))
        cols["hp"].append(_fnum(_grab(r"hiproll_tau_peak=([0-9.]+)", txt)))
        cols["sl"].append(_fnum(_grab(r"slip_peak=([0-9.]+)", txt)))
        cols["tr"].append(_fnum(_grab(r"t_rec=([0-9.]+) ", txt)
                                if _grab(r"t_rec=([0-9.]+) ", txt) != "NA"
                                else _grab(r"t_rec=(NA\S*)", txt)))
        cols["tra"].append(_fnum(_grab(r"t_rec_att\(仅姿态 eps\) -> ([0-9.]+|NA)", txt)))
    n_ok = len(cols["dp"])
    lines.append(f"| {cell} | {n_ok}/{n_fall} | " +
                 " | ".join(_ms(cols[k], k) for k in
                            ("dp", "dth", "dy", "tr", "tra", "hp", "sl")) + " |")

lines += ["", "FALLEN 样本（推力致倒，按交付需求 D 保留为证据，不进上表统计）：", "",
          "| run | dphi_max [rad] | dy_max [m] | hiproll_tau_peak [N.m] |", "|---|---|---|---|"]
for run, dp, dy, hp in fallen_rows:
    lines.append(f"| {run} | {dp} | {dy} | {hp} |")

out = os.path.join(RA9, "deliverables/C6/analysis/summary_stats.md")
open(out, "w").write("\n".join(lines) + "\n")
print("written", out)
print("\n".join(lines[5:60]))
