#!/usr/bin/env python3
"""C6 回填底稿：A1/A2 聚合统计（交付需求 §D 槽位映射的数字来源）。
输出 deliverables/C6/analysis/summary_stats.md"""
import glob, os, sys
import numpy as np
RA9 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RA9, "scripts"))
os.chdir(RA9)
import importlib.util
spec = importlib.util.spec_from_file_location("mf6", os.path.join(RA9, "scripts/make_fig6.py"))
mf6 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mf6)
load_run = mf6.load_run

def f(x, n=2): return f"{x:.{n}f}" if np.isfinite(x) else "NA"

lines = ["# C6 回填底稿（A1/A2 聚合统计）", "",
         "数字均可由 `analyze_walk.py`/`analyze_push.py` 从 experiments/ 重算；",
         "统计窗=各 run 稳态窗；数值取 3 个有效 run 的均值（±run 间 std）。", ""]
lines += ["## 6.4.1 基座跟踪（A1，三速度 × 3 run）", "",
          "| 指标 | v00 (0.0) | v01 (0.1) | v03 (0.3) |", "|---|---|---|---|"]
rows = {"vx_mean": [], "vx_std": [], "vx_rmse": [], "height_mean": [], "height_std": []}
for v, names in mf6.RUNS.items():
    stat = {k: [] for k in rows}
    for n in names:
        p = os.path.join(RA9, "experiments", n, "analysis", "metrics.csv")
        import csv as _csv
        for r in _csv.DictReader(open(p)):
            if r["item"] in stat and r["value"] not in ("", "nan"):
                stat[r["item"]].append(float(r["value"]))
    for k in rows:
        rows[k].append(np.mean(stat[k]) if stat[k] else float("nan"))
for k, label, n in (("vx_mean", "vx mean [m/s]", 3), ("vx_std", "vx std [m/s]", 3),
                    ("vx_rmse", "vx RMSE [m/s]", 3), ("height_mean", "基座高 mean [m]", 3),
                    ("height_std", "基座高 std [m]", 3)):
    lines.append(f"| {label} | " + " | ".join(f(x, n) for x in rows[k]) + " |")

lines += ["", "## 6.4.2 接触时序（A1）", "",
          "| 指标 | v00 | v01 | v03 |", "|---|---|---|---|"]
agree, serr, dP, dA = [], [], [], []
for v, names in mf6.RUNS.items():
    a, s, dp, da = [], [], [], []
    for n in names:
        import csv as _csv
        for r in _csv.DictReader(open(os.path.join(RA9, "experiments", n, "analysis", "metrics.csv"))):
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
t_rms, t_max, q_rms, q_max = [], [], [], []
for v, names in mf6.RUNS.items():
    tr, tm, qr, qm = [], [], [], []
    for n in names:
        import csv as _csv
        for r in _csv.DictReader(open(os.path.join(RA9, "experiments", n, "analysis", "metrics.csv"))):
            if r["category"] == "torque_tracking" and r["item"] == "rms_total": tr.append(float(r["value"]))
            if r["category"] == "torque_tracking" and r["item"] == "max_abs": tm.append(float(r["value"]))
            if r["category"] == "joint_tracking" and r["item"] == "rms_total_excl": qr.append(float(r["value"]))
            if r["category"] == "joint_tracking" and r["item"] == "max_abs_excl": qm.append(float(r["value"]))
    t_rms.append(np.mean(tr)); t_max.append(np.max(tm))
    q_rms.append(np.mean(qr)); q_max.append(np.max(qm))
for label, arr, n in (("力矩误差 RMS [N·m]", t_rms, 2), ("力矩误差 max [N·m]", t_max, 1),
                      ("关节角误差 RMS [rad]（剔除瞬态窗）", q_rms, 4), ("关节角误差 max [rad]（剔除瞬态窗）", q_max, 3)):
    lines.append(f"| {label} | " + " | ".join(f(x, n) for x in arr) + " |")

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

lines += ["", "## 6.4.5 扰动与恢复（A2，各格有效样本）", "",
          "| 工况 | n | dphi_max [rad] | dy_max [m] | t_rec [s] | t_rec_att [s] |",
          "|---|---|---|---|---|---|"]
for cell in ("lat_L1_stand", "lat_L2_stand", "lat_L3_stand", "fwd_L1_stand",
             "fwd_L2_stand", "fwd_L3_stand", "lat_L1_walk", "lat_L2_walk", "lat_L3_walk"):
    dp, dy, tr, tra = [], [], [], []
    for p in sorted(glob.glob(f"experiments/sim_push_{cell}_*/analysis/push_summary.txt")):
        txt = open(p).read()
        if "注入疑似未生效" in txt or "注入前已倒地" in txt:
            continue
        import re
        def grab(pat):
            m = re.search(pat, txt); return m.group(1) if m else "NA"
        dp.append(float(grab(r"dphi_max=([0-9.]+)")))
        dy.append(float(grab(r"dy_max=([0-9.]+)")))
        t = grab(r"t_rec=([0-9.]+) ")
        if t == "NA":
            t = grab(r"t_rec=(NA\S*)")
        tr.append(float(t) if t.replace(".", "").isdigit() else float("nan"))
        x = grab(r"t_rec_att\(仅姿态 eps\) -> ([0-9.]+|NA)")
        tra.append(float(x) if x not in ("NA",) and x.replace(".", "").isdigit() else float("nan"))
    n_ = len(dp)
    tr_ok = [x for x in tr if np.isfinite(x)]
    tra_ok = [x for x in tra if np.isfinite(x)]
    lines.append(f"| {cell} | {n_} | {f(np.mean(dp),4)} | {f(np.mean(dy),4)} | "
                 f"{f(np.mean(tr_ok)) if tr_ok else 'NA(多未恢复)'} | "
                 f"{f(np.mean(tra_ok)) if tra_ok else 'NA'} |")

out = os.path.join(RA9, "deliverables/C6/analysis/summary_stats.md")
open(out, "w").write("\n".join(lines) + "\n")
print("written", out)
print("\n".join(lines[5:60]))
