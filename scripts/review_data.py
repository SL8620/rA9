#!/usr/bin/env python3
"""C6 采集数据质量评审：逐关节/逐 run/扰动样本深挖。
输出到 stdout。数据源 /tmp/fig6_cache（A1）与 experiments/*/analysis（A2）。"""
import glob
import os

import numpy as np

os.chdir("/home/mi/disk/SS/thesis/rA9")

RUNS = ["sim_base_v00_01", "sim_base_v00_03", "sim_base_v00_04",
        "sim_base_v01_01", "sim_base_v01_02", "sim_base_v01_03",
        "sim_base_v03_02", "sim_base_v03_04", "sim_base_v03_06"]
JNAME = ["L髋滚转", "L髋偏航", "L髋俯仰", "L膝", "L踝俯仰", "L踝滚转",
         "R髋滚转", "R髋偏航", "R髋俯仰", "R膝", "R踝俯仰", "R踝滚转"]
LIM = [220, 396, 396, 396, 112, 112] * 2


def section(t):
    print("\n" + "=" * 66 + f"\n{t}\n" + "=" * 66)


# ---------------- A1 逐关节 ----------------
section("A1 逐关节质量（9 run）")
tau_rms, tau_absmax = [], []
q_rms, q_max, q_when = [], [], []
per_run = []
for n in RUNS:
    z = np.load(f"/tmp/fig6_cache/{n}.npz")
    tq = z["realTorque"][:, 1:]
    t_tp, t_jv = z["targetPos"][:, 0], z["jointsPosVel"][:, 0]
    idx = np.clip(np.searchsorted(t_jv, t_tp), 1, len(t_jv) - 1)
    left = np.abs(t_tp - t_jv[idx - 1]) <= np.abs(t_tp - t_jv[idx])
    jv = z["jointsPosVel"][np.where(left, idx - 1, idx), 1:13]
    qe = z["targetPos"][:, 1:13] - jv
    tau_rms.append(np.sqrt((tq ** 2).mean(0)))
    tau_absmax.append(np.abs(tq).max(0))
    q_rms.append(np.sqrt((qe ** 2).mean(0)))
    q_max.append(np.abs(qe).max(0))
    tt = t_tp - t_tp[0]
    q_when.append(tt[np.abs(qe).argmax(0)])
    per_run.append((n, float(np.sqrt((qe ** 2).mean())),
                    float(np.abs(qe).max()), float(np.abs(tq).max())))
tau_rms = np.mean(tau_rms, 0)
tau_absmax = np.max(tau_absmax, 0)
q_rms = np.mean(q_rms, 0)
q_max = np.max(q_max, 0)
q_when = np.array(q_when)
print(f"{'关节':<10}{'力矩RMS':>8}{'力矩峰':>8}{'上限':>6}{'占用%':>6}"
      f"{'角RMS':>9}{'角max':>9}{'max时刻s':>9}")
for i in range(12):
    print(f"{JNAME[i]:<10}{tau_rms[i]:>8.1f}{tau_absmax[i]:>8.1f}{LIM[i]:>6}"
          f"{100 * tau_absmax[i] / LIM[i]:>6.0f}{q_rms[i]:>9.4f}"
          f"{q_max[i]:>9.4f}{np.mean(q_when[:, i]):>9.1f}")

section("A1 逐 run 排名（关节角误差）")
for n, rms, mx, tm in sorted(per_run, key=lambda r: -r[2]):
    print(f"  {n:<20} 角RMS={rms:.4f}  角max={mx:.4f}  力矩max={tm:.0f}")

# 误差与接触切换的关系：误差峰值时刻 ±0.1s 内是否有接触切换
section("误差峰值 vs 接触切换（同步性检验）")
for n in RUNS[:3]:
    z = np.load(f"/tmp/fig6_cache/{n}.npz")
    t_tp, t_jv = z["targetPos"][:, 0], z["jointsPosVel"][:, 0]
    idx = np.clip(np.searchsorted(t_jv, t_tp), 1, len(t_jv) - 1)
    left = np.abs(t_tp - t_jv[idx - 1]) <= np.abs(t_tp - t_jv[idx])
    jv = z["jointsPosVel"][np.where(left, idx - 1, idx), 1:13]
    qe = z["targetPos"][:, 1:13] - jv
    tt = t_tp - t_tp[0]
    k = int(np.abs(qe).max(1).argmax())
    sc = z["simContactFlag"]
    sw = sc[1:, 0][np.abs(np.diff(sc[:, 1:].astype(int), axis=0)).sum(1) > 0]
    t_sw = sc[1:, 0][np.abs(np.diff(sc[:, 1:].astype(int), axis=0)).sum(1) > 0] - t_tp[0]
    dtc = np.abs(t_sw - tt[k]).min() if len(t_sw) else float("nan")
    print(f"  {n}: 误差峰 {tt[k]:.2f}s 距最近接触切换 {dtc * 1000:.0f} ms")

# 误差分布形态
section("关节角误差分布（全部 A1 样本）")
allerr = []
for n in RUNS:
    z = np.load(f"/tmp/fig6_cache/{n}.npz")
    t_tp, t_jv = z["targetPos"][:, 0], z["jointsPosVel"][:, 0]
    idx = np.clip(np.searchsorted(t_jv, t_tp), 1, len(t_jv) - 1)
    left = np.abs(t_tp - t_jv[idx - 1]) <= np.abs(t_tp - t_jv[idx])
    jv = z["jointsPosVel"][np.where(left, idx - 1, idx), 1:13]
    allerr.append(np.abs(z["targetPos"][:, 1:13] - jv).ravel())
allerr = np.concatenate(allerr)
for p in (50, 90, 99, 99.9):
    print(f"  |误差| p{p} = {np.percentile(allerr, p):.4f} rad "
          f"({np.degrees(np.percentile(allerr, p)):.2f}°)")
print(f"  >0.1 rad 的样本占比 {100 * (allerr > 0.1).mean():.3f}%")

# ---------------- A2 扰动 ----------------
section("A2 扰动样本质量（逐格）")
import re
for cell in ("lat_L1_stand", "lat_L2_stand", "lat_L3_stand", "fwd_L1_stand",
             "fwd_L2_stand", "fwd_L3_stand", "lat_L1_walk", "lat_L2_walk",
             "lat_L3_walk"):
    ok = bad_del = bad_spon = fallen = 0
    dps = []
    for p in sorted(glob.glob(f"experiments/sim_push_{cell}_*/analysis/push_summary.txt")):
        txt = open(p).read()
        if "注入疑似未生效" in txt:
            bad_del += 1
            continue
        if "注入前已倒地" in txt:
            bad_spon += 1
            continue
        ok += 1
        m = re.search(r"dphi_max=([0-9.]+)", txt)
        if m:
            dps.append(float(m.group(1)))
        if "FALLEN" in txt:
            fallen += 1
    print(f"  {cell:<15} 有效={ok} 未送达={bad_del} 自发故障={bad_spon} "
          f"推力致倒={fallen}  dphi_max 中位={np.median(dps) if dps else float('nan'):.3f}")

section("结论标记")
print("""  [看数据说话]""")
