#!/usr/bin/env python3
"""误差尖峰根因：目标跳变 vs 真实跟踪失败。"""
import numpy as np

for run, joint, name in (("sim_base_v00_03", 3, "L膝"),
                         ("sim_base_v00_03", 2, "L髋俯仰"),
                         ("sim_base_v01_03", 3, "L膝")):
    z = np.load(f"/tmp/fig6_cache/{run}.npz")
    t_tp, t_jv = z["targetPos"][:, 0], z["jointsPosVel"][:, 0]
    idx = np.clip(np.searchsorted(t_jv, t_tp), 1, len(t_jv) - 1)
    left = np.abs(t_tp - t_jv[idx - 1]) <= np.abs(t_tp - t_jv[idx])
    jv = z["jointsPosVel"][np.where(left, idx - 1, idx), 1:13]
    tp = z["targetPos"][:, 1:13]
    qe = tp - jv
    tt = t_tp - t_tp[0]
    k = int(np.abs(qe[:, joint]).argmax())
    print(f"\n== {run} {name}: 误差峰 t={tt[k]:.3f}s  err={qe[k, joint]:+.3f} rad")
    print(f"   {'t':>8}{'target':>10}{'actual':>10}{'err':>9}")
    for i in range(max(0, k - 5), min(len(tt), k + 6)):
        print(f"   {tt[i]:>8.3f}{tp[i, joint]:>10.3f}{jv[i, joint]:>10.3f}"
              f"{qe[i, joint]:>9.3f}")
    d_t = np.abs(np.diff(tp[:, joint]))
    d_a = np.abs(np.diff(jv[:, joint]))
    w = (tt[1:] > tt[k] - 0.1) & (tt[1:] < tt[k] + 0.1)
    if w.any():
        print(f"   ±0.1s: target 最大步进 {d_t[w].max():.4f}、实际 {d_a[w].max():.4f} rad/采样")
    # 同 run 内误差>0.3rad 的总时长
    big = np.abs(qe[:, joint]) > 0.3
    print(f"   该关节 |err|>0.3 rad 总时长 {np.sum(big) * 0.002:.3f} s（占 run "
          f"{100 * big.mean():.2f}%）")
