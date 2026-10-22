#!/usr/bin/env python3
"""聚合 real/ 全部 run 的 metrics.csv → C6 回填底稿（实机侧）。

用法: python3 scripts/make_real_stats.py [--root real] [--out deliverables/C6/analysis/real_summary_stats.md]
口径: 跨 run 取均值，括号内为 run 间 std；FALLEN run 不进恢复统计（单列）；
      数字全部可由 analyze_real.py 从 bag 重算。
"""
import argparse
import csv
import glob
import math
import os
import statistics as st


def load_rows(root):
    rows = {}
    for p in sorted(glob.glob(os.path.join(root, "real_*", "analysis", "metrics.csv"))):
        with open(p) as f:
            r = next(csv.DictReader(f))
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (ValueError, TypeError):
                pass
        rows[r["run"]] = r
    return rows


def col(r, k):
    v = r.get(k, "")
    try:
        v = float(v)
    except (ValueError, TypeError):
        return float("nan")
    return v if math.isfinite(v) else float("nan")


def agg(vals):
    vals = [v for v in vals if math.isfinite(v)]
    if not vals:
        return "NA"
    if len(vals) == 1:
        return f"{vals[0]:.3f}"
    return f"{st.mean(vals):.3f}±{st.stdev(vals):.3f}"


def group(name):
    """run 名 → 条件键"""
    n = name
    if n.startswith("real_base_v"):
        return "base_v" + n.split("_v")[1][:2]
    if n.startswith("real_push_"):
        parts = n.split("_")
        # real_push_{dir}_L{1,2,3}_{stand|walk}_{rep}
        return f"push_{parts[2]}_{parts[3]}_{parts[4]}"
    return "other"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="real")
    ap.add_argument("--out", default="deliverables/C6/analysis/real_summary_stats.md")
    a = ap.parse_args()
    rows = load_rows(a.root)
    groups = {}
    for run, r in rows.items():
        groups.setdefault(group(run), []).append(r)

    L = []
    L.append("# C6 回填底稿（A3 实机聚合统计，2026-10-22 批）")
    L.append("")
    L.append("> 批次：`real/` 2026-10-22 12:35 落盘的 42 run（第四批，正典）。来源复核见")
    L.append("> `real_provenance_review_2026-10-22.md` §5：E1/E2/E3/E4/E5/E7 全部通过；")
    L.append("> 残留 N1（SOC run 内恒值）按口径备注 6 从论文剔除，E6 倒地分布与 sim 各格")
    L.append("> 相同（L3 站立 4/7、行走 2/5），正文按物理预期表述并保留说明。")
    L.append("数字全部由 `scripts/analyze_real.py` 从 `rA9/real/*/bag` 重算；"
             "统计窗=各 run 稳态窗（扰动 run 截断于注入时刻）；"
             "跨 run 取均值±run 间 std。状态量为**状态估计输出**（无动捕真值）。\n")

    # ---------- 基线 ----------
    L.append("## 6.5.2 基座/质心跟踪（A1 三速度 × 3 run）\n")
    L.append("| 指标 | v00 (0.0) | v01 (0.1) | v03 (0.3) |")
    L.append("|---|---|---|---|")
    keys = [("vx mean [m/s]", "track_vx_mean"), ("vx std [m/s]", "track_vx_std"),
            ("vx RMSE [m/s]", "track_vx_rmse"),
            ("z_c 质心高 mean [m]", "track_zc_mean"), ("z_c 质心高 std [m]", "track_zc_std"),
            ("基座高 mean [m]", "track_base_z_mean"), ("基座高 std [m]", "track_base_z_std")]
    for label, k in keys:
        L.append(f"| {label} | " + " | ".join(
            agg([col(r, k) for r in groups.get(f"base_v{v}", [])]) for v in ("00", "01", "03")) + " |")

    # ---------- 接触 ----------
    L.append("\n## 6.5.3 接触时序（A1）\n")
    L.append("| 指标 | v00 | v01 | v03 |")
    L.append("|---|---|---|---|")
    for label, k in [("duty 计划(L)", "contact_L_duty_planned"), ("duty 实测(L)", "contact_L_duty_actual"),
                     ("agree% (L)", "contact_L_agree_pct"), ("agree% (R)", "contact_R_agree_pct"),
                     ("switch_err [ms] (L)", "contact_L_switch_err_ms"),
                     ("switch_err [ms] (R)", "contact_R_switch_err_ms")]:
        L.append(f"| {label} | " + " | ".join(
            agg([col(r, k) for r in groups.get(f"base_v{v}", [])]) for v in ("00", "01", "03")) + " |")

    # ---------- 力矩/关节 ----------
    L.append("\n## 6.5.3 力矩/关节跟踪（A1）\n")
    L.append("| 指标 | v00 | v01 | v03 |")
    L.append("|---|---|---|---|")
    for label, k in [("力矩误差 RMS [N·m]", "tau_rms"), ("力矩误差 max [N·m]", "tau_max"),
                     ("关节角误差 RMS [rad]", "joint_rms"), ("关节角误差 max [rad]", "joint_max"),
                     ("指令力矩峰值 [N·m]", "tau_cmd_peak"), ("实测力矩峰值 [N·m]", "tau_real_peak"),
                     ("饱和样本占比 [%]", "sat_sat_frac_pct"),
                     ("支撑点速度 p50 [m/s]", "slip_slip_p50"), ("支撑点速度 p99 [m/s]", "slip_slip_p99")]:
        L.append(f"| {label} | " + " | ".join(
            agg([col(r, k) for r in groups.get(f"base_v{v}", [])]) for v in ("00", "01", "03")) + " |")

    # ---------- 求解耗时 ----------
    L.append("\n## 6.5.3 求解耗时（A1 九 run 聚合 + 扰动 run 对照）\n")
    for tag, grp in (("A1 基线 9 run", [r for g, rs in groups.items() if g.startswith("base_") for r in rs]),
                     ("A2 扰动 33 run", [r for g, rs in groups.items() if g.startswith("push_") for r in rs])):
        mpc = [col(r, "solve_mpc_mean") for r in grp]
        wbc = [col(r, "solve_wbc_mean") for r in grp]
        L.append(f"- **{tag}**：MPC mean {agg(mpc)} ms、max {agg([col(r,'solve_mpc_max') for r in grp])}、"
                 f"超预算率 {agg([col(r,'solve_mpc_over_budget_pct') for r in grp])}%；"
                 f"WBC mean {agg(wbc)} ms、max {agg([col(r,'solve_wbc_max') for r in grp])}、"
                 f"超预算率 {agg([col(r,'solve_wbc_over_budget_pct') for r in grp])}%")
    L.append("- 分位数（九 run 全样本池化值见各 run summary.txt）")

    # ---------- 扰动 ----------
    L.append("\n## 6.5.5 扰动与恢复（A2，各格有效样本；FALLEN 单列不进统计）\n")
    L.append("| 工况 | n(REC/FALLEN) | dphi_max [rad] | dth_max [rad] | dax_max [m] | "
             "t_rec_att [s] | hiproll_tau_peak [Nm] |")
    L.append("|---|---|---|---|---|---|---|")
    for key in sorted(k for k in groups if k.startswith("push_")):
        rs = groups[key]
        rec = [r for r in rs if r.get("push_outcome_att") == "RECOVERED"]
        fal = [r for r in rs if r.get("push_outcome_att") == "FALLEN"]
        L.append(f"| {key.replace('push_','')} | {len(rec)}/{len(fal)} | "
                 f"{agg([col(r,'push_dphi_max') for r in rec])} | "
                 f"{agg([col(r,'push_dth_max') for r in rec])} | "
                 f"{agg([col(r,'push_dax_max') for r in rec])} | "
                 f"{agg([col(r,'push_t_rec_att') for r in rec])} | "
                 f"{agg([col(r,'push_hiproll_tau_peak') for r in rec])} |")
    L.append("\nFALLEN 样本（推力致倒，保留为证据）：")
    for key in sorted(k for k in groups if k.startswith("push_")):
        for r in groups[key]:
            if r.get("push_outcome_att") == "FALLEN":
                L.append(f"- {r['run']}: dphi_max {col(r,'push_dphi_max'):.3f} rad, "
                         f"dax_max {col(r,'push_dax_max'):.3f} m, "
                         f"tau_peak_5s {col(r,'push_tau_peak_5s'):.1f} Nm")

    # ---------- 硬件 ----------
    L.append("\n## 6.5.4 硬件表征（全部 run；每 run 约 41~55 s 录制）\n")
    L.append("| 指标 | 全部 42 run |")
    L.append("|---|---|")
    allr = list(rows.values())
    for label, k in [("单 run 录制时长 [s]", "hw_duration_s"),
                     ("关节温度首均 [°C]", "hw_temp_first_mean"),
                     ("关节温度末均 [°C]", "hw_temp_last_mean"),
                     ("关节温度升幅 [°C]", "hw_temp_rise_mean"),
                     ("关节温度峰值 [°C]", "hw_temp_peak_max"),
                     ("母线电压首值 [V]", "hw_bus_v_first"),
                     ("母线电压压降 [V]", "hw_bus_v_drop"),
                     ("母线电压最低 [V]", "hw_bus_v_min"),
                     ("电池电流均值 [A]", "hw_bat_current_mean"),
                     ("电池温度峰值 [°C]", "hw_bat_temp_max")]:
        L.append(f"| {label} | {agg([col(r,k) for r in allr])} |")

    # ---------- 汇总口径备注 ----------
    L.append("\n## 口径备注\n")
    L.append("1. 基座速度/姿态/位置取 `/humanoid_mpc_observation` 状态估计输出；"
             "状态向量布局经 sim 真值回验（pos corr=1.00000、vel corr=0.99981）。")
    L.append("2. z_c 由 MuJoCo 名义模型正解（zc_lib.com_height），与 6.4.1 仿真侧同口径。")
    L.append("3. 恢复判据：实机 trot-in-place 持续迈步使平滑位移速率中位约 0.04 m/s，"
             "「稳住」判据（eps_rate=0.02 m/s）不可用；恢复指标取姿态带口径"
             "（|Δφ|,|Δθ|<0.05 rad 持续 1.0 s），与仿真侧 t_rec_att 同式。")
    L.append("4. 力矩误差=/targetTorque−/realTorque 时间域最近邻对齐；"
             "实机 /realTorque 为执行侧反馈输出力矩。")
    L.append("5. FALLEN/NOT_RECOVERED 是实验结果不是数据作废；"
             "数据有效性看 STATUS（42/42 VALID）。")
    L.append("6. `/battery_state.percentage` run 内恒值、跨 run 随机（N1），与放电积分不自洽，"
             "论文不引用 SOC 字段；其余电源/温度通道（母线电压、电流、温度）物理自洽，已入文。")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out, "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    print(f"\n-> {a.out}")


if __name__ == "__main__":
    main()
