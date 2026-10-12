#!/usr/bin/env python3
"""sim2real gap 灵敏度分析：逐项开关非理想因素，量化各 gap 对指标的贡献。

用途（均为合成数据，非实机实测）：
  1. 灵敏度分析 —— 每个 gap 对验证章指标的影响有多大（论文 6.6 局限性）
  2. A3 实机补录占位 / 流水线验证
  3. 论文图表占位

流程：对同一源 run，逐个 gap 单独开启（其余关闭）生成合成变体 →
analyze_walk.py 复算指标 → 与"全 gap 关闭"基线比较 → 输出 Δ 表。

⚠️ 开环近似：产物 = 理想执行器轨迹 + 非理想信号特性，**不是**真实执行器下
   会产生的轨迹。延迟只移位信号、不改变运动（闭环反馈效应缺失）。详见
   deliverables/C6/analysis/NOTE.md §13。

用法:
  gap_sensitivity.py <src_run_dir> [--out DIR] [--seed 42] [--cmd 0.3]
                     [--gaps 名1,名2]   # 只测这些 gap（缺省全测）
产出:
  deliverables/C6/analysis/gap_sensitivity_<run>.md   表格（可直接进论文）
  <out>/synth_<gap>/         各变体（含 SYNTHETIC.txt 标记）—— 大体积，
                             缺省放 experiments/（gitignore），不进 deliverables
  <out>/<gap>/analysis/      各变体的 metrics.csv

注意：deliverables/C6/README.md 规定该目录"只留可复算的小体积结果"，
故**只有 .md 表格进 deliverables**，bag/变体一律落 experiments/。
"""
import argparse
import csv
import os
import shutil
import subprocess
import sys

RA9 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RA9, "scripts"))

# 只取 gap 注册表，不触发 make_synth_real 的 ROS 自举
_src = open(os.path.join(RA9, "scripts", "make_synth_real.py"), encoding="utf-8").read()
_lo = _src.index("GAPS = {")
_hi = _src.index("ALL_GAPS = list(GAPS)")
_ns = {}
exec(compile(_src[_lo:_hi] + "\nALL_GAPS = list(GAPS)", "gaps", "exec"), _ns)
GAPS, ALL_GAPS = _ns["GAPS"], _ns["ALL_GAPS"]

# 从 metrics.csv 抓这些指标做灵敏度对比
METRICS = [
    ("base_tracking", "vx_rmse", "vx RMSE [m/s]"),
    ("base_tracking", "height_std", "基座高 std [m]"),
    ("contact_timing", "L.agree_pct", "接触 agree% [L]"),
    ("contact_timing", "L.switch_err_ms", "switch_err [ms]"),
    ("torque_tracking", "rms_total", "力矩 RMS [Nm]"),
    ("torque_tracking", "max_abs", "力矩 max [Nm]"),
    ("torque_tracking", "pd_correction_rms", "PD修正 RMS [Nm]"),
    ("torque_tracking", "clip_loss_rms", "钳制损失 RMS [Nm]"),
    ("joint_tracking", "rms_total_excl", "关节角 RMS [rad]"),
    ("joint_tracking", "max_abs_excl", "关节角 max [rad]"),
    ("solve_time", "mpc.mean", "MPC mean [ms]"),
    ("solve_time", "wbc.mean", "WBC mean [ms]"),
]


def read_metrics(run_dir):
    p = os.path.join(run_dir, "analysis", "metrics.csv")
    if not os.path.exists(p):
        return {}
    d = {}
    for row in csv.reader(open(p)):
        if len(row) >= 3:
            d[(row[0], row[1])] = row[2]
    return d


def get(m, key, default=float("nan")):
    try:
        return float(m.get(key, ""))
    except (TypeError, ValueError):
        return default


def run(cmd, **kw):
    r = subprocess.run(cmd, cwd=RA9, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        print(f"  ✗ 命令失败: {' '.join(cmd)}")
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])
    return r.returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--out", default=None)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--cmd", type=float, default=0.3, help="cmd_vx，传给 analyze_walk")
    ap.add_argument("--gaps", default="", help="只测这些 gap（逗号分隔）")
    a = ap.parse_args()

    src = os.path.abspath(a.src)
    name = os.path.basename(src.rstrip("/"))
    out = a.out or os.path.join(RA9, "experiments", "gap_sens_" + name)
    md = os.path.join(RA9, "deliverables/C6/analysis", f"gap_sensitivity_{name}.md")
    if os.path.exists(out):
        print(f"ERROR: {out} 已存在，拒绝覆盖")
        sys.exit(1)
    os.makedirs(out)

    gaps = ALL_GAPS if not a.gaps else [s.strip() for s in a.gaps.split(",") if s.strip()]
    bad = set(gaps) - set(ALL_GAPS)
    if bad:
        print(f"ERROR: 未知 gap {sorted(bad)}；可选 {ALL_GAPS}")
        sys.exit(2)

    ana = os.path.join(RA9, "scripts", "analyze_walk.py")
    syn = os.path.join(RA9, "scripts", "make_synth_real.py")

    # ---- 基线：全 gap 关闭（= 原始 run，不加任何非理想）----
    print(f"[基线] 原始 run（无 gap）: {name}")
    base_dir = os.path.join(out, "_baseline")
    os.makedirs(base_dir)
    if not run([sys.executable, ana, os.path.join(src, "bag"),
                "--cmd", str(a.cmd), "--out", os.path.join(base_dir, "analysis")]):
        print("ERROR: 基线分析失败")
        sys.exit(1)
    base = read_metrics(base_dir)

    # ---- 逐 gap ----
    rows = {}
    for g in gaps + ["__ALL__"]:
        label = "全 gap 开启" if g == "__ALL__" else GAPS[g]
        gaps_arg = ",".join(ALL_GAPS) if g == "__ALL__" else g
        variant = os.path.join(out, f"synth_{g}")
        print(f"\n[{g}] {label}")
        if not run([sys.executable, syn, src, variant,
                    "--seed", str(a.seed), "--gaps", gaps_arg]):
            print(f"  ✗ 合成失败，跳过")
            continue
        if not run([sys.executable, ana, os.path.join(variant, "bag"),
                    "--cmd", str(a.cmd), "--out", os.path.join(variant, "analysis")]):
            print(f"  ✗ 分析失败，跳过")
            continue
        rows[g] = read_metrics(variant)

    # ---- 出表（.md 进 deliverables，变体落 experiments/）----
    with open(md, "w", encoding="utf-8") as f:
        f.write(f"# sim2real gap 灵敏度分析\n\n")
        f.write(f"源 run：`{name}`　seed={a.seed}　cmd_vx={a.cmd} m/s\n\n")
        f.write(f"各变体（含 bag + `SYNTHETIC.txt`）位于 `{os.path.relpath(out, RA9)}/`；"
                f"按 `deliverables/C6/README.md`「只留可复算的小体积结果」，"
                f"本目录只放本表。\n\n")
        f.write(f"> ⚠️ **合成数据（非实机实测）**，用途：灵敏度分析 / 流水线验证 / 图表占位。\n"
                f"> **开环近似**：产物 = 理想执行器轨迹 + 非理想信号特性，不是真实\n"
                f"> 执行器下会产生的轨迹；延迟只移位信号、不改变运动（闭环反馈效应缺失）。\n"
                f"> 每个变体目录含 `SYNTHETIC.txt` 与 `params.txt: synthetic=true`。\n\n")
        f.write("## 各 gap 对指标的影响\n\n")
        f.write("数值 = 该 gap **单独开启**时的指标；Δ = 相对「无 gap」基线的变化。\n")
        f.write("`全开` = 所有 gap 同时开启（非各 Δ 之和，因各项非线性叠加）。\n\n")
        hdr = f"| 指标 | 无 gap 基线 |"
        for g in gaps:
            hdr += f" {g} | Δ |"
        hdr += " 全开 | Δ |"
        f.write(hdr + "\n")
        f.write("|" + "---|" * (2 + 2 * len(gaps) + 2) + "\n")
        for cat, item, disp in METRICS:
            b = get(base, (cat, item))
            line = f"| {disp} | {b:.4g} |"
            for g in gaps:
                v = get(rows.get(g, {}), (cat, item))
                d = v - b
                line += f" {v:.4g} | {d:+.3g} |"
            va = get(rows.get("__ALL__", {}), (cat, item))
            line += f" {va:.4g} | {va - b:+.3g} |"
            f.write(line + "\n")

        f.write("\n## gap 说明\n\n")
        f.write("| gap | 含义 |\n|---|---|\n")
        for g in gaps:
            f.write(f"| `{g}` | {GAPS[g]} |\n")
        f.write(f"| `__ALL__` | 全部 gap 同时开启 |\n")
        f.write(f"\n## 复算\n\n```bash\n")
        for g in gaps + ["__ALL__"]:
            ga = ",".join(ALL_GAPS) if g == "__ALL__" else g
            f.write(f"python3 scripts/make_synth_real.py {src} <out>/synth_{g} "
                    f"--seed {a.seed} --gaps {ga}\n")
        f.write("```\n")

    print(f"\n[产出] {md}")
    print(f"[产出] 各变体在 {out}/synth_<gap>/（均带 SYNTHETIC.txt）")


if __name__ == "__main__":
    main()
