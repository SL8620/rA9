#!/usr/bin/env python3
"""动力学参数扰动生成器：质量/转动惯量/质心位置误差（sim-to-real 模型失配）。

读名义 MJCF，对每个 body 的 <inertial> 独立扰动：
  mass × (1+δm)        δm ~ U(-5%, +5%)     装配/线束/电池差异
  diaginertia × (1+δI) δI ~ U(-10%, +10%)   惯量张量辨识/建模误差
  pos + Δp             Δp ~ U(-5mm, +5mm)   质心位置偏差
控制器仍用名义 URDF（不扰动）——植物 vs 控制器模型失配即 sim-to-real gap。

用法: gen_perturbed_mjcf.py <out_xml> [--seed 42] [--dm 0.05] [--di 0.10] [--dp 0.005]
输出: <out_xml> + <out_xml>.factors.json（逐 body 扰动系数，可复算）
"""
import argparse
import json
import re

import numpy as np

NOMINAL = ("/home/mi/disk/SS/thesis/rA9/src/humanoid_control/"
           "humanoid_legged_description/mjcf/humanoid_legged_control_.xml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--dm", type=float, default=0.05)
    ap.add_argument("--di", type=float, default=0.10)
    ap.add_argument("--dp", type=float, default=0.005)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    src = open(NOMINAL).read()
    factors = {"seed": a.seed, "dm": a.dm, "di": a.di, "dp": a.dp, "bodies": {}}

    # 逐 body 记录与替换（按出现顺序，body 名从最近的 <body name=...> 提取）
    body_ctx = "unknown"
    out = []
    last = 0
    for m in re.finditer(r'<body name="([^"]+)"', src):
        body_ctx = m.group(1)
    pat = re.compile(r'(<inertial pos="([^"]+)"[^>]*mass="([^"]+)"'
                     r'\s+diaginertia="([^"]+)"/>)')
    for m in pat.finditer(src):
        # 该 inertial 之前最近的 body 名
        pre = src[:m.start()]
        bm = list(re.finditer(r'<body name="([^"]+)"', pre))
        name = bm[-1].group(1) if bm else "unknown"
        fm = float(m.group(3))
        fi = [float(x) for x in m.group(4).split()]
        fp = [float(x) for x in m.group(2).split()]
        k_m = 1.0 + rng.uniform(-a.dm, a.dm)
        k_i = 1.0 + rng.uniform(-a.di, a.di)
        dp = rng.uniform(-a.dp, a.dp, 3)
        new_m = fm * k_m
        new_i = [x * k_i for x in fi]
        new_p = [fp[i] + dp[i] for i in range(3)]
        factors["bodies"][name] = dict(
            mass=round(fm, 6), mass_new=round(new_m, 6), k_m=round(k_m, 5),
            k_i=round(k_i, 5), dp=[round(x, 6) for x in dp])
        qm = re.search(r'quat="([^"]+)"', m.group(1))
        quat_attr = f'quat="{qm.group(1)}" ' if qm else ""
        repl = (f'<inertial pos="{new_p[0]:.6g} {new_p[1]:.6g} {new_p[2]:.6g}" '
                f'{quat_attr}mass="{new_m:.6g}" '
                f'diaginertia="{new_i[0]:.6g} {new_i[1]:.6g} {new_i[2]:.6g}"/>')
        out.append(src[last:m.start()])
        out.append(repl)
        last = m.end()
    out.append(src[last:])
    open(a.out, "w").write("".join(out))
    open(a.out + ".factors.json", "w").write(json.dumps(factors, indent=1))
    tot = sum(v["mass_new"] for v in factors["bodies"].values())
    nom = sum(v["mass"] for v in factors["bodies"].values())
    print(f"[perturb] {a.out}: {len(factors['bodies'])} bodies, "
          f"总质量 {nom:.2f}->{tot:.2f} kg ({100*(tot/nom-1):+.1f}%), seed={a.seed}")


if __name__ == "__main__":
    main()
