#!/usr/bin/env python3
"""力矩链路 bag 级校验：/targetTorque → /controlInput → /realTorque。

用途两件：
  1. QA 新录 bag：确认新语义话题在流、钳制符合力矩界、PD 修正量合理。
  2. 判定旧 bag 是否可**确定性重建**新语义（无需重录）：
       /realTorque_new = clip(/controlInput, ±LIM) = clip(/realTorque_old, ±LIM)
     若该式在新 bag 上残差为 0，则旧 bag 亦可由同一算式回填。

口径（2026-10-12 /realTorque 语义变更后）：
  * 旧 bag（无 /controlInput）：/realTorque = data.ctrl = 控制输入（前馈+PD）
  * 新 bag（有 /controlInput）：/realTorque = qfrc_actuator[-12:] = 执行器实际出力
    /controlInput = data.ctrl（承接旧语义）
  * 两代 bag 的 torque_tracking 指标**不可跨代比较**（analyze_walk.py 会标注）

用法: verify_torque_chain.py <bag_dir> [--lim-from task_.info] [--quiet]
退出码: 0=通过/可重建, 1=不通过, 2=无法判定
"""
import argparse
import os
import re
import sys

import numpy as np

RA9 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if "AMENT_PREFIX_PATH" not in os.environ:
    _c = ('source /opt/ros/jazzy/setup.bash 2>/dev/null; '
          'source "%s/install/setup.bash" 2>/dev/null; '
          'exec python3 "$0" "$@"' % RA9)
    os.execvp("bash", ["bash", "-c", _c, os.path.abspath(__file__)] + sys.argv[1:])

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ModuleNotFoundError:
    import glob
    for _p in glob.glob("/opt/ros/*/lib/python3*/site-packages"):
        sys.path.insert(0, _p)
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

JN = ["L_hipR", "L_hipY", "L_hipP", "L_knee", "L_ankP", "L_ankR",
      "R_hipR", "R_hipY", "R_hipP", "R_knee", "R_ankP", "R_ankR"]
WANT = ["/targetTorque", "/controlInput", "/realTorque"]


def read_limits(path):
    """从 task_.info 的 torqueLimitsTask 块读力矩界 → 12 维。"""
    txt = open(path, encoding="utf-8").read()
    blk = re.search(r"torqueLimitsTask\s*\{(.*?)\}", txt, re.S).group(1)
    d = {int(i): float(v) for i, v in
         re.findall(r"\(\s*(\d)\s*,\s*0\s*\)\s+([\d.]+)", blk)}
    return np.array([d[i % 6] for i in range(12)], float)


def load(bag):
    t = {k: [] for k in WANT}
    v = {k: [] for k in WANT}
    for f in sorted([os.path.join(bag, x) for x in os.listdir(bag)
                     if x.endswith(".mcap")]):
        r = SequentialReader()
        r.open(StorageOptions(uri=f, storage_id="mcap"), ConverterOptions("", ""))
        types = {x.name: get_message(x.type) for x in r.get_all_topics_and_types()
                 if x.name in WANT}
        while r.has_next():
            tp, data, ts = r.read_next()
            if tp not in types:
                continue
            m = deserialize_message(data, types[tp])
            t[tp].append(ts * 1e-9)
            v[tp].append([float(x) for x in m.data])
        r.close()
    return {k: (np.asarray(t[k], float),
                np.asarray(v[k], float).reshape(len(v[k]), -1) if v[k]
                else np.zeros((0, 1))) for k in WANT}


def nn(st, sv, dt):
    if len(st) == 0:
        return np.zeros((0, sv.shape[1]))
    i = np.clip(np.searchsorted(st, dt), 1, len(st) - 1)
    l = np.abs(dt - st[i - 1]) <= np.abs(dt - st[i])
    return sv[np.where(l, i - 1, i)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--lim", default=os.path.join(
        RA9, "src/humanoid_control/humanoid_interface/config/mpc/task_.info"))
    ap.add_argument("--tol", type=float, default=1.0, help="恒等式容差 [Nm]")
    a = ap.parse_args()

    LIM = read_limits(a.lim)
    print(f"[bag] {a.bag}")
    print(f"[lim] 力矩界 {LIM.astype(int).tolist()}  (源 {os.path.relpath(a.lim, RA9)})")

    D = load(a.bag)
    has_new = len(D["/controlInput"][0]) > 0
    n_real = len(D["/realTorque"][0])
    print(f"\n[话题] /targetTorque {len(D['/targetTorque'][0])} 条  "
          f"/controlInput {len(D['/controlInput'][0])} 条  /realTorque {n_real} 条")

    if n_real == 0:
        print("\n判定: ⚠️ 无法判定（/realTorque 为空）")
        return 2

    print(f"[代际] {'新语义（有 /controlInput）' if has_new else '旧语义（无 /controlInput）'}")

    rt_t, rt = D["/realTorque"]
    src_t, src = (D["/controlInput"] if has_new else D["/targetTorque"])
    src_name = "/controlInput" if has_new else "/targetTorque（旧bag等价于controlInput）"

    # ---- 恒等式 1：/realTorque == clip(源, ±LIM) ----
    # 旧 bag 源用 /targetTorque 不行（旧 /realTorque 才是 controlInput），
    # 故旧 bag 直接对自身 clip。
    if has_new:
        ci = nn(*D["/controlInput"], rt_t)
        pred = np.clip(ci, -LIM, LIM)
    else:
        pred = np.clip(rt, -LIM, LIM)     # 旧 bag：重建 = clip(自身)
    err = np.abs(rt - pred).max() if has_new else 0.0
    print(f"\n[恒等式1] /realTorque == clip({src_name}, ±LIM)")
    if has_new:
        print(f"   残差 max = {err:.4f} Nm   "
              f"{'✓ 成立 → 旧 bag 可由同一算式回填' if err < a.tol else '✗ 不成立 → 回填不安全'}")
    else:
        print("   旧 bag 无 /controlInput，无法直接验证；恒等式已在新 bag + 离线 "
              "MuJoCo 复算确认（见 NOTE.md §12）")

    # ---- 恒等式 2：/controlInput == /targetTorque + PD（旧 bag 不可分）----
    if has_new:
        tq = nn(*D["/targetTorque"], rt_t)
        d = np.abs(ci - tq)
        print(f"[恒等式2] /controlInput − /targetTorque = PD 修正量")
        print(f"   RMS {np.sqrt((d**2).mean()):.3f}  max {d.max():.1f} Nm   "
              f"（非零即 PD 在工作，符合预期）")

    # ---- 钳制统计 ----
    clipped = np.abs(rt) > (LIM - 1e-6)
    print(f"\n[钳制统计]（/realTorque 顶到上限的样本）")
    hit = []
    for i in range(12):
        n = int(clipped[:, i].sum())
        if n:
            hit.append(f"{JN[i]}({n},峰{np.abs(rt[:, i]).max():.0f}/{LIM[i]:.0f})")
    print("   " + ("；".join(hit) if hit else "无（全程未触上限）"))
    if has_new:
        over = np.abs(ci) - LIM
        over[over < 0] = 0
        frac = 100 * over.sum() / max(np.abs(ci).sum(), 1e-9)
        print(f"   被削掉的力矩占 /controlInput 积分: {frac:.3f}%")
    print(f"   /realTorque 采样率: {n_real / max(rt_t[-1] - rt_t[0], 1e-9):.1f} Hz"
          f"  （低采样会漏峰值，报峰值请用 /targetTorque）")

    # ---- 判定 ----
    if has_new:
        ok = err < a.tol
        print(f"\n判定: {'✅ 新语义正确' if ok else '❌ 新语义与钳制不符'}")
        return 0 if ok else 1
    print("\n判定: ⚠️ 旧语义 bag —— 指标可算但语义是控制输入；"
          "可回填转换（无需重录），见 NOTE.md §12")
    return 2


if __name__ == "__main__":
    sys.exit(main())
