#!/usr/bin/env python3
"""靶向验证 /realTorque 语义变更 + 力矩界双层钳制。

原理：data.ctrl = τ_ff + Kp·Δq + Kd·Δv（humanoid_sim.py）。
令 Kp=Kd=0 → ctrl ≡ targetTorque，于是
  /controlInput  = data.ctrl          = 我们注入的原始值
  /realTorque    = qfrc_actuator[-12:] = min(注入值, 生效上限)
两者之差就应当精确等于钳制量。不需要控制器、不需要步态。

预期（200/200/320/320/200/200）：
  A 全 50 Nm        → 两话题都 ≈ 50      （未饱和，应相等）
  B 膝 400 (idx3,9) → controlInput≈400, realTorque≈320
  C 踝俯仰 250(idx4,10) → controlInput≈250, realTorque≈200
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

JN = ["L_hipR", "L_hipY", "L_hipP", "L_knee", "L_ankP", "L_ankR",
      "R_hipR", "R_hipY", "R_hipP", "R_knee", "R_ankP", "R_ankR"]
LIM = np.array([200, 200, 320, 320, 200, 200] * 2, float)

# (名称, 注入向量)。期望值 = clip(注入, ±LIM)：未饱和关节应等于注入值本身，
# 超界关节应被夹到对应轴的上限。早期版本对全部关节套上限，是测试脚本的 bug。
PHASES = [
    ("A 未饱和 全50Nm",   np.full(12, 50.0)),
    ("B 膝400超界",       np.array([100, 50, 150, 400, 80, 40] * 2, float)),
    ("C 踝俯仰250超界",   np.array([100, 50, 150, 150, 250, 40] * 2, float)),
    ("D 负向膝-400超界",  np.array([100, 50, 150, -400, 80, 40] * 2, float)),
    ("E 全轴正向打满+500", np.full(12, 500.0)),
    ("F 全轴负向打满-500", np.full(12, -500.0)),
]
PHASE_DUR = 1.2          # 每段采集时长 [s]
SETTLE = 0.4             # 段首过渡丢弃 [s]


class Verify(Node):
    def __init__(self):
        super().__init__("verify_torque")
        self.pub_tau = self.create_publisher(Float32MultiArray, "/targetTorque", 2)
        self.pub_kp = self.create_publisher(Float32MultiArray, "/targetKp", 2)
        self.pub_kd = self.create_publisher(Float32MultiArray, "/targetKd", 2)
        self.pub_pause = self.create_publisher(Bool, "/pauseCmd", 2)
        self.real, self.ctrl_in = [], []      # (wall_t, vec)
        self.n_real = self.n_ctrl = 0
        self.create_subscription(Float32MultiArray, "/realTorque", self.on_real, 50)
        self.create_subscription(Float32MultiArray, "/controlInput", self.on_ctrl, 50)
        self.sim_alive = False
        # /sim_time 只在非暂停时发布；/jointsPosVel 暂停期也在发（humanoid_sim
        # 的 pause 分支），故用它做存活探针
        self.create_subscription(Float32MultiArray, "/jointsPosVel", self.on_sim, 10)

    def on_sim(self, m):
        self.sim_alive = True

    def on_real(self, m):
        self.n_real += 1
        self.real.append((time.time(), np.asarray(list(m.data), float)))

    def on_ctrl(self, m):
        self.n_ctrl += 1
        self.ctrl_in.append((time.time(), np.asarray(list(m.data), float)))

    def send(self, tau):
        m = Float32MultiArray()
        m.data = [float(x) for x in tau]
        self.pub_tau.publish(m)
        z = Float32MultiArray()
        z.data = [0.0] * 12
        self.pub_kp.publish(z)      # PD 归零 → ctrl ≡ targetTorque
        self.pub_kd.publish(z)


def main():
    rclpy.init()
    n = Verify()
    t0 = time.time()

    print("[1] 等 sim 起来（/jointsPosVel，暂停期也发）...", flush=True)
    end = time.time() + 20
    while time.time() < end and not n.sim_alive:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.sim_alive:
        print("  ✗ sim 未起来（20s 无 /jointsPosVel）"); rclpy.shutdown(); return 2
    print("  ✓ sim 在跑", flush=True)

    print("[2] 解除暂停（/pauseCmd false，重发对抗 DDS 发现）...", flush=True)
    end = time.time() + 4
    while time.time() < end:
        m = Bool(); m.data = False
        n.pub_pause.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f"  /realTorque {n.n_real} 条  /controlInput {n.n_ctrl} 条", flush=True)

    print("[3] 逐段注力矩并采集（Kp=Kd=0）...", flush=True)
    results = []
    for name, tau in PHASES:
        expect = np.clip(tau, -LIM, LIM)      # 未饱和=注入值本身；超界=夹到该轴上限
        n.real.clear(); n.ctrl_in.clear()
        t_start = time.time()
        while time.time() - t_start < PHASE_DUR:
            n.send(tau)
            rclpy.spin_once(n, timeout_sec=0.01)
        reals = np.array([v for t, v in n.real if t - t_start > SETTLE])
        ctrls = np.array([v for t, v in n.ctrl_in if t - t_start > SETTLE])
        results.append((name, tau, expect, reals, ctrls))

    # ---- 报告 ----
    ok_all = True
    print("\n" + "=" * 84)
    print("话题存在性")
    print("=" * 84)
    ok_topic = n.n_ctrl > 0
    print(f"  /controlInput : {'✓ 有 ' + str(n.n_ctrl) + ' 条（新语义 bag）' if ok_topic else '✗ 无 → 仍是旧语义'}")
    print(f"  /realTorque   : {'✓ 有 ' + str(n.n_real) + ' 条' if n.n_real else '✗ 无'}")
    ok_all &= ok_topic and n.n_real > 0

    for name, tau, expect, reals, ctrls in results:
        print("\n" + "=" * 84)
        print(f"{name}   注入: {tau.astype(int)}")
        print("=" * 84)
        if len(reals) == 0 or len(ctrls) == 0:
            print(f"  ✗ 样本不足 real={len(reals)} ctrl={len(ctrls)}")
            ok_all = False
            continue
        rmed = np.median(reals, axis=0)
        cmed = np.median(ctrls, axis=0)
        print(f"  {'关节':<9}{'注入':>8}{'controlInput':>14}{'realTorque':>12}{'期望real':>10}{'误差':>9}")
        for i in range(12):
            e = abs(rmed[i] - expect[i])
            good = e < 3.0
            ok_all &= good
            print(f"  {JN[i]:<9}{tau[i]:>8.0f}{cmed[i]:>14.1f}{rmed[i]:>12.1f}"
                  f"{expect[i]:>10.0f}{e:>9.2f}  {'✓' if good else '✗'}")
        # 关键断言：controlInput 应保持注入值（未被 MuJoCo 改写）
        d_ctrl = np.abs(cmed - tau).max()
        d_real = np.abs(rmed - expect).max()
        print(f"  —— controlInput vs 注入 maxΔ = {d_ctrl:.2f} Nm"
              f"{'  ✓ 未被改写' if d_ctrl < 3 else '  ✗ 被改写！'}")
        print(f"  —— realTorque vs 期望   maxΔ = {d_real:.2f} Nm"
              f"{'  ✓ 钳制正确' if d_real < 3 else '  ✗ 钳制不符'}")
        ok_all &= d_ctrl < 3

    print("\n" + "=" * 84)
    print(f"总判定: {'✅ 全部通过' if ok_all else '❌ 有失败项'}")
    print("=" * 84)
    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
