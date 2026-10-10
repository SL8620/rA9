#!/usr/bin/env python3
"""质心高度 z_c 正解计算（名义 MJCF 模型 + 基座状态 + 关节角）。

口径：z_c = 机器人整体质心的世界系高度，由 MuJoCo 前向运动学（subtree_com）
按名义模型计算——与 C4 的质心跟踪量 r_c 对齐（6.4.1 口径修正，2026-10-10）。
输入 z_c ≈ f(base_z, roll, pitch, q_j)；yaw 不影响 z 分量（旋转矩阵第三行
与偏航无关），故 roll/pitch 足够。
"""
import numpy as np

NOMINAL = ("/home/mi/disk/SS/thesis/rA9/src/humanoid_control/"
           "humanoid_legged_description/mjcf/humanoid_legged_control_.xml")

_model = None
_data = None
_base_id = None


def _init():
    global _model, _data, _base_id
    if _model is None:
        import mujoco as mj
        _model = mj.MjModel.from_xml_path(NOMINAL)
        _data = mj.MjData(_model)
        _base_id = mj.mj_name2id(_model, mj.mjtObj.mjOBJ_BODY, "base_link")


def com_height(base_z, roll, pitch, joints):
    """逐样本质心高度 [m]。joints: (N,12)，roll/pitch/base_z: (N,)"""
    _init()
    import mujoco as mj
    n = len(base_z)
    out = np.empty(n)
    q = np.zeros(_model.nq)
    for i in range(n):
        cr, sr = np.cos(roll[i] / 2), np.sin(roll[i] / 2)
        cp, sp = np.cos(pitch[i] / 2), np.sin(pitch[i] / 2)
        # yaw=0: quat = Ry(pitch)⊗Rx(roll) → (w,x,y,z) = (cp·cr, cp·sr, sp·cr, −sp·sr)
        q[3:7] = [cp * cr, cp * sr, sp * cr, -sp * sr]
        q[2] = base_z[i]
        q[7:19] = joints[i]
        _data.qpos[:] = q
        mj.mj_forward(_model, _data)
        out[i] = _data.subtree_com[_base_id][2]
    return out
