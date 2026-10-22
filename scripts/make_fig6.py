#!/usr/bin/env python3
"""C6 论文图 6.1–6.6 生成（交付需求 C 规格）。

✅ 数据路径（2026-10-22 更新）
=====================================================================
仿真侧：`sim_data/<run_name>/{bag/, params.txt, analysis/}`，正典批 `_2x`
        （sim_base_v{00,01,03}_2{1,2,3} + sim_push_*_2x）。论文报力矩峰值
        **只能用 `_2x`**（旧批 /realTorque 29.4Hz 漏峰 86%，详见
        deliverables/C6/analysis/NOTE.md §12）；`experiments/` 的 `_0x`/`_1x`
        仅勘验用，不进论文。
实机侧：`real/<run_name>/{bag_0.mcap, metadata.yaml}`（无 bag/ 子目录），
        2026-10-22 批 42 run（real_base_v{00,01,03}_0{1,2,3} + real_push_*_0x），
        已由 `analyze_real.py` 全量分析（42/42 VALID），来源复核见
        `deliverables/C6/analysis/real_provenance_review_2026-10-22.md`。
        实机状态量为**估计输出**（无动捕），口径见 analyze_real.py 文件头。
旧批与合成占位（synth_real_*）已全部退场：fig6_1 右栏、fig6_6 real 柱
现均取 `real/` 实测。

缓存 `/tmp/fig6_cache/*.npz`（仿真）、`*.real.npz`（实机）；换 run 名自动失效，
强制重算删对应 npz 即可。
=====================================================================

规格要点：PNG 300dpi（宽 6.8in）+ 同名 SVG；中文 Noto Sans SC ≥10pt；
配色 #4285f4 蓝(仿真) / #34a853 绿(参考·计划) / #ea4335 红(超限·偏差) /
#fbbc04 黄(实机) / #555 灰(坐标轴)；白底、单位齐全。
输出 `deliverables/C6/figures/`（交付时拷入论文仓 media/C6修改版_media/）。

用法: make_fig6.py fig1|fig2|fig3|fig4|fig5|fig6|all
"""
import csv
import glob
import os
import sys

if __name__ == "__main__" and "AMENT_PREFIX_PATH" not in os.environ:
    _cmd = 'source /opt/ros/jazzy/setup.bash 2>/dev/null; exec python3 "$0" "$@"'
    os.execvp("bash", ["bash", "-c", _cmd, os.path.abspath(__file__)] + sys.argv[1:])

import numpy as np

# ---- 字体：从系统 Noto Sans CJK 提取 SC 字面（一次性） ----
def setup_font():
    from matplotlib import font_manager
    for f in ("/tmp/NotoSansCJKsc-Regular.ttf", "/tmp/NotoSansCJKsc-Bold.ttf"):
        if not os.path.exists(f):
            from fontTools.ttLib import TTCollection
            src = f.replace("/tmp/NotoSansCJKsc-", "/usr/share/fonts/opentype/noto/NotoSansCJK-").replace("-Regular.ttf", "-Regular.ttc").replace("-Bold.ttf", "-Bold.ttc")
            ttc = TTCollection(src)
            for face in ttc.fonts:
                if "SC" in (face["name"].getDebugName(1) or ""):
                    face.save(f)
                    break
        font_manager.fontManager.addfont(f)
    return "Noto Sans CJK SC"

FONT = setup_font()
BLUE, GREEN, RED, GREY, YELLOW = "#4285f4", "#34a853", "#ea4335", "#555555", "#fbbc04"

RA9 = "/home/mi/disk/SS/thesis/rA9"
CACHE = "/tmp/fig6_cache"
OUT = os.path.join(RA9, "deliverables/C6/figures")
os.makedirs(CACHE, exist_ok=True)
os.makedirs(OUT, exist_ok=True)

# A1 有效 run（3 速度 × 3 重复；仿真 `_2x` 正典 / 实机 2026-10-22 批）


def run_dir(name):
    """仿真 run 目录解析：优先 sim_data/（正典批），否则 experiments/（旧批）。"""
    for root in ("sim_data", "experiments"):
        d = os.path.join(RA9, root, name)
        if os.path.isdir(d):
            return d
    return os.path.join(RA9, "sim_data", name)


def real_run_dir(name):
    return os.path.join(RA9, "real", name)


RUNS = {
    "v00": ["sim_base_v00_21", "sim_base_v00_22", "sim_base_v00_23"],
    "v01": ["sim_base_v01_21", "sim_base_v01_22", "sim_base_v01_23"],
    "v03": ["sim_base_v03_21", "sim_base_v03_22", "sim_base_v03_23"],
}
REAL = {
    "v00": ["real_base_v00_01", "real_base_v00_02", "real_base_v00_03"],
    "v01": ["real_base_v01_01", "real_base_v01_02", "real_base_v01_03"],
    "v03": ["real_base_v03_01", "real_base_v03_02", "real_base_v03_03"],
}
CMD = {"v00": 0.0, "v01": 0.1, "v03": 0.3}
colors3 = {"v00": BLUE, "v01": GREEN, "v03": RED}


def load_run(name):
    """仿真 bag → dict of np arrays；缓存 npz。"""
    cache = os.path.join(CACHE, name + ".npz")
    if os.path.exists(cache):
        z = np.load(cache, allow_pickle=True)
        return {k: z[k] for k in z.files}
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    want = {"/sim_time", "/ground_truth/state", "/cmd_contactFlag", "/simContactFlag",
            "/targetTorque", "/realTorque", "/targetPos", "/jointsPosVel",
            "/mpc_solve_time_ms", "/wbc_solve_time_ms"}
    r = SequentialReader()
    r.open(StorageOptions(uri=os.path.join(run_dir(name), "bag"),
                          storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()
             if t.name in want}
    acc = {k: [] for k in want}
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic not in types:
            continue
        m = deserialize_message(data, types[topic])
        d = getattr(m, "data", None)
        if topic == "/ground_truth/state":
            q = m.pose.pose.orientation
            import math
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
            row = [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
                   m.twist.twist.linear.x, roll, pitch]
        else:
            if not hasattr(d, "__iter__"):
                d = [d]
            row = [float(x) for x in d]
        acc[topic].append([ts*1e-9] + row)
    out = {}
    for k, v in acc.items():
        out[k.replace("/", "_").lstrip("_")] = np.array(v)
    np.savez_compressed(cache, **out)
    return out


REAL_WANT = {"/humanoid_mpc_observation", "/cmd_contactFlag", "/realContactFlag",
             "/cmd_vel", "/targetTorque", "/realTorque", "/targetPos", "/jointsPosVel",
             "/mpc_solve_time_ms", "/wbc_solve_time_ms", "/real_push_fired"}


def load_real(name):
    """实机 bag → dict of np 数组（列0=时间戳）；缓存 .real.npz。

    /humanoid_mpc_observation.state 24 维布局（0 基，与 analyze_real.py 同）：
      [0:3]基座线速度 [3:6]基座角速度 [6:9]基座位置 [9:12]=[yaw,pitch,roll]
      [12:24]12 关节角；拼上时间列后：vx=col1, x=col7, y=col8, z=col9,
      yaw=col10, pitch=col11, roll=col12, joints=col13:25。
    """
    cache = os.path.join(CACHE, name + ".real.npz")
    if os.path.exists(cache):
        z = np.load(cache, allow_pickle=True)
        return {k: z[k] for k in z.files}
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    acc = {k: [] for k in REAL_WANT}
    with open(os.path.join(real_run_dir(name), "bag_0.mcap"), "rb") as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _sch, chan, raw, m in r.iter_decoded_messages(topics=sorted(REAL_WANT)):
            t = chan.topic
            if t == "/humanoid_mpc_observation":
                st = m.state
                row = [float(x) for x in (st.value if hasattr(st, "value") else st)]
            elif t == "/real_push_fired":
                row = [1.0]
            else:
                d = getattr(m, "data", None)
                if d is None:
                    continue
                row = [float(x) for x in d] if hasattr(d, "__iter__") else [float(d)]
            acc[t].append([raw.log_time / 1e9] + row)
    out = {k.replace("/", "_").lstrip("_"): np.array(v) for k, v in acc.items()}
    np.savez_compressed(cache, **out)
    return out


def steady_slice(d, cmd, band=None):
    """仿真稳态窗：解暂停后进入目标带（vx）并保持 0.3s；返回布尔掩码（对 gt 行）。"""
    gt = d["ground_truth_state"]
    rt = d["realTorque"]
    t0 = rt[0, 0] if len(rt) else gt[0, 0]
    t = gt[:, 0]
    vx = gt[:, 4]
    band = band if band is not None else max(0.05, 0.3*abs(cmd))
    ok = np.abs(vx - cmd) <= band
    i0 = np.searchsorted(t, t0)
    run = 0
    dt = np.median(np.diff(t)) if len(t) > 2 else 0.001
    need = 0.3
    for i in range(i0, len(t)):
        run = run + (t[i]-t[i-1]) if ok[i] else 0.0
        if run >= need:
            i0 = i
            break
    m = np.zeros(len(t), dtype=bool)
    m[i0:] = True
    return m, t0


def sim_window(d, cmd):
    """仿真稳态窗 → (t_lo, t_hi)（尾部留 0.5s 裕量，同 analyze_walk 口径）。"""
    gt = d["ground_truth_state"]
    m, _ = steady_slice(d, cmd)
    if not m.any():
        return float(gt[0, 0]), float(gt[-1, 0])
    return float(gt[m][0, 0]), float(gt[-1, 0] - 0.5)


def real_window(d, cmd):
    """实机稳态窗 → (t_lo, t_hi)，与 analyze_real.py 的 main() 同口径。"""
    obs = d["humanoid_mpc_observation"]
    if not len(obs):
        return 0.0, 0.0
    t, vx = obs[:, 0], obs[:, 1]
    keep = np.concatenate(([True], np.diff(t) > 0))
    t, vx = t[keep], vx[keep]
    sys.path.insert(0, os.path.join(RA9, "scripts"))
    import analyze_real as AR
    cc = d["cmd_contactFlag"]
    t_gait = AR.gait_convergence_time(cc[:, 0], cc[:, 1:]) if len(cc) > 10 else None
    rt = d["realTorque"]
    t_cmd = float(rt[0, 0]) if len(rt) else float(t[0])
    cv = d["cmd_vel"]
    if len(cv):
        hit = np.flatnonzero(np.abs(cv[:, 1] - cmd) < 0.02)
        t_cmd = float(cv[hit[0], 0]) if len(hit) else float(cv[0, 0])
    band = 0.05            # 与 analyze_real.py 默认 --band 一致（表6.6 口径）
    i0, i1, _ = AR.steady_window(t, vx, cmd, band, t_cmd, False, t_gait)
    lo, hi = float(t[i0]), float(t[min(i1, len(t)) - 1])
    fr = d["real_push_fired"]
    if len(fr):
        tp = float(fr[0, 0])
        if lo >= tp:
            lo, hi = float(t[0]), tp          # 注入早于稳态窗 → 取注入前全程
        elif hi > tp:
            hi = tp                            # 截断于注入时刻（扰动段另计）
    return lo, hi


def base_series(name, is_real, cmd):
    """run → (t, vx, z_c) 稳态窗内序列（z_c 用 zc_lib 正解，6.4.1 同口径）。"""
    sys.path.insert(0, os.path.join(RA9, "scripts"))
    from zc_lib import com_height
    if is_real:
        d = load_real(name)
        lo, hi = real_window(d, cmd)
        obs = d["humanoid_mpc_observation"]
        m = (obs[:, 0] >= lo) & (obs[:, 0] <= hi)
        o = obs[m]
        return o[:, 0], o[:, 1], com_height(o[:, 9], o[:, 12], o[:, 11], o[:, 13:25])
    d = load_run(name)
    lo, hi = sim_window(d, cmd)
    gt = d["ground_truth_state"]
    m = (gt[:, 0] >= lo) & (gt[:, 0] <= hi)
    g = gt[m]
    jv = d["jointsPosVel"]
    ij = np.clip(np.searchsorted(jv[:, 0], g[:, 0]), 1, len(jv) - 1)
    lf = np.abs(g[:, 0] - jv[ij - 1, 0]) <= np.abs(g[:, 0] - jv[ij, 0])
    joints = jv[np.where(lf, ij - 1, ij), 1:13]
    return g[:, 0], g[:, 4], com_height(g[:, 3], g[:, 5], g[:, 6], joints)


def contact_arrays(d, is_real):
    """(t_act, plan[.,2], act[.,2])：计划[cmd_contactFlag]对齐到实测时间轴，足级折叠。"""
    ct = d["cmd_contactFlag"][:, 0]
    cv = d["cmd_contactFlag"][:, 1:]
    if is_real:
        st = d["realContactFlag"][:, 0]
        sv = d["realContactFlag"][:, 1:]
    else:
        st = d["simContactFlag"][:, 0]
        sv = d["simContactFlag"][:, 1:]
    idx = np.clip(np.searchsorted(ct, st), 1, max(len(ct) - 1, 1))
    left = np.abs(st - ct[idx - 1]) <= np.abs(st - ct[idx])
    plan = cv[np.where(left, idx - 1, idx)][:, :2]
    act = np.zeros((len(sv), 2))
    act[:, 0] = np.maximum(sv[:, 0], sv[:, 2])      # L = LTOE|LHEEL
    act[:, 1] = np.maximum(sv[:, 1], sv[:, 3])      # R = RTOE|RHEEL
    return st, plan, act


# ---------------------------------------------------------- fig6_1
def fig1():
    """fig6_1 基座/质心跟踪：左=仿真三档，右=实机三档（同口径稳态窗）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(2, 2, figsize=(6.8, 3.2), sharex=False)
    grid = np.arange(0, 8, 0.01)   # 稳态窗相对时间（窗起点=0）
    for col, (title, is_real) in enumerate((("仿真", False), ("实机", True))):
        groups = REAL if is_real else RUNS
        for v in ("v00", "v01", "v03"):
            vxs, hs, vm, vs, zm, zs = [], [], [], [], [], []
            for n in groups[v]:
                t, vx, zc = base_series(n, is_real, CMD[v])
                t = t - t[0]
                vxs.append(np.interp(grid, t, vx, left=np.nan, right=np.nan))
                hs.append(np.interp(grid, t, zc, left=np.nan, right=np.nan))
                vm.append(vx.mean()); vs.append(vx.std())
                zm.append(np.nanmean(zc)); zs.append(np.nanstd(zc))
            vxs, hs = np.array(vxs), np.array(hs)
            mu, sd = np.nanmean(vxs, 0), np.nanstd(vxs, 0)
            # 图例=稳态窗 mean±std（与表6.4/表6.6 同口径，多 run 取均值）
            axes[0, col].plot(grid, mu, color=colors3[v], lw=1.1,
                              label=f"vx={CMD[v]:.1f}: {np.mean(vm):.3f}±{np.mean(vs):.3f}")
            axes[0, col].fill_between(grid, mu-sd, mu+sd, color=colors3[v], alpha=.15, lw=0)
            axes[0, col].axhline(CMD[v], color=GREY, ls="--", lw=.7)
            mh, sh = np.nanmean(hs, 0), np.nanstd(hs, 0)
            axes[1, col].plot(grid, mh, color=colors3[v], lw=1.1,
                              label=f"vx={CMD[v]:.1f}: {np.mean(zm):.3f}±{np.mean(zs):.4f} m")
            axes[1, col].fill_between(grid, mh-sh, mh+sh, color=colors3[v], alpha=.15, lw=0)
        axes[0, col].set_title(f"({chr(97+col)}) {title}", fontsize=9.5, loc="left")
        axes[0, col].set_ylabel("vx [m/s]", fontsize=9)
        axes[1, col].set_ylabel("质心高度 z_c [m]", fontsize=9)   # 6.4.1 口径：正解
        axes[1, col].set_xlabel("稳态窗时间 [s]", fontsize=9)
        for r in (0, 1):
            axes[r, col].legend(fontsize=6.5, frameon=False)
            axes[r, col].grid(alpha=.25)
            axes[r, col].tick_params(colors=GREY, labelsize=8)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_1_base_tracking.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_1 done（左仿真/右实机）->", OUT)


# ---------------------------------------------------------- fig6_2
def fig2():
    """fig6_2 接触时序：上两行=仿真（左/右足），下两行=实机，三速度列。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(4, 3, figsize=(6.8, 4.6), sharex=False)
    for col, v in enumerate(("v00", "v01", "v03")):
        for half, is_real in ((0, False), (1, True)):
            name = (REAL if is_real else RUNS)[v][0]
            d = load_real(name) if is_real else load_run(name)
            st, plan, act = contact_arrays(d, is_real)
            lo, hi = ((real_window(d, CMD[v]) if is_real else sim_window(d, CMD[v])))
            lo = lo + 0.2
            hi = min(hi, lo + 6.0)
            w = (st >= lo) & (st <= hi)
            t2, plan2, act2 = st[w] - lo, plan[w], act[w]
            for foot in (0, 1):
                ax = axes[2*half + foot, col]
                ax.fill_between(t2, 0, plan2[:, foot], step="post",
                                color=GREEN, alpha=.55, lw=0)
                ax.plot(t2, act2[:, foot]*0.6, drawstyle="steps-post",
                        color=BLUE if not is_real else YELLOW, lw=1.2)
                bad = plan2[:, foot] != act2[:, foot]
                if bad.any():
                    ax.plot(t2[bad], act2[bad, foot]*0.6, "s", ms=1.2, color=RED, lw=0)
                agree = 100.0 * (plan2[:, foot] == act2[:, foot]).mean()
                ns_p = int(np.sum(np.abs(np.diff(plan2[:, foot])) > 0))
                ns_a = int(np.sum(np.abs(np.diff(act2[:, foot])) > 0))
                ax.text(.98, .06, f"agree {agree:.1f}%  切换 {ns_p}/{ns_a}",
                        ha="right", va="bottom", fontsize=7,
                        transform=ax.transAxes, color=GREY)
                ax.set_ylim(-0.15, 1.4)
                ax.set_yticks([])
                ax.tick_params(colors=GREY, labelsize=8)
                for s in ax.spines.values():
                    s.set_color(GREY)
                if col == 0:
                    tag = "仿真" if not is_real else "实机"
                    ft = "左足" if foot == 0 else "右足"
                    ax.set_ylabel(f"{tag}\n{ft}", fontsize=8.5)
                if half == 1 and foot == 1:
                    ax.set_xlabel("时间 [s]", fontsize=9)
                if half == 0 and foot == 0:
                    ax.set_title(f"vx = {CMD[v]:.1f} m/s", fontsize=9.5)
    axes[0, 2].plot([], [], color=GREEN, alpha=.55, lw=6, label="计划接触（实线带）")
    axes[0, 2].plot([], [], color=BLUE, lw=1.2, label="实测接触（仿真）")
    axes[0, 2].plot([], [], color=YELLOW, lw=1.2, label="实测接触（实机）")
    axes[0, 2].plot([], [], "s", color=RED, ms=3, label="不一致")
    axes[0, 2].legend(loc="upper right", bbox_to_anchor=(1.02, 1.45),
                      fontsize=7, frameon=False, ncol=2)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_2_contact_timing.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_2 done（仿真上/实机下）->", OUT)


# ---------------------------------------------------------- fig6_3
def fig3():
    """fig6_3 力矩/关节跟踪：12 关节 4 组，仿真与实机并列柱（稳态窗同口径）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False
    # 12 关节分 4 组：髋滚转(0,6) 髋俯仰+偏航(1,2,7,8) 膝(3,9) 踝(4,5,10,11)
    groups = [("髋滚转", [0, 6], 220), ("髋俯仰/偏航", [1, 2, 7, 8], 396),
              ("膝", [3, 9], 396), ("踝", [4, 5, 10, 11], 112)]

    def joint_stats(name, is_real, cmd):
        d = load_real(name) if is_real else load_run(name)
        lo, hi = real_window(d, cmd) if is_real else sim_window(d, cmd)
        rt = d["realTorque"]
        w = (rt[:, 0] >= lo) & (rt[:, 0] <= hi)
        tq = rt[w, 1:]
        tp, jv = d["targetPos"], d["jointsPosVel"]
        wm = (tp[:, 0] >= lo) & (tp[:, 0] <= hi)
        tp = tp[wm]
        idx = np.clip(np.searchsorted(jv[:, 0], tp[:, 0]), 1, len(jv) - 1)
        left = np.abs(tp[:, 0] - jv[idx - 1, 0]) <= np.abs(tp[:, 0] - jv[idx, 0])
        qe = tp[:, 1:13] - jv[np.where(left, idx - 1, idx), 1:13]
        return (np.sqrt((tq**2).mean(axis=0)), np.abs(tq).max(axis=0),
                np.sqrt((qe**2).mean(axis=0)), np.abs(qe).max(axis=0))

    stats = {}
    for is_real, groups_runs in ((False, RUNS), (True, REAL)):
        acc = [joint_stats(n, is_real, CMD[v]) for v, names in groups_runs.items()
               for n in names]
        stats[is_real] = (np.mean([a[0] for a in acc], axis=0),   # tau rms（run 均值）
                          np.max([a[1] for a in acc], axis=0),   # tau peak（run max）
                          np.mean([a[2] for a in acc], axis=0),  # q rms
                          np.max([a[3] for a in acc], axis=0))   # q peak
    sim_rms, sim_pk, sim_qr, sim_qp = stats[False]
    real_rms, real_pk, real_qr, real_qp = stats[True]

    fig, axes = plt.subplots(2, 1, figsize=(6.8, 4.0), sharex=True)
    xs, xlab = [], []
    x = 0
    w = .38
    for gi, (gname, idxs, limit) in enumerate(groups):
        for j in idxs:
            for k, (rms, pk, c_fill, c_line) in enumerate((
                    (sim_rms[j], sim_pk[j], BLUE, BLUE),
                    (real_rms[j], real_pk[j], YELLOW, YELLOW))):
                xx = x + (k - .5) * w
                axes[0].bar(xx, rms, width=w*.92, color=c_fill,
                            alpha=1.0 if k == 0 else .85,
                            edgecolor=c_line, lw=.8 if k else 0)
                axes[0].plot([xx, xx], [rms, pk], color=GREY, lw=1)
                axes[0].plot([xx], [pk], "_", color=GREY, ms=6)
                qr = (sim_qr, real_qr)[k][j] * 180 / np.pi
                qp = (sim_qp, real_qp)[k][j] * 180 / np.pi
                axes[1].bar(xx, qr, width=w*.92, color=c_fill,
                            alpha=1.0 if k == 0 else .85,
                            edgecolor=c_line, lw=.8 if k else 0)
                axes[1].plot([xx, xx], [qr, qp], color=GREY, lw=1)
                axes[1].plot([xx], [qp], "_", color=GREY, ms=6)
            xs.append(x)
            xlab.append(f"J{j+1}")
            x += 1
        mid = x - len(idxs)/2 - .5
        axes[0].plot([x-len(idxs)-.5, x-.5], [limit, limit], color=RED,
                     ls="--", lw=.9)
        axes[0].annotate(f"{gname} (峰值 {limit})", (mid, limit),
                         textcoords="offset points", xytext=(0, 3),
                         ha="center", fontsize=7, color=RED)
        if gi < len(groups)-1:
            for ax in axes:
                ax.axvline(x-.5, color=GREY, lw=.5, alpha=.5)
    axes[0].set_ylim(0, 460)          # 让 396 参考线可见
    axes[0].set_ylabel("力矩 [N·m]", fontsize=9)
    axes[1].set_ylabel("关节角误差 [°]", fontsize=9)
    axes[1].set_xlabel("关节（12 关节分 4 组；柱=RMS，须=峰值；蓝=仿真，黄=实机）",
                       fontsize=9)
    axes[0].set_title("(a) 力矩使用与表2.4 峰值水平", fontsize=9.5, loc="left")
    axes[1].set_title("(b) 关节角跟踪误差（仿真/实机并列）", fontsize=9.5, loc="left")
    axes[1].set_xticks(xs)
    axes[1].set_xticklabels(xlab, fontsize=7)
    for ax in axes:
        ax.tick_params(colors=GREY, labelsize=8)
        ax.grid(alpha=.25, axis="y")
        for s in ax.spines.values():
            s.set_color(GREY)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_3_torque_tracking.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_3 done（仿真/实机并列）->", OUT)


# ---------------------------------------------------------- fig6_4
def fig4():
    """fig6_4 求解实时性：仿真/实机耗时分布（左）与时间散点（右）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False

    def collect(is_real):
        mpc, wbc, mpc_t, wbc_t = [], [], [], []
        groups = REAL if is_real else RUNS
        for names in groups.values():
            for n in names:
                d = load_real(n) if is_real else load_run(n)
                for key, arr, tt in (("mpc_solve_time_ms", mpc, mpc_t),
                                     ("wbc_solve_time_ms", wbc, wbc_t)):
                    a = d[key]
                    arr.append(a[:, 1])
                    tt.append(a[:, 0] - a[0, 0])
        return (np.concatenate(mpc), np.concatenate(wbc),
                np.concatenate(mpc_t), np.concatenate(wbc_t))

    sim_m, sim_w, sim_mt, sim_wt = collect(False)
    real_m, real_w, real_mt, real_wt = collect(True)

    fig, axes = plt.subplots(1, 2, figsize=(6.8, 3.2))
    for arr, c, name in ((sim_m, BLUE, "MPC 仿真"), (sim_w, GREEN, "WBC 仿真")):
        axes[0].hist(arr, bins=60, color=c, alpha=.55, label=name, lw=0)
    for arr, c, name in ((real_m, BLUE, "MPC 实机"), (real_w, GREEN, "WBC 实机")):
        axes[0].hist(arr, bins=60, histtype="step", color=c, lw=1.1,
                     ls="--", label=name)
    axes[0].axvline(10, color=RED, ls="--", lw=1, label="预算 10 ms")
    axes[0].axvline(2, color=RED, ls=":", lw=1, label="预算 2 ms")
    axes[0].set_xlabel("单次求解耗时 [ms]", fontsize=9)
    axes[0].set_ylabel("样本数", fontsize=9)
    axes[0].legend(fontsize=6.5, frameon=False, ncol=2)
    axes[0].tick_params(colors=GREY, labelsize=8)

    for arr, tt, budget, c, ms, lbl in ((sim_m, sim_mt, 10, BLUE, .6, "MPC 仿真"),
                                        (sim_w, sim_wt, 2, GREEN, .6, "WBC 仿真"),
                                        (real_m, real_mt, 10, BLUE, 1.2, "MPC 实机"),
                                        (real_w, real_wt, 2, GREEN, 1.2, "WBC 实机")):
        over = arr > budget
        mk = "." if ms < 1 else "x"
        axes[1].plot(tt[~over], arr[~over], mk, ms=ms, color=c,
                     alpha=.45 if ms < 1 else .8, label=lbl)
        if over.any():
            axes[1].plot(tt[over], arr[over], ".", ms=2, color=RED)
    axes[1].axhline(10, color=RED, ls="--", lw=1)
    axes[1].axhline(2, color=RED, ls=":", lw=1)
    axes[1].set_ylim(0, 11.5)          # 两条预算线都可见
    axes[1].set_xlabel("run 内时间 [s]", fontsize=9)
    axes[1].set_ylabel("求解耗时 [ms]", fontsize=9)
    axes[1].legend(fontsize=6.5, frameon=False, ncol=2)
    axes[1].tick_params(colors=GREY, labelsize=8)

    def cap(a, over, budget):
        return (f"mean {a.mean():.2f}  p95 {np.percentile(a,95):.2f}  "
                f"p99 {np.percentile(a,99):.2f}  max {a.max():.2f} ms；"
                f"超预算率 {100*over.mean():.2f}%（预算 {budget:.0f} ms）")
    axes[0].annotate("仿真  " + cap(sim_m, sim_m > 10, 10), (0, 1.04),
                     xycoords="axes fraction", fontsize=7, color=BLUE, va="bottom")
    axes[0].annotate("实机  " + cap(real_m, real_m > 10, 10), (0, 1.11),
                     xycoords="axes fraction", fontsize=7, color=GREY, va="bottom")
    for ax in axes:
        ax.grid(alpha=.25)
        for s in ax.spines.values():
            s.set_color(GREY)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_4_solve_time.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_4 done（仿真/实机）->", OUT)
    print("  仿真 MPC:", cap(sim_m, sim_m > 10, 10))
    print("  实机 MPC:", cap(real_m, real_m > 10, 10))
    print("  仿真 WBC:", cap(sim_w, sim_w > 2, 2))
    print("  实机 WBC:", cap(real_w, real_w > 2, 2))


# ---------------------------------------------------------- fig6_5
def fig5(state="stand", direction="lat"):
    """扰动恢复 2x2：(a) 滚转角 (b) 侧向/纵向偏移 (c) 髋滚转力矩 (d) t_rec/峰值姿态。
    仿真=实线+实心点，实机=虚线+空心点（同工况对照）。t=0 为注入回执时刻。"""
    import re
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False

    def load_push_sim(name):
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        want = {"/sim_push_fired", "/ground_truth/state", "/realTorque"}
        r = SequentialReader()
        r.open(StorageOptions(uri=os.path.join(run_dir(name), "bag"),
                              storage_id="mcap"), ConverterOptions("", ""))
        types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()
                 if t.name in want}
        ev = {k: [] for k in want}
        while r.has_next():
            topic, data, ts = r.read_next()
            if topic in types:
                ev[topic].append((ts*1e-9, deserialize_message(data, types[topic])))
        return ev

    def run_series_sim(name):
        """→ (t_rel, roll_dev, axis_dev, hip) 或 None。"""
        ev = load_push_sim(name)
        if not ev["/sim_push_fired"]:
            return None
        t0 = ev["/sim_push_fired"][0][0]
        import math
        gt = np.array([[t] + [m.pose.pose.position.x, m.pose.pose.position.y,
                              m.pose.pose.position.z,
                              math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.x +
                                            m.pose.pose.orientation.y*m.pose.pose.orientation.z),
                                         1-2*(m.pose.pose.orientation.x**2 +
                                              m.pose.pose.orientation.y**2))]
                       for t, m in ev["/ground_truth/state"]])
        t, px, py, roll = gt[:, 0]-t0, gt[:, 1], gt[:, 2], gt[:, 4]
        pre = (t >= -1) & (t < 0)
        r0 = roll[pre].mean()
        a0 = (py if direction == "lat" else px)[pre].mean()
        axis = (py - a0) if direction == "lat" else (px - a0)
        hip = np.array([[tt, max(abs(m.data[0]), abs(m.data[6]))]
                        for tt, m in ev["/realTorque"]])
        hip[:, 0] -= t0
        return t, roll-r0, axis, hip

    def run_series_real(name):
        d = load_real(name)
        if not len(d["real_push_fired"]):
            return None
        t0 = d["real_push_fired"][0, 0]
        obs = d["humanoid_mpc_observation"]
        t = obs[:, 0] - t0
        px, py, roll = obs[:, 7], obs[:, 8], obs[:, 12]
        pre = (t >= -1) & (t < 0)
        if not pre.any():
            return None
        r0 = roll[pre].mean()
        a0 = (py if direction == "lat" else px)[pre].mean()
        axis = (py - a0) if direction == "lat" else (px - a0)
        hip = np.column_stack([d["realTorque"][:, 0] - t0,
                               np.maximum(np.abs(d["realTorque"][:, 1]),
                                          np.abs(d["realTorque"][:, 7]))])
        return t, roll-r0, axis, hip

    levels = {"L1": 3.1, "L2": 6.2, "L3": 12.4}
    series = {"sim": {}, "real": {}}   # side -> level -> list of tuple
    scatter = {"sim": [], "real": []}
    for side in ("sim", "real"):
        for lv, imp in levels.items():
            series[side][lv] = []
            pat = (f"sim_push_{direction}_{lv}_{state}_*" if side == "sim"
                   else f"real_push_{direction}_{lv}_{state}_*")
            root = os.path.join(RA9, "sim_data" if side == "sim" else "real")
            for p in sorted(glob.glob(os.path.join(root, pat))):
                n = os.path.basename(p)
                if side == "sim":
                    ps = os.path.join(p, "analysis", "push_summary.txt")
                    if not os.path.exists(ps):
                        continue
                    txt = open(ps).read()
                    if "注入疑似未生效" in txt or "注入前已倒地" in txt:
                        continue
                    s = run_series_sim(n)
                    tr = re.search(r"t_rec=([0-9.]+|NA\S*)", txt)
                    trv = float(tr.group(1)) if tr and tr.group(1)[0].isdigit() else float("nan")
                else:
                    ms = os.path.join(p, "analysis", "metrics.csv")
                    if not os.path.exists(ms):
                        continue
                    row = next(csv.DictReader(open(ms)))
                    if str(row.get("push_fallen")) == "True":
                        continue              # 倒地样本单列于正文，不进曲线
                    s = run_series_real(n)
                    tv = row.get("push_t_rec_att") or row.get("push_t_rec")
                    try:
                        trv = float(tv)
                    except (TypeError, ValueError):
                        trv = float("nan")
                if s is None:
                    continue
                t, roll, axis, hip = s
                w = (t >= -1) & (t <= 6)
                series[side][lv].append((t[w], roll[w], axis[w], hip))
                scatter[side].append((imp, trv, float(np.abs(roll[w]).max())))

    fig, axes = plt.subplots(2, 2, figsize=(6.8, 4.2))
    colors = {"L1": BLUE, "L2": GREEN, "L3": RED}
    grid = np.arange(-1, 6, 0.01)   # 公共时间栅格：各 run 采样长度不同
    for lv in levels:
        for side, ls in (("sim", "-"), ("real", "--")):
            S = series[side][lv]
            if not S:
                continue
            rl = np.nanmean([np.interp(grid, s[0], s[1], left=np.nan, right=np.nan)
                             for s in S], axis=0)
            off = np.nanmean([np.interp(grid, s[0], s[2], left=np.nan, right=np.nan)
                              for s in S], axis=0)
            tag = "仿真" if side == "sim" else "实机"
            axes[0, 0].plot(grid, rl*180/np.pi, color=colors[lv], ls=ls, lw=1.1,
                            label=f"{lv} {tag}（n={len(S)}）")
            axes[0, 1].plot(grid, off*100, color=colors[lv], ls=ls, lw=1.1,
                            label=f"{lv} {tag}")
    axes[0, 0].axhline(0.05*180/np.pi, color=GREY, ls="--", lw=.8)
    axes[0, 0].axhline(-0.05*180/np.pi, color=GREY, ls="--", lw=.8)
    axes[0, 0].axvline(0, color=GREY, ls=":", lw=.8)
    axes[0, 0].set_xlabel("t - t_push [s]", fontsize=9)
    axes[0, 0].set_ylabel("滚转角偏差 [°]", fontsize=9)
    axes[0, 0].legend(fontsize=6, frameon=False, ncol=2)
    axes[0, 1].axvline(0, color=GREY, ls=":", lw=.8)
    axes[0, 1].set_xlabel("t - t_push [s]", fontsize=9)
    axes[0, 1].set_ylabel(f"{'侧向' if direction=='lat' else '纵向'}偏移 [cm]", fontsize=9)
    axes[0, 1].legend(fontsize=6, frameon=False, ncol=2)
    for lv in levels:
        for side, ls in (("sim", "-"), ("real", "--")):
            S = series[side][lv]
            if not S:
                continue
            hps = np.nanmean([np.interp(np.arange(0, 5, .02), s[3][:, 0], s[3][:, 1],
                                        left=np.nan, right=np.nan) for s in S], axis=0)
            axes[1, 0].plot(np.arange(0, 5, .02), hps, color=colors[lv], ls=ls,
                            lw=1.1, label=f"{lv} {'仿真' if side=='sim' else '实机'}")
    axes[1, 0].axhline(143.89, color=RED, ls="--", lw=.8, label="143.89 Nm")
    axes[1, 0].axhline(220, color=RED, ls=":", lw=.8, label="220 Nm")
    axes[1, 0].set_xlabel("t - t_push [s]", fontsize=9)
    axes[1, 0].set_ylabel("髋滚转力矩峰值 [N·m]", fontsize=9)
    axes[1, 0].legend(fontsize=6, frameon=False, ncol=2)
    ax2 = axes[1, 1].twinx()
    for side, mk, c in (("sim", "o", BLUE), ("real", "s", YELLOW)):
        sc = scatter[side]
        if not sc:
            continue
        imp_a = np.array([s[0] for s in sc])
        tr_a = np.array([s[1] for s in sc])
        pk_a = np.array([s[2] for s in sc])*180/np.pi
        ok = np.isfinite(tr_a)
        lbl = "t_rec(仿真)" if side == "sim" else "t_rec(实机)"
        axes[1, 1].plot(imp_a[ok], tr_a[ok], mk, ms=4, color=c,
                        mfc=c if side == "sim" else "none", label=lbl)
        axes[1, 1].plot(imp_a[~ok], np.zeros((~ok).sum()), "x", ms=5, color=GREY)
        ax2.plot(imp_a, pk_a, "^" if side == "sim" else "v", ms=3.5,
                 color=RED if side == "sim" else "#c5221f", alpha=.7,
                 mfc=RED if side == "sim" else "none",
                 label="峰值姿态(仿真)" if side == "sim" else "峰值姿态(实机)")
    ax2.set_ylabel("峰值姿态偏差 [°]", fontsize=8, color=RED)
    ax2.tick_params(colors=RED, labelsize=7)
    axes[1, 1].set_xlabel("冲量 J [N·s]", fontsize=9)
    axes[1, 1].set_ylabel("t_rec [s]", fontsize=9)
    axes[1, 1].legend(fontsize=6, frameon=False, loc="upper left")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]):
        ax.grid(alpha=.25)
        ax.tick_params(colors=GREY, labelsize=8)
        for s in ax.spines.values():
            s.set_color(GREY)
    fig.tight_layout()
    tag = f"{direction}_{state}"
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_5_recovery_{tag}.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print(f"fig6_5[{tag}] done ->", OUT,
          f"(samples sim={len(scatter['sim'])} real={len(scatter['real'])})")


# ---------------------------------------------------------- fig6_6
def fig6():
    """fig6_6 sim2real：四组指标双柱对比（仿真 vs 实机，0.1+0.3 档聚合）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False

    def sim_metric(name, cat, item):
        for r in csv.DictReader(open(os.path.join(run_dir(name),
                                                 "analysis", "metrics.csv"))):
            if r["category"] == cat and r["item"] == item:
                return float(r["value"])
        return float("nan")

    def real_metric(name, key):
        r = next(csv.DictReader(open(os.path.join(real_run_dir(name),
                                                 "analysis", "metrics.csv"))))
        return float(r[key])

    def agg(side):
        names = ((RUNS if side == "sim" else REAL)["v01"] +
                 (RUNS if side == "sim" else REAL)["v03"])
        zc, qe, ag, mpc = [], [], [], []
        for n in names:
            if side == "sim":
                zc.append(sim_metric(n, "base_tracking", "zc_std"))
                qe.append(sim_metric(n, "joint_tracking", "rms_total_excl"))
                ag += [sim_metric(n, "contact_timing", f"{f}.agree_pct")
                       for f in ("L", "R")]
                mpc.append(sim_metric(n, "solve_time", "mpc.mean"))
            else:
                zc.append(real_metric(n, "track_zc_std"))
                qe.append(real_metric(n, "joint_rms"))
                ag += [real_metric(n, f"contact_{f}_agree_pct") for f in ("L", "R")]
                mpc.append(real_metric(n, "solve_mpc_mean"))
        return dict(zc=np.nanmean(zc), qe=np.nanmean(qe),
                    agree=np.nanmean(ag), mpc=np.nanmean(mpc))

    sim, rea = agg("sim"), agg("real")
    keys = [("z_c std [m]", "zc", 4), ("关节角误差 RMS [rad]", "qe", 4),
            ("接触 agree [%]", "agree", 1), ("MPC 耗时 [ms]", "mpc", 2)]
    fig, ax = plt.subplots(figsize=(6.8, 2.8))
    xs = np.arange(len(keys))
    w = .35
    b1 = ax.bar(xs - w/2, [sim[k] for _, k, _ in keys], w, color=BLUE, label="仿真")
    b2 = ax.bar(xs + w/2, [rea[k] for _, k, _ in keys], w, color=YELLOW,
                label="实机")
    for bars, vals in ((b1, [sim[k] for _, k, _ in keys]),
                       (b2, [rea[k] for _, k, _ in keys])):
        for bb, vv, (_, _, nd) in zip(bars, vals, keys):
            ax.annotate(f"{vv:.{nd}f}", (bb.get_x()+bb.get_width()/2, vv),
                        textcoords="offset points", xytext=(0, 2),
                        ha="center", fontsize=7)
    for i, (_, k, _) in enumerate(keys):
        if sim[k] != 0:
            d = 100 * (rea[k] - sim[k]) / abs(sim[k])
            ax.annotate(f"{d:+.0f}%", (i, max(sim[k], rea[k])),
                        textcoords="offset points", xytext=(0, 14),
                        ha="center", fontsize=7.5, color=RED)
    ax.set_xticks(xs)
    ax.set_xticklabels([n for n, _, _ in keys], fontsize=8.5)
    ax.set_ylabel("数值（单位见标签）", fontsize=9)
    ax.legend(fontsize=7.5, frameon=False)
    ax.grid(alpha=.25, axis="y")
    ax.tick_params(colors=GREY, labelsize=8)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_6_sim2real.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_6 done（仿真/实机实测）->", OUT)
    print("  sim :", sim)
    print("  real:", rea)


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("fig1", "all"):
        fig1()
    if what in ("fig2", "all"):
        fig2()
    if what in ("fig3", "all"):
        fig3()
    if what in ("fig4", "all"):
        fig4()
    if what in ("fig5", "all"):
        fig5("stand", "lat")
        fig5("stand", "fwd")
        fig5("walk", "lat")
    if what in ("fig6", "all"):
        fig6()
