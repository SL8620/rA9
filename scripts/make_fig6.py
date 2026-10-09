#!/usr/bin/env python3
"""C6 论文图 6.2/6.3/6.4 生成（交付需求 C 规格）。

规格要点：PNG 300dpi（宽 6.8in）+ 同名 SVG；中文 Noto Sans SC ≥10pt；
配色 #4285f4 蓝(仿真) / #34a853 绿(参考·计划) / #ea4335 红(超限·偏差) / #555 灰(坐标轴)；
白底、单位齐全。输出 deliverables/C6/figures/（交付时拷入论文仓 media/C6修改版_media/）。

用法: make_fig6.py fig2|fig3|fig4|all
数据: A1 有效 run（sim_base_v0{0,1,3}_*，见 runs_status.md），bag 读取缓存到 /tmp/fig6_cache/
"""
import glob
import os
import sys

if "AMENT_PREFIX_PATH" not in os.environ:
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
BLUE, GREEN, RED, GREY = "#4285f4", "#34a853", "#ea4335", "#555555"

RA9 = "/home/mi/disk/SS/thesis/rA9"
CACHE = "/tmp/fig6_cache"
OUT = os.path.join(RA9, "deliverables/C6/figures")
os.makedirs(CACHE, exist_ok=True)
os.makedirs(OUT, exist_ok=True)

# A1 有效 run（3 速度 × 3 重复）
RUNS = {
    "v00": ["sim_base_v00_01", "sim_base_v00_03", "sim_base_v00_04"],
    "v01": ["sim_base_v01_01", "sim_base_v01_02", "sim_base_v01_03"],
    "v03": ["sim_base_v03_02", "sim_base_v03_04", "sim_base_v03_06"],
}
CMD = {"v00": 0.0, "v01": 0.1, "v03": 0.3}


def load_run(name):
    """bag → dict of np arrays；缓存 npz。"""
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
    r.open(StorageOptions(uri=os.path.join(RA9, "experiments", name, "bag"),
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


def steady_slice(d, cmd, band=None):
    """稳态窗：解暂停后进入目标带（vx）并保持 0.3s；返回布尔掩码（对 gt 行）。"""
    gt = d["ground_truth/state"]
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


# ---------------------------------------------------------- fig6_2
def fig2():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(2, 3, figsize=(6.8, 3.4), sharex=False)
    for col, (v, names) in enumerate(RUNS.items()):
        d = load_run(names[0])          # 每速度取首个有效 run 的代表性 6s
        ct, cv = d["cmd_contactFlag"][:, 0], d["cmd_contactFlag"][:, 1:]
        st, sv = d["simContactFlag"][:, 0], d["simContactFlag"][:, 1:]
        rt = d["realTorque"]
        t0 = rt[0, 0] if len(rt) else st[0, 0]
        # 稳态段内取 6s 窗口（解暂停后 3s 起）
        lo, hi = t0 + 3.0, t0 + 9.0
        cm = (ct >= lo) & (ct <= hi)
        sm = (st >= lo) & (st <= hi)
        ct2, cv2 = ct[cm]-lo, cv[cm]
        st2, sv2 = st[sm]-lo, sv[sm]
        # 计划足级（2 元素 [L,R]）最近邻对齐到实测时间轴
        idx = np.clip(np.searchsorted(ct2, st2), 1, max(len(ct2)-1, 1))
        left = np.abs(st2-ct2[idx-1]) <= np.abs(st2-ct2[idx])
        plan = cv2[np.where(left, idx-1, idx)]
        act = np.zeros_like(plan)
        act[:, 0] = np.maximum(sv2[:, 0], sv2[:, 2])   # L = LTOE|LHEEL
        act[:, 1] = np.maximum(sv2[:, 1], sv2[:, 3])   # R
        for foot in (0, 1):
            ax = axes[foot, col]
            name = "左足" if foot == 0 else "右足"
            ax.fill_between(st2, 0, plan[:, foot], step="post",
                            color=GREEN, alpha=.55, lw=0)
            ax.plot(st2, act[:, foot]*0.6, drawstyle="steps-post",
                    color=BLUE, lw=1.2)
            bad = plan[:, foot] != act[:, foot]
            if bad.any():
                ax.plot(st2[bad], act[bad, foot]*0.6, "s", ms=1.2,
                        color=RED, lw=0)
            agree = 100.0 * (plan[:, foot] == act[:, foot]).mean()
            ns_p = int(np.sum(np.abs(np.diff(plan[:, foot])) > 0))
            ns_a = int(np.sum(np.abs(np.diff(act[:, foot])) > 0))
            ax.text(.98, .06, f"agree {agree:.1f}%  切换 {ns_p}/{ns_a}",
                    ha="right", va="bottom", fontsize=7.5,
                    transform=ax.transAxes, color=GREY)
            ax.set_ylim(-0.15, 1.4)
            ax.set_yticks([])
            ax.tick_params(colors=GREY, labelsize=8)
            for s in ax.spines.values():
                s.set_color(GREY)
            if col == 0:
                ax.set_ylabel(f"{name}\n接触状态", fontsize=9)
            if foot == 1:
                ax.set_xlabel("时间 [s]", fontsize=9)
            if foot == 0:
                ax.set_title(f"vx = {CMD[v]:.1f} m/s", fontsize=9.5)
    axes[0, 2].plot([], [], color=GREEN, alpha=.55, lw=6, label="计划接触（实线带）")
    axes[0, 2].plot([], [], color=BLUE, lw=1.2, label="实测接触")
    axes[0, 2].plot([], [], "s", color=RED, ms=3, label="不一致")
    axes[0, 2].legend(loc="upper right", bbox_to_anchor=(1.02, 1.35),
                      fontsize=7.5, frameon=False, ncol=3)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_2_contact_timing.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_2 done ->", OUT)


# ---------------------------------------------------------- fig6_3
def fig3():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False
    # 12 关节分 4 组：髋滚转(0,6) 髋俯仰+偏航(1,2,7,8) 膝(3,9) 踝(4,5,10,11)
    groups = [("髋滚转", [0, 6], 220), ("髋俯仰/偏航", [1, 2, 7, 8], 396),
              ("膝", [3, 9], 396), ("踝", [4, 5, 10, 11], 112)]
    tau_rms, tau_peak, q_rms, q_peak = [], [], [], []
    for names in RUNS.values():
        for n in names:
            d = load_run(n)
            rt = d["realTorque"]
            tq = rt[:, 1:]                       # /realTorque 仅在非暂停段发布
            # targetPos(488Hz) 与 jointsPosVel(~420Hz) 频率不同，时间轴最近邻对齐
            t_tp, t_jv = d["targetPos"][:, 0], d["jointsPosVel"][:, 0]
            idx = np.clip(np.searchsorted(t_jv, t_tp), 1, len(t_jv) - 1)
            left = np.abs(t_tp - t_jv[idx - 1]) <= np.abs(t_tp - t_jv[idx])
            jv_al = d["jointsPosVel"][np.where(left, idx - 1, idx), 1:13]
            qe = d["targetPos"][:, 1:13] - jv_al  # 关节角跟踪误差
            tau_rms.append(np.sqrt((tq**2).mean(axis=0)))
            tau_peak.append(np.abs(tq).max(axis=0))
            q_rms.append(np.sqrt((qe**2).mean(axis=0)))
            q_peak.append(np.abs(qe).max(axis=0))
    tau_rms = np.mean(tau_rms, axis=0)
    tau_peak = np.max(tau_peak, axis=0)
    q_rms = np.mean(q_rms, axis=0)
    q_peak = np.max(q_peak, axis=0)

    fig, axes = plt.subplots(2, 1, figsize=(6.8, 3.6), sharex=True)
    xs, xlab = [], []
    x = 0
    for gi, (gname, idxs, limit) in enumerate(groups):
        for j in idxs:
            c = RED if tau_peak[j] > limit else BLUE
            axes[0].bar(x, tau_rms[j], width=.7, color=c)
            axes[0].plot([x, x], [tau_rms[j], tau_peak[j]], color=GREY, lw=1)
            axes[0].plot([x], [tau_peak[j]], "_", color=GREY, ms=6)
            if tau_peak[j] > limit:
                axes[0].annotate(f"{tau_peak[j]:.0f}", (x, tau_peak[j]),
                                 textcoords="offset points", xytext=(0, 3),
                                 ha="center", fontsize=6.5, color=RED)
            axes[1].bar(x, q_rms[j]*180/np.pi, width=.7, color=BLUE)
            axes[1].plot([x, x], [q_rms[j]*180/np.pi, q_peak[j]*180/np.pi],
                         color=GREY, lw=1)
            axes[1].plot([x], [q_peak[j]*180/np.pi], "_", color=GREY, ms=6)
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
    axes[1].set_xlabel("关节（12 关节分 4 组；柱=RMS，须=峰值）", fontsize=9)
    axes[0].set_title("(a) 力矩使用与表2.4 峰值水平", fontsize=9.5, loc="left")
    axes[1].set_title("(b) 关节角跟踪误差（仿真）", fontsize=9.5, loc="left")
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
    print("fig6_3 done ->", OUT)


# ---------------------------------------------------------- fig6_4
def fig4():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = FONT
    plt.rcParams["axes.unicode_minus"] = False
    mpc, wbc, mpc_t, wbc_t = [], [], [], []
    for names in RUNS.values():
        for n in names:
            d = load_run(n)
            for key, arr, tt in (("mpc_solve_time_ms", mpc, mpc_t),
                                 ("wbc_solve_time_ms", wbc, wbc_t)):
                a = d[key]
                arr.append(a[:, 1])
                tt.append(a[:, 0] - a[0, 0])
    mpc = np.concatenate(mpc); wbc = np.concatenate(wbc)
    mpc_t = np.concatenate(mpc_t); wbc_t = np.concatenate(wbc_t)

    fig, axes = plt.subplots(1, 2, figsize=(6.8, 3.2))
    for arr, c, name in ((mpc, BLUE, "MPC"), (wbc, GREEN, "WBC")):
        axes[0].hist(arr, bins=60, color=c, alpha=.65, label=name, lw=0)
    axes[0].axvline(10, color=RED, ls="--", lw=1, label="预算 10 ms")
    axes[0].axvline(2, color=RED, ls=":", lw=1, label="预算 2 ms")
    axes[0].set_xlabel("单次求解耗时 [ms]", fontsize=9)
    axes[0].set_ylabel("样本数", fontsize=9)
    axes[0].legend(fontsize=7.5, frameon=False)
    axes[0].tick_params(colors=GREY, labelsize=8)

    over_m = mpc > 10
    over_w = wbc > 2
    axes[1].plot(mpc_t[~over_m], mpc[~over_m], ".", ms=.6, color=BLUE, alpha=.5)
    axes[1].plot(mpc_t[over_m], mpc[over_m], ".", ms=2, color=RED)
    axes[1].plot(wbc_t[~over_w], wbc[~over_w], ".", ms=.6, color=GREEN, alpha=.5)
    axes[1].plot(wbc_t[over_w], wbc[over_w], ".", ms=2, color=RED)
    axes[1].axhline(10, color=RED, ls="--", lw=1)
    axes[1].axhline(2, color=RED, ls=":", lw=1)
    axes[1].set_ylim(0, 11.5)          # 两条预算线都可见
    axes[1].set_xlabel("run 内时间 [s]", fontsize=9)
    axes[1].set_ylabel("求解耗时 [ms]", fontsize=9)
    axes[1].tick_params(colors=GREY, labelsize=8)

    def cap(a, over, budget):
        return (f"mean {a.mean():.2f}  p95 {np.percentile(a,95):.2f}  "
                f"p99 {np.percentile(a,99):.2f}  max {a.max():.2f} ms；"
                f"超预算率 {100*over.mean():.2f}%（预算 {budget:.0f} ms）")
    axes[0].annotate(cap(mpc, over_m, 10), (0, 1.04), xycoords="axes fraction",
                     fontsize=7.5, color=BLUE, va="bottom")
    axes[0].annotate(cap(wbc, over_w, 2), (0, 1.12), xycoords="axes fraction",
                     fontsize=7.5, color=GREEN, va="bottom")
    for ax in axes:
        ax.grid(alpha=.25)
        for s in ax.spines.values():
            s.set_color(GREY)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig6_4_solve_time.{ext}"),
                    dpi=300 if ext == "png" else None)
    plt.close(fig)
    print("fig6_4 done ->", OUT)
    print("  MPC:", cap(mpc, over_m, 10))
    print("  WBC:", cap(wbc, over_w, 2))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("fig2", "all"):
        fig2()
    if what in ("fig3", "all"):
        fig3()
    if what in ("fig4", "all"):
        fig4()
