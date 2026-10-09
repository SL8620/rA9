# C6 交付物（数据 / 图表）

论文第 6 章《系统集成与实验验证》的交付物落盘处。需求源：`markdown/C6交付需求.md`（论文仓），
回填目标：`markdown/C6修改版.md` 的【待补】槽位。

**总原则**：所有数字可由脚本从录包复算，不估算不编造；拿不到的数据标"未获得+原因"，不用近似值填。

## 目录

```
deliverables/C6/
├── README.md          本文件：交付清单与槽位映射
├── analysis/          每 run 的指标统计（summary.txt / metrics.csv / NOTE.md）
└── figures/           图 6.1–6.8（.png 300dpi + 同名 .svg 源）
```

原始 rosbag 在 `experiments/<run>/`（gitignore，不入库）；本目录只留可复算的小体积结果。
脚本在 `scripts/`（`analyze_walk.py` / `analyze_push.py` / `run_*.sh` / `collect.sh`）。

## 交付清单

| # | 项 | 状态 | 落盘 |
|---|---|---|---|
| B0 | `analyze_walk.py` 恢复/重写（含 STATUS 判定） | ✅ | `scripts/analyze_walk.py` |
| B2 | 存量 run 补分析（v2/v3/smoke、v4-v6 nan 归因、repro） | ✅ | `analysis/` |
| B2 | ground_truth 速度口径核查结论 | ✅ | `analysis/NOTE.md` §1 |
| B2 | auto_v8 高度异常归因 | ✅ | `analysis/NOTE.md` §3 |
| A1 | 仿真基准补录（如存量 run 无效） | 🔴 | `experiments/sim_base_v*_*/` |
| A2 | 仿真扰动 27 run | 🔴 | `experiments/sim_push_*_*/` |
| B1 | `analyze_push.py` + 扰动分析 | 🔴 | `scripts/analyze_push.py` |
| A3 | 实机补录（含 solve-time；扰动视安全条件） | 🔴 | `experiments/real_*_*/` |
| C | 图 6.1–6.7（6.8 可选） | 🔴 | `figures/fig6_*` |
| D | 槽位回填 `C6修改版.md` + 图片插入 | 🔴 | 论文仓（不在本仓） |

## 槽位映射

| 交付物 | 回填槽位 |
|---|---|
| A1/B0 基准 run 统计 | 6.4.1 跟踪、6.4.2 时序、6.4.3 力矩、表6.5 |
| B0 耗时统计 | 6.4.4、表6.5 |
| A2/B1 扰动 | 6.4.5 全部、图6.5 |
| A3/B0 实机 | 6.5.2、6.5.3、6.5.5、表6.5 实机列 |
| B0 口径核查 | 6.4.1 现象讨论、6.5.2、6.5.5、6.6 局限 |
| B0 模型自洽 | 6.3.4 |
| B0 异常统计 | 6.5.4、表6.6 |
| C 图表 | 各插图位（替换注释为 `<img>`） |

## 验收标准

1. **可复算**：任意交付数字能在 run 目录上重跑得到；脚本留在 `rA9/scripts/`。
2. **无静默缺数**：nan / 缺话题必须在 summary 标注原因，不插值填充。
3. **口径一致**：单位 m / rad / N / N·m / s 或 ms；稳态窗规则统一；sim / real 标注明确。
4. **图规格达标**：PNG 300 dpi（宽 6.8 in）+ 同名 SVG；中文 Noto Sans SC ≥10 pt；
   配色 `#4285f4` 蓝(仿真) / `#34a853` 绿(参考·计划) / `#ea4335` 红(超限·偏差) / `#555` 灰(坐标轴)；白底，单位齐全。

## 图表规格速查

| 文件 | 尺寸(in) | 数据源 | 要点 |
|---|---|---|---|
| fig6_1_base_tracking | 6.8×3.2 | A1/A3 + walk_smoke | 双栏，左仿真三档 vx 时序+指令虚线+z_c±std 色带，右实机同式；图例含 mean±std |
| fig6_2_contact_timing | 6.8×3.4 | cmd/simContactFlag | 每足一行甘特条带，不一致段红描；右下标 agree% 与 planned/actual 切换数 |
| fig6_3_torque_tracking | 6.8×3.6 | targetTorque/realTorque/jointsPosVel | 上 12 关节分 4 组柱(RMS 柱+峰值须)，叠 396/220/112 参考线，超限标红注值；下关节角误差同布局；sim/real 并列 |
| fig6_4_solve_time | 6.8×3.2 | solve-time 话题 | 左直方图+箱线，右耗时-时间散点，10/2 ms 红虚线，超预算点标红；图注给 mean/p95/p99/max+超预算率 |
| fig6_5_recovery | 6.8×3.6 | A2 | 2×2：(a)滚转角三档三线+注入点+ε 虚线+t_rec 箭头 (b)质心横移 (c)髋滚转力矩叠 143.89/220 (d)t_rec 与峰值姿态散点+拟合 |
| fig6_6_sim2real | 6.8×2.8 | 6.4/6.5 汇总 | 四组分组柱(z_c std、力矩 RMS、agree%、vx 均值)，sim/real 双柱，柱顶标值柱间标差异% |
| fig6_7_data_pipeline | 5.5×3.0 | 流程 | HTML/SVG，风格同 `media/fig4_x_mpc_solve_flow.html`；录制→分析→指标→图表，回环"params.txt 留档→复现校验" |
| fig6_8_model_checks（可选） | 4.0×2.2 | B0 附加 | FK/IK 回代误差散点 + 踝映射迭代收敛曲线 |

图题随图交付：`图6.X 标题`，与正文图表引用顺序一致。

## 交付物补充说明（2026-09-30 Phase 0 冻结）

- `analysis/NOTE.md`：B2 三个待决项归因 + 口径分歧（§5 z 高度口径需论文侧决定）。
- `analysis/runs_status.md`：全部存量 run 的 STATUS/RTF 一览（无静默缺数）。
- `analysis/<run>/`：两个可用样本（walk_smoke_001、manual_repro_003）的
  summary/metrics/fig1–4 完整拷贝；其余 run 判 INVALID，看 `runs_status.md`。
- **2026-10-09 更新**：自动 run 翻倒根因已定位并修复（使能竞态 + 参考未重锚，
  见 NOTE §7）；旧"RTF≠1 时间基准不一致"结论**是测量伪影已废除**（NOTE §8：
  仿真节拍 RTF=1.000，录包有 ~20% 整批丢包）。修复后回归 run
  （rtf_smoke_004/005、rtf_verify_001、rtf_final_001）全 VALID，RTF 1.000。
  **A1/A2 收数可按正常流程进行**；分析口径以 `/sim_time` 跨度法 RTF +
  录包捕获率为准。

## 规矩

- 每个实验节点冻结 git 状态再收数据（`params.txt` 记 `git_commit`+`git_dirty`，论文数字须可溯源到提交哈希）。
- rosbag / 大文件不进 git；`experiments/` 保持 gitignore。
- **收带 solve-time 的数据必须用本分支**（埋点 `/mpc_solve_time_ms` `/wbc_solve_time_ms` 只在
  `lapi_branch` 之后的提交里，冻结版 `86121b0` 无）。build/install 若是从冻结版编的需重建。
