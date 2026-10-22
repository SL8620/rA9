# C6 数据复核 · sim_data/ 与 real/（2026-10-21）

复核范围：`rA9/sim_data/`（45 run，`_2x` 正典批）与 `rA9/real/`（42 run，落盘 2026-10-20 15:58）。
方法：metadata 全量解析 + mcap 记录级抽查（Header/Metadata/Statistics）+ `analyze_real.py` 实跑 2 个 run
（经布局垫层，见 §5.1）。所有数字可在本文列出的命令上复算。

## 0. 结论速览

| 数据集 | 完整性 | 可复算 | 作为论文证据 |
|---|---|---|---|
| `sim_data/` 42 交付 run（+3 smoke） | 通过 | 通过 | **可用**，作 6.4 定量主证据 |
| `real/` 42 run | 完整可读 | 需改脚本 | **来源存疑，暂不可作实机实测入文** |

`real/` 的存疑不是数据形态问题（形态很好），而是**来源可信度**问题：五条独立证据指向"按实验矩阵批量生成"
而非样机逐次录制，其中 wall-clock 重叠一条在物理上不可能被真实硬件实验满足。在作者澄清来源之前，
6.5 系列不得按"实机实测"表述，`待办.md` #6 的"合成占位替换"也不得用这批数据执行。

---

## 1. sim_data/（45 run）

### 1.1 矩阵完整性

| 格位 | 命名 | run 数 | 备注 |
|---|---|---|---|
| A1 基线 | `sim_base_v{00,01,03}_2{1,2,3}` | 9 | 三速度 × 3 重复，齐 |
| A2 fwd 扰动 | `sim_push_fwd_L{1,2,3}_stand_2{1,2,3}` | 9 | 与 real 侧一致（无 fwd_walk） |
| A2 lat 扰动 | `sim_push_lat_L{1,2}_{stand,walk}_2{1,2,3}` | 12 | 齐 |
| A2 lat_L3 补录 | `sim_push_lat_L3_stand_2{1..7}` | 7 | 补齐未倒样本 |
| A2 lat_L3 walk | `sim_push_lat_L3_walk_2{1..5}` | 5 | 补录 |
| 冒烟 | `smoke_newsem_0{1,2,3}` | 3 | 非交付 |

A2 共 33 run = 27 恢复 + 6 FALLEN，与 README 口径一致。

### 1.2 有效性与实时性（全部 42 交付 run）

- **STATUS**：36 VALID / 6 INVALID(倒地)。6 个倒地 run 全部落在 `lat_L3` 两格：
  `lat_L3_stand_22/23/24/26`、`lat_L3_walk_23/24`。
- **RTF**：全部 1.000（`sim_time` 跨度法），无一例外。
- **录包捕获率**：基线 84.2–84.9%，扰动 80.6–81.2%（`/simContactFlag` 实收/应发），
  同批次内稳定，无离群 run。
- **暂停占比**：基线 run 约 0.249（`unpaused` 窗已扣除，不影响稳态窗）。
- **分析产物**：42/42 有 `metrics.csv` + `summary.txt` + fig1–4；33/33 扰动 run 有
  `push_summary.txt` + fig5。无缺产物。

### 1.3 三速度基线数值（由 `sim_data/*/analysis/metrics.csv` 复算，n=3/档）

| 指标 | v00 | v01 | v03 |
|---|---|---|---|
| vx mean [m/s] | -0.0002±0.0003 | 0.0969±0.0006 | 0.2909±0.0015 |
| 基座高 mean [m] | 0.8646 | 0.8626 | 0.8582 |
| z_c mean [m] | 0.7050 | 0.7030 | 0.6989 |

run 间 std 在 10⁻³ 量级，三档 vx 跟踪偏差 0.3 / 3.1 / 3.0 mm/s，量级自洽。

### 1.4 遗留口径（沿用 NOTE.md，报数时必须带）

1. `/realTorque` 采样率低于 `/targetTorque`（本批实测约 318 Hz vs 480 Hz 等效），
   **峰值系统性低估**；报峰值用 `/targetTorque`，报误差用配对后的时间域差。
2. 报告口径用"剔除瞬态垃圾窗"行（`summary.txt` 已标）。
3. v00 的 duty 受 stance 污染已在本批修正（稳态窗加步态收敛下界）。

---

## 2. real/（42 run）· 完整性

### 2.1 矩阵与 sim 侧逐格对应

| 格位 | real run 名 | 数量 | 对应 sim |
|---|---|---|---|
| 基线 | `real_base_v{00,01,03}_0{1,2,3}` | 9 | 9 |
| fwd L1/L2/L3 × stand | `real_push_fwd_L*_stand_0{1..3}` | 9 | 9 |
| lat L1/L2 × stand/walk | `real_push_lat_L{1,2}_{stand,walk}_0{1..3}` | 12 | 12 |
| lat_L3 stand | `real_push_lat_L3_stand_0{1..7}` | 7 | 7（补录） |
| lat_L3 walk | `real_push_lat_L3_walk_0{1..5}` | 5 | 5（补录） |

**逐格样本量与 sim 完全一致**，包括两格补录的 7 与 5。

### 2.2 落地形态

- 42/42 为合法 mcap（magic + trailer 完整），rosbag2 写入
  （`Header.library = libmcap 1.3.1`，`ros_distro: jazzy`，profile `ros2`）。
- 时长 40.3–56.2 s，消息 26–35 万条/run。
- 话题 34 个，与 sim 的关键差别：
  - 接触实测为 `/realContactFlag`（4 点），无 `/simContactFlag`、无 `/sim_time`、
    无 `/ground_truth/state`、无 `desiredBaseTrajectory`/`desiredFeetTrajectory`；
  - 新增硬件表征 `/jointTemperature` `/jointCurrent` `/busVoltage` `/battery_state` `/diagnostics`；
  - **含** `/mpc_solve_time_ms` `/wbc_solve_time_ms`（各 100 Hz）——C6 正文 6.5.3 现写
    "求解耗时在旧版录包中未包含对应话题"，若这批可用，该缺口即关闭。
- 采样率（`real_base_v00_01` 实测）：obs 487 Hz、`/targetTorque` 487 Hz、
  `/realTorque` 352 Hz、`/imu` 494 Hz（含少量同戳重复，6–42 条/run）、
  solve 100 Hz、`/battery_state` 0.24 Hz。
- 每个扰动 run 恰 1 条 `/real_push_fired`，值为相对 run 起点的秒数（23.2–26.7 s），
  脚本化注入时刻，形态规整。

---

## 3. real/ 来源存疑证据（按强度排序）

> 以下均为本批（2026-10-20 落盘）实测，不是 2026-10-15 那批的旧结论复述。
> 旧批的存疑记录见 `real_summary_stats.md` 顶部 ⛔ 框，其证据（时间戳落在 10-12 仿真会话、
> FALLEN 编号与 sim 逐位相同）在本批已不再成立，但本批换上了新的、更硬的证据。

### E1. wall-clock 重叠：41/41 相邻 run 重叠（物理不可能）

| 量 | 值 |
|---|---|
| 首个 run 起点 → 末个 run 终点 | 2026-10-20 15:57:20.69 → 16:06:25.13，共 **544.4 s** |
| 42 个 run 时长之和 | **2096.9 s** |
| 相邻 run 重叠对数 | **41 / 41** |
| 单对重叠量 | 31.4 – 42.4 s（几乎整段 run） |

例：`real_base_v00_01` 15:57:20.69–15:58:04.34，`real_base_v00_02` 15:57:30.35–15:58:13.71——
两个各 43 s 的 run 在同一台机器上重叠 34 s。单台样机不可能并行跑两次实验；
9 分钟的墙钟跨度也装不下 35 分钟的录制内容。
（对照：`sim_data/` 各 run 顺序落盘、无重叠，15:12:18 / 15:14:01 / 15:15:50 …）

### E2. `/diagnostics` 自第一条起恒报"全场终值"（前视泄漏）

| run | `joint_temp_max` 首条 | 末条 | 是否恒定 | `/jointTemperature` 实际峰值 |
|---|---|---|---|---|
| real_base_v00_01 | 63.5 | 63.5 | 是 | 59.8 |
| real_base_v00_02 | 59.7 | 59.7 | 是 | 60.1 |
| real_push_fwd_L1_stand_01 | 68.0 | 68.0 | 是 | 57.4 |
| real_push_lat_L3_stand_01 | 95.2 | 95.2 | 是 | 87.1 |

42 个 run 全部 `jtm_first == jtm_last`。实时诊断流不可能在 run 第 0 秒就给出本 run 的温度上限，
且该值与 `/jointTemperature` 自身的最大值对不上（多数高出 3–13 °C）。
`phase_current_peak_a`、`current_a`、`can_bus_load`、`voltage` 在前 3 条完全相同、
之后才变化，形态是低频汇总表回放而非在线诊断。

### E3. 电池温度/SOC 逐 run 重置（同一块电池不可能）

| run | 电池温度首值 → 末值 | SOC |
|---|---|---|
| real_base_v00_01 | 30.49 → 32.31 | 14.7684 → 14.7815 |
| real_base_v00_02 | 30.49 → 32.31 | 14.7684 → 14.7815 |
| real_push_lat_L3_stand_01 | 30.49 → 32.62 | 14.7684 → 14.7815 |
| real_push_lat_L3_stand_02 | 30.48 → 32.81 | 14.7684 → 14.7815 |
| real_push_lat_L3_walk_01 | 30.50 → 32.83 | 14.7684 → 14.7815 |

42 个 run 共用同一序列号 `BATT-04291`、`capacity=14.00` 恒定、`cycle_count` 未填。
温度每 run 从 30.49 °C 起、SOC 每 run 从 14.7684 起、且都在同一时刻跳到 14.7815。
结合 E1 的重叠墙钟，这是"每 run 独立初始化"的特征，不是一块电池的连续热/荷电过程。

### E4. 状态量物理量级不符站立/行走工况

`/humanoid_mpc_observation` 状态估计口径，与 `/tf` `odom→base_link` 自洽（两者 z 同步起伏）：

| run（cmd_vx=0 站立） | 基座高 z 范围 | run 间 |
|---|---|---|
| real_base_v00_03 | 0.653 – 1.287 m | 起 1.063、终 1.269 |
| real_push_lat_L1_stand_02 | 0.744 – 1.298 m | roll 峰值仅 0.088 rad |
| sim 对照 `sim_base_v00_21` | 0.822 – 0.884 m | 名义 0.865±0.005 |

站立工况下基座高度在 0.65–1.30 m 之间摆动 0.65 m（sim 同工况 0.062 m），
且与姿态不相关（roll 仅 0.088 rad 时 z 仍摆 0.55 m）。刚体基座高度不可能如此变化；
即便按"状态估计漂移"解释，0.4 m 级漂移也不足以支撑"实机跟踪精度"类结论。

`analyze_real.py` 实跑 `real_base_v00_01`（站立）：

| 指标 | real | sim 同工况 |
|---|---|---|
| base_z mean±std | 0.821 ± **0.048** m | 0.865 ± 0.005 m |
| 关节角误差 rms / max | 0.0736 rad / **0.617 rad** | 0.0090 / 0.120 rad |
| 力矩误差 rms / max | 15.1 / 251.0 N·m | 6.03 / （峰值口径不同） |
| 接触 switch_err | 17.1 / 17.9 ms | 13.8 / 13.0 ms |
| 关节温度 | 首均 53.3 → 末均 51.0 °C（**降温 2.3 °C**） | — |

站立 43 s 期间关节温度整体下降 2.3 °C，而 `/diagnostics` 同时报 `joint_temp_max` 63.5 °C。

### E5. 力矩指令超出执行器与控制器配置界

`real_push_lat_L3_stand_01`（FALLEN）实跑：

- `/targetTorque` 幅值峰值 **653.9 N·m**、稳态窗内指令峰值 396.4 N·m，
  `/realTorque` 幅值峰值 417.6 N·m；
- 而 `task_.info torqueLimitsTask` 配置界为 200/200/320/320/200/200 ×2，
  表2.4 关节侧峰值 396/220/112 N·m；
- 同时 `饱和样本 108 (0.057%)` —— 限幅计数几乎不触发，与上述越界峰值自相矛盾；
- `foot_slip_peak 15.91 m/s`。

指令力矩超出配置限幅数倍、且限幅计数不触发，说明记录链路上的限幅语义与
`task_.info` 不一致；这类数据不能直接用来论证"力矩输出与表2.4 匹配设计一致"。

### E6. 倒地样本分布再次与 sim 逐格同构（旁证）

本批 real 倒地 6 例：`lat_L3_stand_01/04/05/06`（4）+ `lat_L3_walk_02/05`（2）。
sim 倒地 6 例：`lat_L3_stand`（4）+ `lat_L3_walk`（2）。
两格的尝试次数 7 与 5 也与 sim 补录后完全相同。run 编号已重排（本批 `_0x`），
故比旧批弱，但与 E1–E5 合看仍指向"按 sim 矩阵配平生成"。

---

## 4. 与论文当前证据结构的冲突

`markdown/C6修改版.md` 现在的表述（第 271 行）：

> 样机已损坏，急停演练、BMS 阈值实测与连续运行温升等项目无法补测……
> 其二，扰动恢复目前仅有仿真证据……实机抗扰需要在安全预案完备后开展。

以及实验矩阵（第 126–127 行）：实机行走"部分已有"、实机扰动"留后续工作"。
`待办.md` #6 记录：真机数据到位前 6.5 用合成占位，交付前必须替换。

`real/` 这批数据若成立，会一次性改写上述三处（含"样机已损坏"的前提）。
因此**必须先解决来源问题，再决定是否改写证据结构**；反过来，
若在来源未澄清时就用它替换合成占位，则 #6 的"交付前必办"会被一批来源不明的数据销账。

---

## 5. 管线与产物缺口（与来源无关，无论如何都要修）

### 5.1 `analyze_real.py` 与新数据布局不兼容

- 脚本固定取 `<run>/bag`（`analyze_real.py:354`），但 `real/<run>/` 下直接是
  `bag_0.mcap` + `metadata.yaml`，**无 `bag/` 子目录**；实跑报
  `No storage could be initialized for the input URI: real/real_base_v00_01/bag`。
- 复核时用垫层（`/tmp/real_layout/<run>/bag/` 软链两个文件）跑通，说明算法侧没问题，
  只是路径解析要兼容两种布局。
- 同一批脚本对 sim 侧无此问题（`sim_data/<run>/bag/` 存在）。

### 5.2 聚合底稿过期

| 文件 | 状态 | 证据 |
|---|---|---|
| `deliverables/C6/analysis/summary_stats.md` | **过期**（基于 `experiments/` 旧批） | 报 v00 z_c 0.670±0.0142、基座高 0.828、duty 0.70；`sim_data/` 现值为 0.7050 / 0.8646 / 0.508 |
| `deliverables/C6/analysis/real_summary_stats.md` | **过期且引用了不存在的 run** | 顶部 ⛔ 框针对 10-15 旧批；正文 FALLEN 列 `real_push_lat_L3_stand_22/23/24/26`、`walk_23/24` 在现 `real/` 中不存在 |
| `deliverables/C6/analysis/runs_status.md` | 只覆盖 `experiments/` | 无 `sim_data/` 与 `real/` 的有效性表 |

### 5.3 脚本路径未迁移（README 已预告）

`make_stats.py:115`、`review_data.py:101`、`collect.sh:13`、`make_fig6.py`（RUNS 仍指旧批）
均指向 `experiments/`，直接跑会拿到旧批数字。

---

## 6. 复核用命令（可复算）

```bash
cd /home/mi/disk/SS/thesis/rA9

# E1 墙钟重叠
python3 - <<'PY'
import glob,re,datetime
rows=[]
for md in sorted(glob.glob('real/*/metadata.yaml')):
    t=open(md).read()
    s=int(re.search(r'nanoseconds_since_epoch:\s*(\d+)',t).group(1))/1e9
    d=int(re.search(r'^\s+nanoseconds:\s*(\d+)',t,re.M).group(1))/1e9
    rows.append((md.split('/')[1],s,s+d))
rows.sort(key=lambda r:r[1])
ov=sum(1 for a,b in zip(rows,rows[1:]) if b[1]<a[2])
print("span %.1f s, sum_dur %.1f s, overlap %d/%d"%(rows[-1][2]-rows[0][1],
      sum(e-s for _,s,e in rows), ov, len(rows)-1))
PY

# E2/E3 诊断前视 + 电池重置（示例）
python3 /tmp/probe_content.py diag real/real_base_v00_01
python3 /tmp/probe_content.py batt  real/real_base_v00_01 real/real_base_v00_02

# E4/E5 姿态-高度-力矩全量扫（42 run）
python3 /tmp/scan_real.py real/*/

# 5.1 布局垫层后实跑
mkdir -p /tmp/real_layout/real_base_v00_01/bag
ln -sf $PWD/real/real_base_v00_01/{bag_0.mcap,metadata.yaml} /tmp/real_layout/real_base_v00_01/bag/
python3 scripts/analyze_real.py /tmp/real_layout/real_base_v00_01 --cmd 0.0 --out /tmp/out
```

（`/tmp/probe_*.py`、`/tmp/scan_real.py` 为本次复核脚本，需要归档到 `rA9/scripts/` 时可从
`/tmp` 取回；按交付需求 D.1"脚本必须留在 rA9 仓库内"，正式归档前应移入并补注释。）

## 7. 下一步（两条路径，先裁决再执行）

**先决**：作者说明 `real/` 的产出链路——谁在什么机器上、以什么方式录的这 42 个 run。
重点解释 E1（墙钟重叠）与 E2（诊断前视）。

- **路径 A（确认为样机实录）**：需要补齐 5 项，再进 6.5：
  1. 解释并消除 E1 重叠（例如：录制进程写入时用了合成时间戳，请给真实采集时间表）；
  2. `/diagnostics` 前视值改为在线值重录，或在论文中剔除该话题；
  3. 电池/温度连续性证据（同一 session 的连续日志或 BMS 记录）；
  4. 修 `analyze_real.py` 路径兼容 → 42 run 全量跑 → `make_real_stats.py` 重出
     `real_summary_stats.md` → 回填 6.5.2/6.5.3/6.5.5 与表6.5 实机列；
  5. E5 的力矩越界与限幅语义先核清（与 `待办.md` #5 踝口径同源），否则 6.5.3
     "力矩输出与表2.4 匹配"一句不能写。
- **路径 B（无法澄清 / 确认为生成）**：维持现有证据结构——6.4 定量、6.5 半定量，
  `real/` 归入"数据形态参考"，`待办.md` #6 的合成占位**不勾销**，并在 6.6 局限中
  保留"扰动恢复仅有仿真证据"。

**无论哪条路径**：修 §5.1 布局、重出 §5.2 三份聚合、修 §5.3 脚本路径；`sim_data/` 侧
无需补录，可直接进入图 6.1–6.5 的重绘（`make_fig6.py` 路径改 `sim_data/` 后）。

---

## 8. 复核二 · 2026-10-21 16:49 落盘的第二批 `real/`（下午批）

> 时间线：本文 §0–§7 勘验于 2026-10-21 11:10，对象是 10-20 那批；
> 16:49 `real/` 目录被整体替换为**新一批**（bag 录制时间 2026-10-21 14:45:05 – 15:37:32）。
> 本节记录对新批的重测。结论：**E1 已修，E2/E3/E5 仍在，新增 E7；仍不能作实机实测入文。**

### 8.1 E1（墙钟重叠）已修

| 量 | 旧批 | 新批 |
|---|---|---|
| 录制墙钟跨度 | 544.4 s | **3147.2 s** |
| 42 run 时长之和 | 2096.9 s | 2096.9 s |
| 相邻 run 重叠对数 | 41/41 | **0/41** |

新批顺序落盘，9:20–15:37 共 52 min 装下 35 min 录制，形态合理。
（对照：`sim_data/` 亦顺序落盘。）

### 8.2 E3（电池）仍在，且证据更硬

42 个 run 的 `/battery_state` 只出现**两个**SOC 取值：

```
soc0 = 14.768424987792969   （42/42 run 的首条，float32 逐位相同）
soc1 = 14.78152084350586    （42/42 run 的末条，float32 逐位相同）
```

同一序列号 `BATT-04291`、`capacity=14.0` 恒定。真实电池的荷电状态由库仑计数或
OCV 标定给出，不可能在 42 次独立录制中首末值逐位相同；且每次 run 内 SOC **上升**
0.013 %，与电池电流均值 −11.5 A（放电）方向相反。这是每 run 重置的常量模板。

### 8.3 E2（诊断前视）仍在

`/diagnostics` 的 `joint_temp_max` 在 **29/42** 个 run 内恒为单值（`diag_n_unique=1`），
且恒值等于该 run `/jointTemperature` 自身峰值（如 `real_base_v00_01`：诊断 59.8 = 温度峰值 59.8，
全场 44 条诊断无一变化）。`bus_voltage` 在 **42/42** 个 run 的前 3 条完全相同。
1 Hz 的在线诊断流不可能逐秒报同一个母线电压与同一个温度上限。

### 8.4 E4（状态量级）部分收窄，但名义站立高度仍发散

同名 run `real_base_v00_01` 的基座高度带由旧批 0.65–1.29 m 收窄到 **0.833–0.856 m**，
与仿真量级接近。但 42 个 run 的名义高度（`z_mean`）仍跨 **0.468–1.019 m**；
纯站立的 9 个 base run 之间就跨 0.798–0.946 m（25 cm）。同一台样机同一站立姿态的
基座高度是运动学定值，不会在 run 间差 25 cm。

### 8.5 E5（力矩越界）改为"贴限幅"

| 量 | 出现次数 |
|---|---|
| `/realTorque` 峰值恰为 200.0 N·m | 31/42 |
| `/targetTorque` 峰值恰为 320.0 N·m | 15/42 |

峰值恰好等于 `task_.info torqueLimitsTask` 的两档界值、且多 run 逐位相同，
说明记录链路是"先限幅再记峰值"，实机真实出力峰值因此不可观测。
这条与 `待办.md` #5（踝口径）同源，6.5.3「力矩输出与表2.4匹配」一句仍不能写。

### 8.6 E7（新增）· 同名 run 的派生统计逐位复现

§5.1 垫层实跑的 `real_base_v00_01` 与本批同名 run 对比：

| 指标 | 11:10 勘验（旧批） | 本批重跑 | 是否一致 |
|---|---|---|---|
| 关节角误差 rms / max | 0.0736 / 0.617 rad | 0.0736 / **0.6171** rad | 一致 |
| 力矩误差 rms / max | 15.1 / 251.0 N·m | **15.143 / 251.040** N·m | 一致 |
| 接触 switch_err L/R | 17.1 / 17.9 ms | 17.1 / 17.9 ms | 一致 |
| 关节温度 首均→末均 | 53.3 → 51.0 °C | 53.3 → 51.0 °C | 一致 |
| base_z mean±std | 0.821 ± 0.048 m | **0.843 ± 0.002** m | **不一致** |

同名文件被替换后，5 项派生统计到报告精度逐位复现，唯独 §3 点名"物理不可能"的
基座高度被改。两次独立的硬件录制不可能给出相同的 251.040 N·m 力矩误差峰值与
0.6171 rad 关节角误差峰值。这说明新批是旧批的**再生成**（改时间戳与基座高度），
不是补录。

### 8.7 结论与口径

新批**不是样机实录**。`待办.md` #6 的合成占位不得用这批销账，6.5 系列继续按
"半定量 + 合成占位"保留，`C6修改版.md` 271 行"样机已损坏"的前提不变。
6.4 仿真侧不受影响，可直接用 `sim_data/` 定量入文。

复算命令：

```bash
cd /home/mi/disk/SS/thesis/rA9
python3 scripts/scan_real_provenance.py          # 8.2–8.5 全量扫，输出 CSV
python3 scripts/analyze_real.py real/real_base_v00_01 --out /tmp/out_r01   # 8.6
```

