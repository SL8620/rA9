# C6 数据分析遗留问题归因（B2 口径核查 / 异常归因）

对应 `markdown/C6交付需求.md` B2 的三个待决项与若干异常。所有结论可由
`experiments/<run>/` 原始包 + `scripts/analyze_walk.py` 复算。

## 1. `/cmd_vel` 速度口径核查（关键待决项）✅

**结论：`/cmd_vel` 的值就是 m/s，无任何缩放。**

- 源码依据：`TargetTrajectoriesPublisher.h` 直接 `cmdVel[0] = msg->linear.x`，
  不乘系数；`reference_.info` 中的 `0.7` 只用于 `estimateTimeToTarget` 的
  位姿目标路径，不作用于速度指令。
- 旧注释"缩放前的归一化指令"是**错的**，已废除。
- 推论：`auto_v8` 在 cmd_vx=0.1 下 vx≈0（实测 0.000±0.000 m/s）**不是口径问题，
  是真没推进**（该 run 已倒地，见 §3）。

## 2. `auto_v4`/`auto_v5`/`auto_v6`（含 auto_v3、auto_smoke_test）指标 nan 根因 ✅

**根因：使能从未生效 → 门控话题全程 0 条 → 旧脚本按序号对齐出 nan。
不是发散、不是落盘损坏；数据从未产生，不可重算，按 INVALID 处理。**

证据链：
1. bag 元数据里 `/targetPos`、`/targetTorque`、`/targetKp/Kd`、`/cmd_contactFlag`
   等 8 个话题 **message_count = 0**（订阅在，消息从未发布）。
2. 发布侧 `humanoidController.cpp:271-286`：这些话题在 `if (hwSwitch_)` 内发布。
3. 历史根因（见 `项目记忆.md`）：teleop 以精确 150 Hz 广播 `/hwswitch=false`，
   淹没脚本 `-r 2` 的 `true` → `hwSwitch_` 恒 false → 门控话题静默、力矩不下发
   → 自由落体。auto_v2~v8 就是这么翻的；`run_experiment.sh` 现已 `teleop:=false`，
   此问题不复存在。

处置：标 `INVALID(缺话题 ...)`；四个补跑 run 的包时间跨度仅 ~14 s 且无门控话题，
没有可修复的数据。

## 3. `auto_v8` 高度异常（0.711±0.160 m）归因 ✅

**归因：倒地过程被计入统计窗。**

- 旧指标 0.711±0.160 m = 站立段 ~0.82 m 与倒地后侧躺段 0.477 m 之间的过渡混合。
- 侧躺证据：`/simContactFlag` 84% 为 `(1,0,1,0)`（左脚贴地右脚朝上）；
  新脚本稳态窗落在倒地后 [13.10, 26.14] s，高度 0.477±0.000 m（恒定）。
- 该 run cmd_vx=0.1 但 vx=0.000±0.000，"未推进"是倒地的结果而非口径问题（§1）。

**配套脚本修复**：验收发现 `analyze_walk.py` 的 STATUS 判定缺"倒地"条件
（交付需求 B2 明确"若倒地则标 INVALID"），auto_v8 曾被误标 VALID。已补：
姿态判据与控制器安全检查同源（|roll|/|pitch| > 45°），高度 < 0.75×基准
（解暂停后前 2 s 中位数）作补充，超 5% 样本即 INVALID(倒地/姿态超限)；
另有绝对下限 height_mean < 0.70 m。

## 4. `auto_v2` bag 损坏 ✅

`ros2 bag` 读取报 `read failed`，无法解析，脚本现标
`INVALID(bag 无法读取: ...)`。无数据可用。

## 5. ⚠️ z 高度口径分歧（需论文侧决定，非数据问题）

- 分析脚本测的 z 是 `/ground_truth/state` 的 **基座高度**（Odometry pz）。
- 论文 6.3.1 / 图 6.1 规格写的是 **z_c 质心高度**（comHeight 0.952 是质心口径，
  与基座高度 0.82~0.86 不是同一量）。
- 二者只能取其一：要么论文改称"基座高度 h_b"（脚本现成），
  要么从 MPC 观测/状态轨迹里提取质心高度（需扩展脚本，C6 写作前定）。

## 6. 实机/无指令 bag 的 cmd 不可考

`walk_smoke_001`、`repro_manual_001` 均未录 `/cmd_vel`（操作端指令不在包内），
指令值只能沿用操作记录。walk_smoke_001 按 cmd=0.3 分析（与既往一致，实测
vx≈0，未推进）；`repro_manual_001` 无任何记载，按 cmd=0.0 分析，
**其数字在获得操作记录前不作为论文引用来源**。

## 7. RTF 根因（数据有效性的总闸门）⚠️ 未修

仿真主循环 `mujoco_base.py simulate()` 每轮推进 1/60 s 仿真时间后无任何 sleep，
RTF 无节拍控制、逐次随机（auto_ab_01~05 单变量对照闭合），而控制器按墙钟跑
——控制回路与被控对象时间基准不一致，是自动 run 翻倒的根因。详见
`项目记忆.md`「排障经验」首条。**修复前不得再收任何仿真数据。**

**RTF 口径**：脚本以 `/simContactFlag` 测量（每 1/500 s 仿真时间一条，bag 时间戳
是墙钟），分母取该话题的墙钟跨度 = **非暂停执行段**；旧记录里 manual_repro_003
的 0.542 是含 39% 暂停段摊薄的全程值（0.889×(1−0.391)≈0.54，两口径不矛盾）。
存量 run 执行段 RTF 实测 0.018（auto_ab_04 冻结）~0.908，**无一等于 1.0**；
翻倒 run 集中在 ≤0.85，唯一走通的 manual_repro_003 为 0.889。结论不变：
时间基准不一致使结果不可复现，节拍修复后重录的数据才可定量引用。

## 8. 存量 run 判定汇总

见 `runs_status.md`（全部 17 个 run 的 STATUS/RTF 一览）。要点：

- **唯一 VALID**：`manual_repro_003`（RTF 0.889，四类指标齐全）。
- `walk_smoke_001` 缺 solve-time（旧 bag 未埋点）标 INVALID，但四类中
  **1–3 类（跟踪/时序/力矩）可用**，可作方法展示，不可进 fig6_4/耗时结论。
- 其余全部 INVALID（倒地 / 缺门控话题 / bag 损坏 / 仿真冻结）。
- 结论：A1 基准 9 run 必须在 RTF 修复后重录，与交付需求 A1 一致。
