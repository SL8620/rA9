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

## 7. 自动 run 翻倒根因（2026-10-09 定位并修复）✅

旧归因"RTF 不确定是翻倒根因"**不成立**（RTF 乱跳本身是测量伪影，见 §8；
仿真节拍一直 ~1.0）。真正的根因链有两个，均已修复并回归验证：

**根因 ①使能竞态**：脚本发 `/hwswitch true` ≠ 控制器收到。rtf_smoke_001 中
update() 从解暂停前 2.57 s 就在跑（`/wbc_solve_time_ms`、`/cmd_contactFlag`
全程在流），但 `hwSwitch_` 门控的 `/targetTorque` 直到解暂停后 +0.61 s 才出现
（DDS 发现耗时 ~2.5 s）。sim 侧 `targetKp/targetTorque` 缓存为 0 → 无力矩
自由落体，0.5 s 内倒地。manual_repro_003 走通正因人工提前 3 分钟使能。
修复：run_experiment.sh 不盲等，改为**确认 `/targetTorque` 真在流再解暂停**。

**根因②参考未重锚**（决定性）：使能修复后（rtf_smoke_002/003）仍在 1.2~1.5 s
振荡倒地。逐项对照 manual_repro_003 发现：两者观测状态完全相同（r_c/θ_b/q_j
一致、mode=3），但解暂停瞬间前馈力矩 **manual 全程 190~207 Nm（膝 −45、踝
+33 的站立支撑解）vs smoke 稳定在 38~43 Nm（踝 0.4 的弱解）**——是两个稳定
但截然不同的 MPC 解，不是收敛时间问题（等 45 s 无效，ff 无漂移）。差异在
参考轨迹：manual 解暂停前收到过 `/cmd_vel`（teleop 零速广播），
`cmdVelToTargetTrajectories` 把参考重锚到 (comHeight 0.952, defaultJointState)
的标准站立参考；smoke 只有 `starting()` 的"保持冻结状态"参考 → 弱解撑不住
62 kg。修复：解暂停前**先喂 4 s 零速 `/cmd_vel` 重锚参考**。

**回归验证**：rtf_smoke_004（原地 trot）STATUS VALID——高度 0.830±0.017、
agree 98.1%、switch_err 13.5 ms；rtf_smoke_005（cmd_vx=0.1 行走）STATUS VALID
——vx=0.099±0.021、agree 97.3%、switch_err 8~9 ms。收数流水线打通。

注：旧"PRE_UNPAUSE_WAIT 2s vs 30s 相同"的排除结论是**被根因①混淆的**（当时
怎么等都先自由落体）；45 s 实验证明 MPC 收敛时间本身确实不是因素。

## 7.5 自发故障模式：垃圾关节目标（2026-10-09 A1/扰动收采中发现）⚠️ 未根治

trot 工况 run 有 **~20-30% 概率自发倒地**（非扰动引起；A1 的 sim_base_v00_02、
v03_03 与扰动 stand_01 系列同款）。失效签名（sim_push_lat_L1_stand_01 实录）：

- 解暂停后 ~8-10 s（步态指令 +5.8~7.8 s 发出、切换后 ~2 s）控制器的
  `/targetPos` 膝目标突变至 **+9.58 rad**（正常站立 0.89），持续 ~10 个周期；
- sim 侧 PD 忠实执行：Kp=120×8.7 rad 误差 ≈ 1044 Nm 膝力矩（实测 1051 Nm 精确吻合），
  0.25 s 内毁机（roll −133°、滑出 1.2 m）；
- **MPC 线程未死**（/mpc_solve_time_ms 持续到 run 末），垃圾目标出自扰动/切换
  工况下 evaluatePolicy/WBC 的瞬态解；NaN 防线拦不住（9.58 有限）；
- 与扰动无关的证据：sim_push_lat_L1_stand_01 的扰动在倒地**之后**才触发
  （trigger_sim 15.4 ↔ 解暂停+15.4 s，倒地在 +10.3 s）。

**影响与处置**：stand(trot) 扰动工况会被自发故障污染——analyze_push.py 已加
"注入前已倒地（自发故障，非扰动数据）"判定剔除；行走工况 L1 实测抗扰成功
（Δφ_max 0.023 rad、Δy 0.046 m、未倒）。若要根治需查模式切换后 MPC/WBC 的
瞬态解（待办）；C6 文本可作为可靠性局限如实表述（6.5.4/6.6）。

**根因 ③`/pauseCmd` 同款竞态**（A1 收采中发现）：解暂停命令 `timeout 3` 单次
发布可能整批丢失（DDS 发现竞态）→ sim 全程不解暂停，run 整个没有步进数据。
指纹：`/realTorque`、`/simContactFlag`、`/sim_time` 全 0 条，而 `/imu`、
`/ground_truth/state` 各 ~18 万条（暂停分支 ~9 kHz 刷屏）。sim_base_v03_01、
v03_05、rtf_prof_001 即此死因（早期误判为"录包饿死"）。修复：解暂停循环发
并**以 /sim_time 在流为成功判据**，60 s 未生效则中止该 run。经验法则：
**脚本发的任何关键命令都要"验证生效"而不是"发完即信"**（/hwswitch、
/pauseCmd 均已验证化）。

## 8. RTF（仿真实时率）——旧"RTF 乱跳"结论是测量伪影 ✅ 已澄清

**2026-10-09 结论：仿真节拍一直是 ~1.0；旧"RTF 0.015~0.759 乱跳、控制/植物
时间基准不一致是翻倒根因"是条数法×录包丢包的测量伪影，已废除。**

证据链：
1. 内部节拍计量（perf 仪表，解暂停时复位）：修后 22000 步/22.000 s，
   **step_wall=1.000 ms、RTF=1.000**；物理仅 0.13~0.17 ms/步、发布 0.12 ms/步，
   节拍有大量余量（旧代码的 `sim_rate=1000` 步进门本来就在 1:1 节拍）。
2. 录包对 500 Hz 仿真话题**整批丢 ~20%**：`/simContactFlag`、`/jointsPosVel`、
   `/ground_truth/state`、`/imu` 四个同块发布的消息数**完全相同**（7480 条，
   应发 ~9320）——批量丢弃特征。条数法把丢包算成 RTF 亏损（0.803）。
3. **auto_ab_04"机器人冻住 RTF 0.015"实为录包饿死**：22 s 仅收 194 条
   （捕获 2%），但相邻消息间隔中位数 2.002 ms——仿真步进完全正常。

**测量规范（已固化）**：sim 新增 `/sim_time`（仿真时间秒，500 Hz 随发布块），
analyze_walk.py 以 **RTF = Δsim_time/Δwall（跨度法）** 为准（对丢包鲁棒），
条数法只作"下限估计"；同时输出**录包捕获率**（实收/应发），<50% 判
INVALID(数据不可靠)。旧 bag 无 `/sim_time`，其 RTF 标"下限估计"。

主循环修复仍保留（欠账有界追赶取代 >0.1 s 丢弃欠账；暂停期保持锚点；
render 模式关 vsync）——对渲染停顿的恢复是真实改进。

**对论文的含义**：仿真以 RTF≈1.0 实时运行，墙钟≈仿真时间，MPC T_s=0.01 的
时间基准与植物一致，sim 指标可与实机直接并列（注明 sim/real）；无需按 RTF
折算。每个 run 的 RTF 与捕获率随 summary 落盘，可复算。

## 8. 存量 run 判定汇总

见 `runs_status.md`（全部 17 个 run 的 STATUS/RTF 一览）。要点：

- **唯一 VALID**：`manual_repro_003`（RTF 0.889，四类指标齐全）。
- `walk_smoke_001` 缺 solve-time（旧 bag 未埋点）标 INVALID，但四类中
  **1–3 类（跟踪/时序/力矩）可用**，可作方法展示，不可进 fig6_4/耗时结论。
- 其余全部 INVALID（倒地 / 缺门控话题 / bag 损坏 / 仿真冻结）。
- 结论：A1 基准 9 run 必须在 RTF 修复后重录，与交付需求 A1 一致。

## 9. 数据质量评审（2026-10-10，脚本 scripts/review_data.py / review_spike.py 可复算）

**总体判定：数据可用、质量良好**（A1 九 run + A2 五十七有效样本）。
关节角误差分布 p50 0.01°、p90 0.34°、p99 1.69°；剔除瞬态窗后九 run 收敛于
RMS 0.007~0.010 rad、max 0.12~0.16 rad；速度跟踪 RMSE 0.007~0.039 m/s；
接触 agree 96.9~98.2%；实时性双 0% 超预算。

**三个已处置/待处置发现**：

1. **瞬态垃圾窗（已处置）**：/targetPos 存在 ~12 ms 跳变（单步进最大 1.57 rad，
   实际关节纹丝不动），与 §7.5 自发故障同族轻量版，每 run 0~4 次。旧"角误差
   max 1.3~1.6 rad"为假象。**报告口径**：analyze_walk.py 新增剔除指标
   （rms_total_excl/max_abs_excl + glitch_count/glitch_ms），主报剔除值；
   力矩误差 max 294~391 N·m 的峰值同源于垃圾窗 PD 反应 + 接触切换，RMS 干净。
2. **🚩 踝俯仰力矩界与硬件链不一致（待处置）**：`task_.info:237 torqueLimitsTask`
   踝两轴取 200 N·m，而 C2 表2.4 的踝俯仰模组峰值为 112 N·m（双电机协同）——
   界值超硬件 79%；实测前馈在 200.0 饱和、ctrl 峰 213 N·m（190% 硬件）。
   髋滚转界 200/硬件 220 ✓、髋俯仰·膝界 320/硬件 396 ✓ 保守；**踝滚转 C2 无
   校核行**。C2 正文"控制器设计中设置关节力矩边界与硬件选型一致"在踝轴未兑现。
   处置选项：(a) 校正 torqueLimitsTask 踝俯仰 200→112 并重录 A1/A2；
   (b) 论文 6.4.3/6.6 如实写入不一致与修正建议（现数据照用）。
3. **扰动样本损耗 28%**（10 未送达/5 自发故障/5 推力致倒[保留]/2 其他）：
   九格有效样本 3~9 个；lat_L3_walk 仅 3 个有效含 1 倒地，该格结论置信度低。

### §9-2 处置结论（2026-10-10 作者裁决）

**配置回退原始值（踝两轴 200/200），既有采集数据为正典；踝峰值扭矩口径
入待办（论文仓 `待办.md` #5，与 #4 motor-side chain 关联）。**

新证据（对照实验）：v03（0.3 m/s）行走的踝俯仰**动态需求峰值 184~200 N·m**
（旧界 200 下前馈饱和，sim_base_v03_02/06 实测）；112 界对照实验中 v03 复现
不稳定（sim_base_v03_11~13，与自发故障混杂，不能单独归因限幅）。112 口径
存疑（双电机 224 误记？静态核算 81.19 低估动态需求？）——待作者核实电机侧
参数后定论。

**数据批次划分**：
- 正典（原始界 200/200/320）：A1 `sim_base_*_0x`、A2 `sim_push_*_01~09`
  及 walk_smoke_001/manual_repro_003 —— 论文引用只用这批
- 探索批（临时界 112，`*_1x`/`*_11+`）：仅作 §9 勘验依据，不进论文数据

## 10. 采集前置检查（2026-10-10 血泪教训，已固化为流程）

**症状**：合成占位批次"六连倒"、零扰动对照也倒——一度误判为"模型失配致
±1% 质量即失稳"。**真因**：孤儿进程累积拖垮 CPU（load 65~76）→ RTF
1.000→0.84 → defaultJointState 过渡振荡（膝目标 0.9→1.8→0.55 发散）→ 一致倒地。

**泄漏源三类**（全部已回收固化进 run_experiment.sh 清场/trap）：
1. `robot_state_publisher` 不在清理名单，每 run 漏 1 个（现场 5 个×6% CPU）
2. `ros2 topic pub/echo` 的 python 孙进程（timeout 杀父不杀孙）
3. 中止路径的 recorder（trap 无 flush 收尾）

**收数前置检查（每次采集前必须过）**：
- [ ] `uptime` 1min 负载 < 5（20 核机）
- [ ] `pgrep -c "robot_state_publisher|ros2 topic|humanoid_sim"` ≈ 当前 run 数
- [ ] 冒烟 run 的 RTF（跨度法）≥ 0.98

**修复后复验**：零扰动对照 STATUS: VALID、RTF=1.000；扰动批次六连 VALID
（质量±5%/惯量±10%/质心±5mm 下行走稳定 → 控制器参数鲁棒性成立）。

## 11. 速度指令送达延迟（fig6_1 平线段解释，2026-10-12）

fig6_1 旧版前 ~7.5 s 为平线（原地踏步、无波动）——非"未使能"数据：解暂停后
trot 步态即启动（接触切换从 t=0 连续），但 `/cmd_vel` 发布者 DDS 发现耗时
~5-8 s，速度指令 7.5 s 才送达 → 机器人先原地踏步再加速趋近目标（进目标带
~9.5 s）。控制器全程在工作，采集均在使能/解暂停之后 ✓。

**图已修正**：fig6_1 改画稳态窗（剔除原地踏步段与加速过渡），图例统计与
summary/表6.x 同口径（旧图例被全程均值拖垮：v03 显示 0.067 而稳态真值
0.285）；下栏标签改"质心高度 z_c"（旧标"基座高度"系标签替换静默失败）。

## 12. 力矩界**双层钳制** + `/realTorque` 语义更正（2026-10-12，sim-to-real 前置）

### 12.1 🚩 MJCF 有两层力矩钳制，§9 的"20 Nm 落不了地"判断偏小

`humanoid_legged_control_.xml` 里力矩上限被夹**两次**：

| 层 | 位置 | 作用 | 改前 | 改后 |
|---|---|---|---|---|
| actuator 级 `ctrlrange` | `<actuator>` 各 `<motor>` | 钳制指令 ctrl | 200/200/**300/300**/200/200 | 200/200/**320/320**/200/200 |
| joint 级 `actuatorfrcrange` | `<worldbody>` 各 `<joint>` | 钳制该关节执行器**合力** | **200 全轴** | 200/200/**320/320**/200/200 |

**实际生效 = min(两层)**。改前两层不一致 → 全轴被夹在 200 Nm，膝·髋俯仰的
320 请求实有 **120 Nm** 落不了地（§9-2 记的 20 Nm 是只看 ctrlrange 的低估）。

**但影响可忽略**：正典九 run 中 >200 Nm 的样本占 **0.00~0.02%**，被削力矩占
力矩积分 **0.00~0.03%**（峰值仅出现在瞬态垃圾窗）。**故既有数据不需要重录**，
只是口径文档错了。§9-2 的裁决（保留 200/200 正典、踝 112 入待办）不变。

**处置**：两层一并对齐 `task_.info torqueLimitsTask`（200/200/320/320/200/200），
MJCF 内加注释固化"改任一层必须同步改另一层"。新旧数据差异仅在罕见截断瞬间，
符合"可比性优先"的裁决取向。

### 12.2 `/realTorque` 语义错误（更正后需重录力矩类指标）

`humanoid_sim.py` 原本把 `data.ctrl`（= 前馈+PD 的**控制输入**）发布到
`/realTorque`，而 PD 环就跑在 sim 内部 1 kHz、用无噪声无延迟真值状态。
→ 验证章指标[3]"力矩跟踪 rms 6.13 Nm"实为**PD 修正量**，不是执行器跟踪
误差；仿真里无执行器动力学可跟踪，该指标在 sim 侧是恒等式，**不能与真机
同名指标对照**。

**处置**（2026-10-12）：
- `/realTorque` → `qfrc_actuator[-12:]`（执行器实际出力，含截断效应；nv=18
  = 浮动基 6 + 12 关节，关节分量取 `[-12:]`，顺序与 ctrl 一一对应已验证）
- 新增 `/controlInput` → `data.ctrl`，承接旧语义（PD 修正诊断仍可复算）
- `scripts/analyze_walk.py` 自动识别代际（**有无 `/controlInput` 话题**）并在
  metrics.csv / summary.txt 标注 `torque_semantics`，新 bag 另出 `pd_correction_*`

**代际不可比**：`sim_base_*_0x`、`sim_push_*_0x`、walk_smoke_001、
manual_repro_003 等旧 bag 的 `/realTorque` 是旧语义；新录 bag 是新语义。
**力矩类指标需重录**，其余（基座跟踪/接触时序/求解耗时）不受影响可沿用。

### 12.3 附带核实（本次复核确认无误的项）

- `/realTorque` 仅 **29.5 Hz**（687 条/run），漏掉 30~60% 峰值 → 任何基于它的
  "力矩峰值"都被系统性低估，报告峰值须用 `/targetTorque`（477 Hz）或新
  `/realTorque` 并注明采样率。
- `/simContactFlag` 存在接触抖动：v03_06 LTOE 66 次切换中 **26 次持续 <20 ms**
  （最短 2.1 ms），规划侧 0 次 → summary 的 "switches 24/54" 差距由此而来，
  `switch_err` 最近邻配对已压小影响，但论文报 switch 数应注明是否消抖。
- `/foot_vel_estimate`（16 维）九 run **全程常量**，估计器未在算。
- 录制窗口是**限时存活段**：9 run 中 6 个在 `collect.sh` 停止录制后 2~5 s 触发
  pitch 安全限（**0 个在录制窗口内倒地**）；每档速度恰有一个干净 run
  （v00_04 / v01_03 / v03_06）。论文须写明录制时长，勿暗示可持续行走。

### 12.4 回填方案（2026-10-12 落地，替代重录）+ 发布频率修复

**结论：语义变更不需要重录。** 恒等式
`qfrc_actuator[-12:] = clip(data.ctrl, ±LIM)`
在真实闭环数据（`smoke_newsem_02`）上实测**残差 0.0000 Nm**，故旧 bag 的新语义
量可确定性重建。

**实现：分析层回填，不改写 bag。** `analyze_walk.py` 检测到无 `/controlInput`
时，取旧 `/realTorque`（= `data.ctrl`）作控制输入，按上式算执行器出力。
理由：改写 147 个大 bag 风险高且污染溯源（原数据 vs 修改后数据不可分）；
分析层回填原始数据不动，且额外可得 PD 修正量与钳制损失两个分解项。
两代 bag 的 `torque_tracking` 口径由此**统一，可跨代比较**。

**指标自动分解**（summary [3] 现同时给出三项）：
- `torque tracking` = PD 修正量 + 钳制损失（实测两项相等 ⇒ 该指标实为 PD 修正量）
- `PD 修正量(τ_ff→控制输入)` = `/controlInput − /targetTorque`
- `钳制损失(控制输入→执行器)` = `/controlInput − /realTorque`

**发布频率修复**：`/realTorque` `/controlInput` 原在外层循环按 `1/40s` 发布，
受外层 ~60Hz 墙钟节拍限制实得 **29.4Hz**，漏 37~59% 力矩峰值。已并入 500Hz
发布块（与 `/sim_time` `/imu` 同源），实测 **438Hz**（500×录包捕获率 88%）。

| bag | /controlInput | 平均漏峰 | 最大漏峰 |
|---|---|---|---|
| smoke_newsem_02（改前） | 29.4 Hz | **54%** | 93% |
| smoke_newsem_03（改后） | **438 Hz** | **−2%** | 9% |

（−2% 是 PD 使 `/controlInput` 峰值可略超 `/targetTorque`，正常。）

**待办：重录推迟到执行器链路建模之后一次到位**（加延迟/力矩包络/摩擦会改变
行走行为，现在重录可能白做）。届时 9 个基线 run 的力矩峰值指标即可干净取用。

**验证脚本**（可复跑）：
- `scripts/verify_torque.py` + `scripts/run_verify.sh` —— 在线注入式验证，
  令 Kp=Kd=0 使 `ctrl ≡ targetTorque`，直接量钳制。六段全过，含正负向对称
  与全轴打满（实测生效上限 200/200/320/320/200/200）。
- `scripts/verify_torque_chain.py <bag>` —— bag 级链路校验（恒等式/钳制统计/
  代际识别）。`scripts/run_verify.sh chain <名>` = 跑闭环 + 自动校验。

**冒烟产物**：`experiments/smoke_newsem_01~03`（01 未录 `/controlInput`，系
`run_experiment.sh` 录包话题表写死所致，已补；02 改频率前；03 改后）。

## 13. sim2real gap 后处理流水线（2026-10-12）

### 13.1 方案与边界

**架构**：不把执行器动力学放进仿真回路（会改行走、有失稳风险）。保持"仿真能走、
数据照常采"，用 **sim2real gap 后处理**把非理想性叠加到录包上，产出**合成**数据。
这是 `scripts/make_synth_real.py` 的既有模式，本轮扩展到延迟链路与力矩-转速包络，
并新增 `scripts/gap_sensitivity.py` 做逐 gap 灵敏度分析。

**⚠️ 开环近似（论文 6.6 局限性必须写）**：后处理只叠加**信号特性**，轨迹仍是理想
执行器跑出来的。产物 = "理想执行器轨迹 + 非理想信号特性"，**不是**"真实执行器下
会产生的轨迹"。指令延迟只移位信号、不改变运动（闭环反馈效应缺失）；摩擦/背隙
只能作为力矩损失叠加，不能反馈回运动。

**⚠️ 合成标记纪律**（`deliverables/C6/README.md` 总原则）：产物一律 `synth_` 前缀
+ `SYNTHETIC.txt` + `params.txt: synthetic=true`，**禁止在论文中呈现为真机实测**。
用途限定：灵敏度分析 / A3 实机补录占位与流水线验证 / 图表占位。

**参数来源**：全部按器件手册典型量级**估计**，`params.txt` 逐项记 `param_sources`
（`[估计]`/`[手册]`/`[实测]`），真机参数到位后可替换。不是"编造数据"——估的是
仿真/后处理模型的参数，报出的指标仍全部由 `analyze_walk.py` 从录包复算。

### 13.2 新增的两个 gap（`make_synth_real.py`）

| gap | 模型 | 作用话题 |
|---|---|---|
| `delay_cmd_ms` | 执行侧指令延迟 | `/target{Torque,Pos,Vel,Kp,Kd}` `/controlInput` |
| `delay_obs_ms` | 传感侧延迟 | `/jointsPosVel` `/imu` `/ground_truth/state` `/realTorque` `/simContactFlag` |
| `torque_env` | τ_max(ω)=min(τ_rated, τ_stall·(1−\|ω\|/ω_nl)) | `/realTorque` |

**接触标志的延迟口径**：`/simContactFlag` 是足底接触的**测量**（真机有开关去抖
+ 传感延迟）故走 `delay_obs`；`/cmd_contactFlag` 是**规划**（不经传感）故不延迟。
两者之差正是 `switch_err` 要度量的。实测（`delay_obs=3ms` 单开）：
`switch_err 5.47 → 7.92 ms`、`agree 96.97% → 96.47%`，`/cmd_contactFlag`
逐字节未变 ✓。

**延迟实现口径**：按**时间域最近邻**移位，不按样本数移位（录包时间戳突发投递，
§12.3），不插值（避免造数据）。等效延迟误差上界 = 局部采样间隔的一半（实测
均匀 1 kHz 时 0 µs、突发形态 50~200 µs）。前 τ 秒无源数据 → 丢弃并计数进 params。
`/controlInput` 走 cmd（执行器输入）、`/realTorque` 走 obs（执行器输出的**测量**）。

**包络实现口径**：ω 取 `/jointsPosVel` 12:24 列，按时间域最近邻对齐到力矩时间戳；
与既有电压跌落项取 min。

### 13.3 单 gap 隔离验证（脚本可复算）

| 验证 | 结果 |
|---|---|
| `delay_*`：7 话题输出载荷 == 源[t−τ] | **100.00% 逐字节一致** ✓ |
| `delay_*`：同索引重合率 | 0.4~2.7%（证明确实移位；`/targetKp` 100% 因 Kp 大段恒定） |
| `torque_env`：不碰 `/targetTorque` `/jointsPosVel` `/ground_truth/state` | 100.00% 未变 ✓ |
| `torque_env`：`/realTorque` 被改变且不超界 | 12/686 样本，超界 0 ✓ |

### 13.4 关键发现：力矩-转速包络在行走速度下几乎不咬合

在 `sim_base_v03_02`（0.3 m/s trot）全量 `/targetTorque`（486 Hz）上：

| ω_nl [rad/s] | 咬合阈值 | 被剪样本 | 被剪力矩积分 | 最大剪切 |
|---|---|---|---|---|
| 5.0 | 1.29 rad/s | 0.24% | 0.56% | 178 N·m |
| **8.0**（取值） | 2.07 rad/s | **0.12%** | **0.24%** | 98 N·m |
| 12.0 | 3.11 rad/s | 0.07% | 0.13% | 40 N·m |
| 25.0 | 6.48 rad/s | 0.00% | 0.00% | 15 N·m |

**ω_nl 取 8.0 rad/s** 的依据：由膝峰值 320 N·m 反推减速比 i≈40~80，电机空载
3000 rpm(314 rad/s) → 关节侧 ω_nl≈4~8 rad/s，取上界。

**结论一**：行走速度下（各关节 |ω| p99 < 3.8 rad/s），力矩-转速包络不是主要
sim2real gap；**力矩幅值上限**才是。被剪样本集中在瞬态垃圾窗（§7.5），非正常摆动。

**结论二（更重要）**：仿真实测关节转速 **膝 10.4 rad/s、踝 22~24 rad/s，已超过
合理减速关节的空载转速（ω_nl≈4~8 rad/s）**。即：理想执行器跑出了真机做不到的
速度。这本身是 sim-to-real gap 的直接证据，比包络的数值影响更有论文价值。

**结论三**：旧 bag 的 `/realTorque` 只有 29.4 Hz（§12.4），**恰好漏掉包络咬合的
瞬时**——故单 gap 隔离下 `/realTorque` 仅 12/686 样本被改变。用 500 Hz 新数据
（`smoke_newsem_03`）或 `/targetTorque` 才能看全。

### 13.5 灵敏度表与解读

全表见 `deliverables/C6/analysis/gap_sensitivity_sim_base_v03_02.md`
（`scripts/gap_sensitivity.py` 逐 gap 单开复算，源 run `sim_base_v03_02`；
各变体 bag 在 `experiments/gap_sens_sim_base_v03_02/`，按 README「只留小体积
结果」的规矩不进 deliverables）。

**⚠️ 最重要的结论：验证章的稳态 RMS 指标对 sim2real gap 基本不敏感。**

| gap | 力矩 RMS Δ | 力矩 max Δ | 关节角 RMS Δ | 解释 |
|---|---|---|---|---|
| `hyst_lag` | +0.075 | **−4.18** | 0 | 一阶滞后削峰，唯一明显改变峰值的项 |
| `torque_env` | +0.039 | 0 | 0 | 仅 12/686 样本被剪 |
| `delay_cmd` | −0.060 | 0 | **+0.00037** | 目标延迟 → 跟踪误差上升（唯一影响关节跟踪的项） |
| `delay_obs` | +0.002 | 0 | +0.00034 | 测量延迟 → 同上 |
| `backlash` | 0 | 0 | +0.00044 | 只影响关节角，不影响力矩 |
| `voltage` | 0 | 0 | 0 | 仅 1/686 样本（158.8→150.0） |
| `encoder`/`imu`/`est`/`solve_jitter` | 0 | 0 | ~0 | 噪声量级远小于信号 |

**原因（物理上合理，非实现缺陷）**：行走工况下执行器**远未用满**——
`/realTorque` RMS ≈ 25 N·m，而饱和水平 ≈150 N·m、力矩界 200~320 N·m。
非理想性只在**峰值与瞬态**上显现，稳态窗 RMS 平均掉了。

**对论文的意义（6.6 局限性）**：
1. 验证章的 RMS 类指标**不能**用来区分仿真与实机 —— 它们对 gap 不敏感，
   仿真值与实机值接近**不代表**保真度高。
2. 要检测 sim2real gap，必须看**峰值/瞬态类**指标：力矩峰值、接触切换瞬态、
   高频段误差、饱和持续时间。
3. 旧 bag 的 `/realTorque` 29.4 Hz 采样（§12.4）恰好漏掉这些瞬态 → 放大了
   上表的"不敏感"。**重录后（500 Hz）本表应重算**，峰值类差异会显现。

**已解决（2026-10-12）**：`/simContactFlag` 已纳入 `delay_obs`。它是测量（真机
有开关去抖+传感延迟），`/cmd_contactFlag` 是规划不延迟。纳入后 `switch_err`
对延迟开始响应（5.47 → 7.92 ms @3ms 延迟），上表 `switch_err` 行需按新口径重算
（`gap_sensitivity.py` 可复算）。

### 13.6 复算入口

```bash
# 单 gap 隔离验证
python3 scripts/make_synth_real.py experiments/sim_base_v03_02 <out> --gaps delay_cmd,delay_obs
python3 scripts/make_synth_real.py experiments/sim_base_v03_02 <out> --gaps torque_env

# 全 gap 灵敏度表
python3 scripts/gap_sensitivity.py experiments/sim_base_v03_02 --cmd 0.3 --seed 42
```

gap 注册表（10 项，`--gaps`/`--no-gaps` 逐项开关）：`delay_cmd` `delay_obs`
`torque_env` `hyst_lag` `backlash` `encoder` `imu` `est` `voltage` `solve_jitter`。
