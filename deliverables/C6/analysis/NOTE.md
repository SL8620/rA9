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
