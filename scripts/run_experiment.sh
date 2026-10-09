#!/bin/bash
# 一键自动化实验：启动闭环 → 使能输出 → 解除暂停 → 发步态/速度 → 录 rosbag → 收尾
# 用法: run_experiment.sh <实验名> [步态] [cmd_vx] [时长s] [录包秒数]
#   例: run_experiment.sh trot_vx03 trot 0.3 20 15
# 产物: experiments/<实验名>/{bag/, launch.log, params.txt}
set -u
NAME="${1:?用法: run_experiment.sh <实验名> [步态=trot] [cmd_vx=0.3] [时长s=20] [录包秒数=15]}"
GAIT="${2:-trot}"
CMD_VX="${3:-0.3}"
DURATION="${4:-20}"
REC_SECS="${5:-15}"

RA9_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$RA9_DIR/experiments/$NAME"
# 防覆盖护栏（2026-10-09 教训：同名补录会让 rosbag 拒写而 params.txt 被新 run
# 覆盖 → bag 与 params 出处不一致，provenance 作废）。已有 bag 的目录一律拒绝。
if [ -e "$OUT/bag" ]; then
  echo "[run] ERROR: $OUT/bag 已存在，拒绝覆盖实验数据。换名重录（如 _05）或先归档。"
  exit 1
fi
mkdir -p "$OUT"

# ROS setup 脚本引用未绑定变量，与 set -u 不兼容，source 期间放开
set +u
source /opt/ros/jazzy/setup.bash
source "$RA9_DIR/install/setup.bash"
set -u

# 1) 启动闭环（仿真默认暂停、hwswitch 默认关 —— 都由本脚本控制）
#    launch 标志默认 teleop:=false rviz:=false render:=false（批量实验最优）。
#    允许用环境变量覆盖，用于排查自动化翻倒与手动成功的差异。
LAUNCH_TELEOP="${LAUNCH_TELEOP:-false}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
LAUNCH_RENDER="${LAUNCH_RENDER:-false}"
echo "[run] launch flags: teleop:=$LAUNCH_TELEOP rviz:=$LAUNCH_RVIZ render:=$LAUNCH_RENDER"
ros2 launch humanoid_controllers load_cheat_controller.launch.py \
  teleop:=$LAUNCH_TELEOP rviz:=$LAUNCH_RVIZ render:=$LAUNCH_RENDER > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null; sleep 2; pkill -f "cheat_controller_node" 2>/dev/null; pkill -f "humanoid_mujoco_sim/humanoid_sim" 2>/dev/null; pkill -f "humanoid_target_trajectories_publisher" 2>/dev/null; pkill -f "humanoid_gait_command" 2>/dev/null; pkill -f "humanoid_mujoco_sim/teleop" 2>/dev/null; pkill -f rviz2 2>/dev/null' EXIT

# 2) 等控制器就绪（MPC 观测流出现 = 初始策略已收到）
echo "[run] waiting for controller..."
for i in $(seq 1 60); do
  if timeout 3 ros2 topic echo /humanoid_mpc_observation --once >/dev/null 2>&1; then
    echo "[run] controller ready (${i}s)"; break
  fi
  [ "$i" = 60 ] && { echo "[run] ERROR: controller never came up"; exit 1; }
  sleep 1
done

# 2.4) 录包从使能/解暂停之前就开始 —— 早期版本在动作后 3s 才录，
#    反复错过跌倒关键窗口（解暂停后 1.5-3s 内发散），无法归因。
ros2 bag record -o "$OUT/bag" \
  /humanoid_mpc_observation /humanoid_mpc_mode_schedule /humanoid_gait_mode_schedule \
  /humanoid/optimizedStateTrajectory /humanoid/desiredBaseTrajectory \
  /humanoid/desiredFeetTrajectory/LTOE /humanoid/desiredFeetTrajectory/RTOE \
  /humanoid/desiredFeetTrajectory/LHEEL /humanoid/desiredFeetTrajectory/RHEEL \
  /ground_truth/state /jointsPosVel /imu /pauseFlag /simContactFlag /cmd_contactFlag \
  /targetTorque /targetPos /targetVel /targetKp /targetKd /realTorque /foot_vel_estimate \
  /mpc_solve_time_ms /wbc_solve_time_ms /cmd_vel /hwswitch /pauseCmd /sim_time \
  > "$OUT/bag_record.log" 2>&1 &
BAG_PID=$!

# 2.5) 杀掉本栈的 teleop（实验用 /cmd_vel 直发，teleop 的 150Hz 广播必须消失）
pkill -9 -f "humanoid_mujoco_sim/teleop" 2>/dev/null
sleep 0.5

# 3) 使能输出（否则力矩/目标指令被 hwSwitch 门控不发）。
#    注意 DDS 发现竞态：--once 单发可能赶在订阅端发现前丢失，
#    关键命令一律重复发布数秒（run_experiment 早期版本 3 连发全部丢失）。
ros2 topic pub -r 2 /hwswitch std_msgs/msg/Bool "data: true" >/dev/null 2>&1 &
HWSW_PID=$!
# 使能确认（关键，2026-10-09）：/hwswitch 发出 ≠ 控制器收到。实测 rtf_smoke_001
# 中控制器的订阅晚 ~2.5s 才建立（DDS 发现竞态），解暂停瞬间 /targetTorque 尚未
# 流出 → sim 侧 targetKp/targetTorque 缓存为 0 → 无力矩自由落体，0.5s 内倒地。
# manual_repro_003 走通正是因为人工提前 3 分钟使能、力矩早已在流。
# 故此处不盲等，改为确认 /targetTorque 真的在流（hwSwitch_ 门控已生效）再继续。
echo "[run] waiting for enable to take effect (/targetTorque streaming)..."
ENABLED=0
for i in $(seq 1 60); do
  if timeout 2 ros2 topic echo /targetTorque --once >/dev/null 2>&1; then
    ENABLED=1; echo "[run] enable confirmed after ${i} checks"; break
  fi
done
if [ "$ENABLED" != "1" ]; then
  echo "[run] ERROR: 使能 120s 内未生效（/targetTorque 未流出），中止（否则必翻）"
  exit 1
fi
# 使能→解暂停的等待：仿真暂停时控制器仍以 500Hz 跑、MPC 在冻结时刻反复求解。
PRE_UNPAUSE_WAIT="${PRE_UNPAUSE_WAIT:-2}"

# 3.5) 解暂停前"喂"零速指令，把 MPC 参考重锚到标准站立参考（关键，2026-10-09）。
#    cmdVelToTargetTrajectories（TargetTrajectoriesPublisher.cpp）会把参考设为
#    (r_c 速度=cmd, 基座高度=comHeight 0.952, 关节=defaultJointState)；若参考只有
#    starting() 的"保持冻结状态"，MPC 会收敛到 sum|τ_ff|≈42 Nm 的弱解（撑不住
#    62 kg），解暂停即坠。manual_repro_003 走通正因 teleop 零速广播重锚过参考
#    （实测其 τ_ff 全程 190~207 Nm 的站立支撑解）。等待 45s 无效——解是稳定的，
#    不是收敛时间问题。
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep "$PRE_UNPAUSE_WAIT"

# 4) 解除仿真暂停（替代手动按 SPACE）——必须**验证**，不能盲发（2026-10-09）。
#    /pauseCmd 同样有 DDS 发现竞态：timeout 3 单次发布可能整批丢失 → sim 永不解暂停
#    （指纹：/realTorque /simContactFlag /sim_time 全 0 条，而 /imu /ground_truth
#    在暂停分支 9kHz 刷屏）。v03_01/v03_05/rtf_prof_001 即此死因。
echo "[run] unpausing (verified via /sim_time)..."
UNPAUSED=0
for i in $(seq 1 30); do
  timeout 2 ros2 topic pub -r 5 /pauseCmd std_msgs/msg/Bool "data: false" >/dev/null 2>&1
  if timeout 2 ros2 topic echo /sim_time --once >/dev/null 2>&1; then
    UNPAUSED=1; echo "[run] sim unpaused (attempt $i)"; break
  fi
done
if [ "$UNPAUSED" != "1" ]; then
  echo "[run] ERROR: 解暂停 60s 未生效（/sim_time 未流出），中止（否则整 run 无步进数据）"
  exit 1
fi

# 5) 发步态（ModeSchedule 与 gait 字符串双通道，复刻键盘节点行为；
#    模板值取自 humanoid_interface/config/command/gait_.info，模式编码 LCONTACT=1 RCONTACT=2 STANCE=3）
case "$GAIT" in
  stance)      SEQ="[3]";                TIMES="[0.0, 1000.0]" ;;
  trot)        SEQ="[1, 2]";             TIMES="[0.0, 0.45, 0.9]" ;;
  walk)        SEQ="[1, 3, 2, 3]";       TIMES="[0.0, 0.45, 0.6, 1.05, 1.2]" ;;
  quick_trot)  SEQ="[1, 2]";             TIMES="[0.0, 0.3, 0.6]" ;;
  *) echo "[run] ERROR: 未知步态 '$GAIT'（可选 stance/trot/walk/quick_trot）"; exit 1 ;;
esac
timeout 3 ros2 topic pub -r 2 /humanoid_gait_mode_schedule std_msgs/msg/String "{data: '$GAIT'}" >/dev/null 2>&1
timeout 3 ros2 topic pub -r 2 /humanoid_mpc_mode_schedule ocs2_msgs/msg/ModeSchedule "{event_times: $TIMES, mode_sequence: $SEQ}" >/dev/null 2>&1
echo "[run] gait=$GAIT"

# 6) 发速度指令。
#    /cmd_vel 单位就是 m/s，TargetTrajectoriesPublisher 直接把它当目标速度
#    （cmdVel[0]=msg->linear.x，无缩放；reference_.info 的 0.7 只用于位姿目标路径）。
#    CMD_VEL_MODE 控制发送方式（默认 continuous，保持历史行为）：
#      continuous - 全程 -r 20 连续发
#      burst      - 只发 2 s 就停（用于验证「连续覆盖参考」是否为翻倒根因）
#      none       - 完全不发
CMD_VEL_MODE="${CMD_VEL_MODE:-continuous}"
CMDVEL_PID=""
case "$CMD_VEL_MODE" in
  continuous) CMD_VEL_DUR="$DURATION" ;;
  burst)      CMD_VEL_DUR=2 ;;
  none)       CMD_VEL_DUR=0 ;;
  *) echo "[run] ERROR: 未知 CMD_VEL_MODE '$CMD_VEL_MODE'（continuous/burst/none）"; exit 1 ;;
esac
if [ "$CMD_VEL_DUR" -gt 0 ]; then
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $CMD_VX, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
  CMDVEL_PID=$!
  if [ "$CMD_VEL_MODE" = "burst" ]; then
    ( sleep "$CMD_VEL_DUR"; kill "$CMDVEL_PID" 2>/dev/null ) &
  fi
fi
echo "[run] cmd_vx=$CMD_VX mode=$CMD_VEL_MODE for ${DURATION}s"

# 7) 保持实验窗口（录包已在 2.4 启动）
sleep "$REC_SECS"
# 收尾必须等录包进程写完 metadata.yaml 否则 bag 读不了。
# 注意：非交互 shell 的后台任务 SIGINT 被忽略（POSIX），必须用 SIGTERM。
kill -TERM $BAG_PID 2>/dev/null
for i in $(seq 1 20); do
  kill -0 $BAG_PID 2>/dev/null || break
  sleep 1
done
kill -9 $BAG_PID 2>/dev/null
[ -n "$CMDVEL_PID" ] && kill $CMDVEL_PID 2>/dev/null

# 8) 收尾：停输出、恢复暂停（先杀持续使能的发布者再关）
kill $HWSW_PID 2>/dev/null
timeout 2 ros2 topic pub -r 2 /hwswitch std_msgs/msg/Bool "data: false" >/dev/null 2>&1
timeout 2 ros2 topic pub -r 2 /pauseCmd std_msgs/msg/Bool "data: true" >/dev/null 2>&1

# 9) 记录实验参数（可溯源）
cat > "$OUT/params.txt" <<EOF
name=$NAME
gait=$GAIT
cmd_vx=$CMD_VX
cmd_vel_mode=$CMD_VEL_MODE
pre_unpause_wait_s=$PRE_UNPAUSE_WAIT
launch_teleop=$LAUNCH_TELEOP
launch_rviz=$LAUNCH_RVIZ
launch_render=$LAUNCH_RENDER
duration_s=$DURATION
record_s=$REC_SECS
date=$(date -Iseconds)
git_commit=$(git -C "$RA9_DIR" rev-parse HEAD)
git_dirty=$(git -C "$RA9_DIR" status --porcelain | wc -l)
EOF
echo "[run] done -> $OUT"
grep -A4 "Benchmarking" "$OUT/launch.log" | tail -8
