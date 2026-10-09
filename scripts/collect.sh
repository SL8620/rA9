#!/bin/bash
# 半自动收数：人工启动闭环并走起来之后,用本脚本录制+分析当前会话。
# 不启动/不控制任何东西 —— 使能、暂停、步态、速度都由人工在各自终端操作。
# 用法: collect.sh <实验名> [cmd_vx=0.0] [录包秒数=15]
#   例: collect.sh matrix_trot_vx03 0.3 15
# 产物: experiments/<实验名>/{bag/, analysis/, params.txt}
set -u
NAME="${1:?用法: collect.sh <实验名> [cmd_vx=0.0] [录包秒数=15]}"
CMD_VX="${2:-0.0}"
REC_SECS="${3:-15}"

RA9_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$RA9_DIR/experiments/$NAME"
mkdir -p "$OUT"
[ -e "$OUT/bag" ] && { echo "[collect] ERROR: $OUT/bag 已存在,换个实验名"; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
source "$RA9_DIR/install/setup.bash"
set -u

# 确认闭环活着
if ! timeout 5 ros2 topic echo /ground_truth/state --once >/dev/null 2>&1; then
  echo "[collect] ERROR: /ground_truth/state 无数据 —— 请先启动闭环(load_cheat_controller.launch.py)并解除暂停"
  exit 1
fi

echo "[collect] recording ${REC_SECS}s -> $OUT/bag (请保持行走状态)"
ros2 bag record -o "$OUT/bag" \
  /humanoid_mpc_observation /humanoid_mpc_mode_schedule /humanoid_gait_mode_schedule \
  /humanoid/optimizedStateTrajectory /humanoid/desiredBaseTrajectory \
  /humanoid/desiredFeetTrajectory/LTOE /humanoid/desiredFeetTrajectory/RTOE \
  /humanoid/desiredFeetTrajectory/LHEEL /humanoid/desiredFeetTrajectory/RHEEL \
  /ground_truth/state /jointsPosVel /imu /pauseFlag /simContactFlag /cmd_contactFlag \
  /targetTorque /targetPos /targetVel /targetKp /targetKd /realTorque /foot_vel_estimate \
  /mpc_solve_time_ms /wbc_solve_time_ms /cmd_vel /hwswitch /pauseCmd \
  > "$OUT/bag_record.log" 2>&1 &
BAG_PID=$!

# SIGTERM 并等录包进程写完 metadata.yaml（后台任务 SIGINT 被 POSIX 忽略）
sleep "$REC_SECS"
kill -TERM $BAG_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $BAG_PID 2>/dev/null || break; sleep 1; done
kill -9 $BAG_PID 2>/dev/null

cat > "$OUT/params.txt" <<EOF
name=$NAME
mode=manual
cmd_vx=$CMD_VX
record_s=$REC_SECS
date=$(date -Iseconds)
git_commit=$(git -C "$RA9_DIR" rev-parse HEAD)
EOF

python3 "$RA9_DIR/scripts/analyze_walk.py" "$OUT/bag" --cmd "$CMD_VX" --out "$OUT/analysis"
echo "[collect] done -> $OUT"
