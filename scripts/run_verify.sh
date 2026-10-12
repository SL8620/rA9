#!/bin/bash
# 力矩语义/力矩界 —— 在线注入式验证（快速，~30s，不需控制器/步态）
#
# 原理：令 Kp=Kd=0 → data.ctrl ≡ targetTorque，于是
#   /controlInput = data.ctrl           = 注入值
#   /realTorque   = qfrc_actuator[-12:] = clip(注入值, ±LIM)
# 两者之差即钳制量。qfrc_actuator 只取决于 ctrl 截断，与机器人状态无关，
# 故无需闭环即可验证。
#
# 用法:
#   scripts/run_verify.sh              # 在线注入验证（本文件）
#   scripts/run_verify.sh chain <名>   # 全链路冒烟：跑一个闭环 run 再做 bag 级链路校验
#
# 安全约定：**绝不 pkill 通配匹配**（会误伤用户正在跑的 sim / 在录的 rosbag）。
#          清理只按本脚本记录的 PID/PGID 精确杀；发现既有进程只报告不处置。
set -u
RA9_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SCRATCH="${TMPDIR:-/tmp}/ra9_verify"
LOG="$SCRATCH/sim_node.log"
SIM_PID=""
SIM_PGID=""

MODE="${1:-inject}"

cleanup() {
  if [ -n "$SIM_PGID" ]; then
    echo "[clean] 杀本脚本启动的 pgid=$SIM_PGID"
    kill -TERM -- "-$SIM_PGID" 2>/dev/null; sleep 1
    kill -KILL -- "-$SIM_PGID" 2>/dev/null
  fi
  [ -n "$SIM_PID" ] && kill -KILL "$SIM_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

# ---------- 模式 2：全链路冒烟 ----------
if [ "$MODE" = "chain" ]; then
  NAME="${2:?用法: run_verify.sh chain <实验名>}"
  echo "[chain] 跑闭环 run '$NAME'（用 run_experiment.sh）"
  bash "$RA9_DIR/scripts/run_experiment.sh" "$NAME" trot 0.3 12 8 || {
    echo "[chain] ✗ 闭环 run 失败"; exit 1; }
  echo "[chain] 对产物做 bag 级链路校验"
  exec python3 "$RA9_DIR/scripts/verify_torque_chain.py" "$RA9_DIR/experiments/$NAME/bag"
fi

# ---------- 模式 1：在线注入验证 ----------
mkdir -p "$SCRATCH"
echo "[precheck] 既有进程检查（只报告，不处置）"
for pat in "humanoid_mujoco_sim/humanoid_sim" "cheat_controller_node" "ros2 bag record"; do
  n=$(pgrep -fc -- "$pat" 2>/dev/null || true); n=${n:-0}
  if [ "$n" -gt 0 ]; then
    echo "  ⚠️ 已有 $n 个 [$pat] 在跑:"; pgrep -fa -- "$pat" | sed 's/^/      /'
  else
    echo "  · 无 [$pat]"
  fi
done
echo "[precheck] 负载: $(cut -d' ' -f1-3 /proc/loadavg)"

set +u
source /opt/ros/jazzy/setup.bash
source "$RA9_DIR/install/setup.bash"
set -u

XML="$RA9_DIR/install/humanoid_legged_description/share/humanoid_legged_description/mjcf/humanoid_legged_control_.xml"
echo "[check] MJCF 两层力矩界（应各 4 行 320）:"
grep -c 'actuatorfrcrange="-320 320"' "$XML" | xargs echo "   actuatorfrcrange 320:"
grep -c "ctrlrange='-320 320'" "$XML" | xargs echo "   ctrlrange        320:"

echo "[start] humanoid_sim (render:=false) → $LOG"
# setsid 起独立进程组，清理精确到组，不波及同名他进程
setsid ros2 run humanoid_mujoco_sim humanoid_sim --ros-args -p render:=false \
  > "$LOG" 2>&1 &
SIM_PID=$!
SIM_PGID=$(ps -o pgid= -p "$SIM_PID" 2>/dev/null | tr -d ' ')
echo "  pid=$SIM_PID pgid=${SIM_PGID:-?}"

# /sim_time 只在非暂停时发，故用 /jointsPosVel（暂停期也发）做就绪探针
echo "[wait] 等 /jointsPosVel ..."
for i in $(seq 1 30); do
  if timeout 2 ros2 topic echo /jointsPosVel --once >/dev/null 2>&1; then
    echo "  ✓ sim 就绪 (${i}s)"; break
  fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "  ✗ sim 进程已退出，日志尾部:"; tail -25 "$LOG"; exit 1
  fi
  [ "$i" = 30 ] && { echo "  ✗ 等待超时，日志尾部:"; tail -25 "$LOG"; exit 1; }
done

echo "[run] verify_torque.py"
python3 "$RA9_DIR/scripts/verify_torque.py"
rc=$?
echo "[done] 退出码 $rc"
exit $rc
