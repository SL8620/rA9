#!/bin/bash
# 人工启动 + 半自动矩阵:每个格子由人在终端里切换步态/速度并走起来,
# 脚本按回车后自动录包+分析,最后汇总。
# 前置: 已手动启动闭环(load_cheat_controller.launch.py),已解除暂停、已使能输出。
# 用法: run_matrix_manual.sh [录包秒数=15]
set -u
REC_SECS="${1:-15}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RA9_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

declare -a NAMES GAIT VXS
i=0
for gait in trot walk; do
  for vx in 0.0 0.3 0.5; do
    tag=$(echo "$vx" | tr -d '.')
    NAMES[$i]="matrix_${gait}_vx${tag}"
    GAIT[$i]="$gait"
    VXS[$i]="$vx"
    i=$((i+1))
  done
done

for ((k=0; k<i; k++)); do
  echo ""
  echo "===== 格子 $((k+1))/$i: ${NAMES[$k]}  (步态=${GAIT[$k]}  速度=${VXS[$k]}) ====="
  read -r -p "请在终端把步态切到 '${GAIT[$k]}'、速度设为 ${VXS[$k]},等机器人走稳后按回车开始录包... "
  bash "$SCRIPT_DIR/collect.sh" "${NAMES[$k]}" "${VXS[$k]}" "$REC_SECS"
  sleep 2
done

SUM="$RA9_DIR/experiments/matrix_summary.txt"
{
  echo "manual matrix $(date -Iseconds) | git $(git -C "$RA9_DIR" rev-parse --short HEAD)"
  printf "%-22s %-8s %-14s %-10s %-20s %s\n" "exp" "cmd_vx" "vx_mean" "agree%" "MPC ms(p95/max)" "WBC ms(p95/max)"
  for ((k=0; k<i; k++)); do
    f="$RA9_DIR/experiments/${NAMES[$k]}/analysis/metrics.csv"
    [ -f "$f" ] || continue
    vxm=$(grep "base_tracking,vx_mean" "$f" | cut -d, -f3)
    ag=$(grep "agree_pct" "$f" | cut -d, -f3 | head -1)
    mp=$(awk -F, '$1=="solve_time"&&$2=="mpc.p95"{p=$3} $1=="solve_time"&&$2=="mpc.max"{m=$3} END{printf "%.2f/%.2f", p, m}' "$f")
    wp=$(awk -F, '$1=="solve_time"&&$2=="wbc.p95"{p=$3} $1=="solve_time"&&$2=="wbc.max"{m=$3} END{printf "%.2f/%.2f", p, m}' "$f")
    printf "%-22s %-8s %-14s %-10s %-20s %s\n" "${NAMES[$k]}" "${VXS[$k]}" "$vxm" "$ag" "$mp" "$wp"
  done
} > "$SUM"
echo ""
echo "[matrix] summary -> $SUM"
cat "$SUM"
