#!/bin/bash
# C6 扰动实验矩阵（C6交付需求 A2）：3 工况 × 3 冲量档 × 3 重复 = 27 run
#   站立(cmd_vx=0) 侧向 / 站立 前后 / 行走(cmd_vx=0.1) 侧向
#   冲量档 L1=3.1 / L2=6.2 / L3=12.4 N·s，力幅=冲量/80ms，作用点躯干 CoM 上 0.3m
# 命名：sim_push_{lat|fwd}_{L1|L2|L3}_{stand|walk}_{01..03}
# 用法：run_push.sh [目录筛选，如 "lat_L1"，空=全矩阵] [rep 起点=1]
# rep 起点用于配置变更后的重录批次（如 11 表示 sim_push_*_11 起）
set -u
RA9_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FILTER="${1:-}"
REP_BASE="${2:-1}"

declare -A IMP=( [L1]=3.1 [L2]=6.2 [L3]=12.4 )

run_one() {
  local dir="$1" level="$2" state="$3" rep="$4"
  local name="sim_push_${dir}_${level}_${state}_${rep}"
  [ -n "$FILTER" ] && [[ "$name" != *"$FILTER"* ]] && return 0
  [ -e "$RA9_DIR/experiments/$name/bag" ] && { echo "== $name 已存在，跳过"; return 0; }
  if [ "$state" = "stand" ]; then
    CMD=0.0; PUSH_DELAY_S=3
  else
    CMD=0.1; PUSH_DELAY_S=4
  fi
  echo "== $name: dir=$dir impulse=${IMP[$level]} N.s cmd=$CMD delay=${PUSH_DELAY_S}s"
  PUSH_DIR="$dir" PUSH_IMPULSE="${IMP[$level]}" PUSH_DURATION_MS=80 \
    PUSH_R_Z=0.3 PUSH_DELAY_S="$PUSH_DELAY_S" \
    "$RA9_DIR/scripts/run_experiment.sh" "$name" trot "$CMD" 30 25 \
    > "/tmp/push_$name.log" 2>&1
  python3 "$RA9_DIR/scripts/analyze_walk.py" "$RA9_DIR/experiments/$name/bag" \
    --cmd "$CMD" > "/tmp/push_${name}_an.log" 2>&1
  python3 "$RA9_DIR/scripts/analyze_push.py" "$RA9_DIR/experiments/$name" \
    > "/tmp/push_${name}_ap.log" 2>&1
  echo "   $(head -1 "$RA9_DIR/experiments/$name/analysis/summary.txt" 2>/dev/null) | $(head -1 "$RA9_DIR/experiments/$name/analysis/push_summary.txt" 2>/dev/null)"
}

for off in 0 1 2; do
  rep=$(printf "%02d" $((REP_BASE + off)))
  for level in L1 L2 L3; do
    for combo in "lat:stand" "fwd:stand" "lat:walk"; do
      run_one "${combo%%:*}" "$level" "${combo##*:}" "$rep"
    done
  done
done
echo PUSH_MATRIX_DONE
