#!/bin/bash
# 扰动矩阵补录：按格补齐到 N_VALID 个"有效扰动样本"。
# 有效 = analyze_push 判定不含：注入未生效 / 注入前已倒地（自发故障）。
# 推力致倒（FALLEN）保留为扰动抑制上限证据（交付需求 D：不丢弃）。
# 补录从 rep 04 起编号；单格最多试 MAX_TRY 次（自发故障率 ~25%）。
set -u
RA9_DIR="$(cd "$(dirname "$0")/.." && pwd)"
N_VALID="${1:-3}"
MAX_TRY="${2:-6}"
declare -A IMP=( [L1]=3.1 [L2]=6.2 [L3]=12.4 )

count_valid() {  # $1=cell前缀
  local n=0
  for d in "$RA9_DIR"/experiments/sim_push_$1_*/; do
    [ -f "$d/analysis/push_summary.txt" ] || continue
    if ! grep -qE "注入疑似未生效|注入前已倒地" "$d/analysis/push_summary.txt"; then
      n=$((n+1))
    fi
  done
  echo "$n"
}

for combo in "lat:stand" "fwd:stand" "lat:walk"; do
  dir="${combo%%:*}"; state="${combo##*:}"
  for level in L1 L2 L3; do
    cell="${dir}_${level}_${state}"
    have=$(count_valid "$cell")
    try=4
    while [ "$have" -lt "$N_VALID" ] && [ "$try" -lt $((4+MAX_TRY)) ]; do
      rep=$(printf "%02d" "$try")
      name="sim_push_${cell}_${rep}"
      try=$((try+1))
      if [ -e "$RA9_DIR/experiments/$name/bag" ]; then continue; fi
      if [ "$state" = "stand" ]; then CMD=0.0; DELAY=3; else CMD=0.1; DELAY=4; fi
      echo "== $name (cell=$cell have=$have/$N_VALID)"
      PUSH_DIR="$dir" PUSH_IMPULSE="${IMP[$level]}" PUSH_DURATION_MS=80 \
        PUSH_R_Z=0.3 PUSH_DELAY_S="$DELAY" \
        "$RA9_DIR/scripts/run_experiment.sh" "$name" trot "$CMD" 30 25 \
        > "/tmp/push_$name.log" 2>&1
      python3 "$RA9_DIR/scripts/analyze_walk.py" "$RA9_DIR/experiments/$name/bag" \
        --cmd "$CMD" > "/tmp/push_${name}_an.log" 2>&1
      python3 "$RA9_DIR/scripts/analyze_push.py" "$RA9_DIR/experiments/$name" \
        > "/tmp/push_${name}_ap.log" 2>&1
      echo "   $(head -1 "$RA9_DIR/experiments/$name/analysis/push_summary.txt" 2>/dev/null)"
      have=$(count_valid "$cell")
    done
  done
done
echo TOPUP_DONE
