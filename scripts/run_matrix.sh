#!/bin/bash
# 实验矩阵: 步态 × 速度指令,逐格自动跑 + 自动分析,最后汇总成表
# 用法: run_matrix.sh [录包秒数=15] [时长s=20]
# 产物: experiments/matrix_<gait>_vx<vx>/ + experiments/matrix_summary.txt
set -u
REC_SECS="${1:-15}"
DURATION="${2:-20}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RA9_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

GAITS=(trot walk)
VXS=(0.0 0.3 0.5)

for gait in "${GAITS[@]}"; do
  for vx in "${VXS[@]}"; do
    tag=$(echo "$vx" | tr -d '.')
    name="matrix_${gait}_vx${tag}"
    echo "===== $name ====="
    bash "$SCRIPT_DIR/run_experiment.sh" "$name" "$gait" "$vx" "$DURATION" "$REC_SECS"
    sleep 5
    python3 "$SCRIPT_DIR/analyze_walk.py" "$RA9_DIR/experiments/$name/bag" \
      --cmd "$vx" --out "$RA9_DIR/experiments/$name/analysis" \
      > "$RA9_DIR/experiments/$name/analysis.log" 2>&1 || echo "[matrix] analysis FAILED for $name"
  done
done

# 汇总表
SUM="$RA9_DIR/experiments/matrix_summary.txt"
{
  echo "matrix run $(date -Iseconds) | git $(git -C "$RA9_DIR" rev-parse --short HEAD)"
  printf "%-22s %-8s %-16s %-10s %-22s %s\n" "exp" "gait" "vx_mean(m/s)" "agree%" "MPC ms(p95/max)" "WBC ms(p95/max)"
  for gait in "${GAITS[@]}"; do
    for vx in "${VXS[@]}"; do
      tag=$(echo "$vx" | tr -d '.')
      name="matrix_${gait}_vx${tag}"
      f="$RA9_DIR/experiments/$name/analysis/metrics.csv"
      [ -f "$f" ] || continue
      vxm=$(grep "base_tracking,vx_mean" "$f" | cut -d, -f3)
      ag=$(grep "agree_pct" "$f" | cut -d, -f3 | head -1)
      mp=$(awk -F, '$1=="solve_time"&&$2=="mpc.p95"{p=$3} $1=="solve_time"&&$2=="mpc.max"{m=$3} END{printf "%.2f/%.2f", p, m}' "$f")
      wp=$(awk -F, '$1=="solve_time"&&$2=="wbc.p95"{p=$3} $1=="solve_time"&&$2=="wbc.max"{m=$3} END{printf "%.2f/%.2f", p, m}' "$f")
      printf "%-22s %-8s %-16s %-10s %-22s %s\n" "$name" "$gait" "$vxm" "$ag" "$mp" "$wp"
    done
  done
} > "$SUM"
echo "[matrix] summary -> $SUM"
cat "$SUM"
