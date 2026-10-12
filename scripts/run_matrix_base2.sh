#!/bin/bash
# 批次 2x 基线重录：三档速度 × 3 重复 = 9 run（trot）
#
# 批次含义（NOTE.md §12/§13）：2x = 2026-10-12 语义修正后重录
#   · /realTorque = qfrc_actuator[-12:]（执行器实际出力，500Hz 发布）
#   · /controlInput = data.ctrl（控制输入）
#   · 力矩界两层钳制已对齐 task_.info（200/200/320/320/200/200）
#   · /simContactFlag 等测量走 delay_obs（后处理侧，不影响本批原始数据）
# 对照：旧 sim_base_*_0x 批保留为「语义修正前」参照，不删除。
#
# 协议与旧批一致以便对照：gait=trot, cmd_vel_mode=continuous,
#   pre_unpause_wait=2, duration_s=20, record_s=15
#
# 前置（NOTE.md §10 三件套）：负载<5 / 无孤儿进程 / 冒烟 RTF>=0.98
# 且 git 必须已冻结（git_dirty=0），否则论文数字无法溯源。
set -u
RA9_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== 采集前置检查 ==="
L=$(cut -d' ' -f1 /proc/loadavg)
ok=1
if [ "$(echo "$L < 5" | bc -l 2>/dev/null || echo 0)" = "1" ]; then echo "  uptime 1min = $L  ✓"; else echo "  uptime 1min = $L  ✗ 需 <5"; ok=0; fi
# pgrep 不自匹配（脚本文件内调用时，本脚本 cmdline 不含该 pattern）
orph=$(pgrep -fc "humanoid_mujoco_sim/humanoid_sim" || true)
orph=${orph:-0}
if [ "$orph" -eq 0 ]; then echo "  孤儿 humanoid_sim = 0  ✓"; else echo "  孤儿 humanoid_sim = $orph  ✗ 需清场"; ok=0; fi
GD=$(git -C "$RA9_DIR" status --porcelain | wc -l)
if [ "$GD" -eq 0 ]; then
  echo "  git_dirty = 0  ✓ 已冻结"
else
  echo "  git_dirty = $GD  ✗ 未冻结（项目记忆规矩：收数据前须提交）"
  echo "    → 拒绝在 dirty 工作区收数据（论文数字须溯源到唯一 commit）。"
  echo "      请先 commit 再重跑本脚本。"
  exit 2
fi
[ "$ok" = "1" ] || { echo "  → 前置检查未过，退出"; exit 1; }

RUNS=(
  "sim_base_v00_21 0.0" "sim_base_v00_22 0.0" "sim_base_v00_23 0.0"
  "sim_base_v01_21 0.1" "sim_base_v01_22 0.1" "sim_base_v01_23 0.1"
  "sim_base_v03_21 0.3" "sim_base_v03_22 0.3" "sim_base_v03_23 0.3"
)

SUM="$RA9_DIR/experiments/matrix_base2_summary.txt"
{
  echo "batch=2x  date=$(date -Iseconds)  git=$(git -C "$RA9_DIR" rev-parse --short HEAD)"
  printf '%-20s %-6s %-9s %-8s %-8s %-8s %-8s\n' \
    "run" "cmd" "RTF" "捕获%" "agree%" "sw_err" "τRMS"
} > "$SUM"

for entry in "${RUNS[@]}"; do
  read -r NAME VX <<< "$entry"
  echo
  echo "════════ $NAME  (cmd_vx=$VX) ════════"
  # run_experiment.sh 已录包 + 写 params.txt；此处只补分析
  # （不能调 collect.sh —— 它会再次录包，与已有 bag 冲突）
  bash "$RA9_DIR/scripts/run_experiment.sh" "$NAME" trot "$VX" 20 15 || {
    echo "  ✗ $NAME 采集失败，跳过"; continue; }
  python3 "$RA9_DIR/scripts/analyze_walk.py" "$RA9_DIR/experiments/$NAME/bag" \
    --cmd "$VX" --out "$RA9_DIR/experiments/$NAME/analysis" >/dev/null 2>&1 || \
    echo "  ⚠️ $NAME 分析失败"
  M="$RA9_DIR/experiments/$NAME/analysis/metrics.csv"
  if [ -f "$M" ]; then
    python3 - "$M" "$NAME" "$VX" "$SUM" <<'PY'
import csv, sys
m = {}
for r in csv.reader(open(sys.argv[1])):
    if len(r) >= 3:
        m[r[1]] = r[2]
def g(k):
    try: return float(m.get(k, ""))
    except: return float("nan")
line = (f"{sys.argv[2]:<20} {sys.argv[3]:<6} {g('rtf'):>9.3f} "
        f"{g('capture_pct'):>8.1f} {g('L.agree_pct'):>8.1f} "
        f"{g('L.switch_err_ms'):>8.1f} {g('rms_total'):>8.2f}")
open(sys.argv[4], "a").write(line + "\n")
print("  " + line)
PY
  fi
  # 每格之间让系统喘口气，避免上一 run 的进程清理未完成
  sleep 3
done

echo
echo "=== 汇总 -> $SUM ==="
cat "$SUM"
