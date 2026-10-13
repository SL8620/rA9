# sim_data —— 仿真数据（batch `_2x`，2026-10-13 迁入）

本目录存放 **2026-10-12 语义修正后重录**的仿真数据（正典批 `_2x`），
原在 `experiments/`，2026-10-13 整体迁出到此。已加 `.gitignore`（11GB 不入库）。

## 目录结构

```
sim_data/<run_name>/
├── bag/
│   ├── bag_0.mcap          ← rosbag2 录制，MCAP 存储后端，CDR 序列化
│   └── metadata.yaml
├── params.txt              ← 实验参数 + git_commit/git_dirty 溯源
└── analysis/
    ├── metrics.csv         ← analyze_walk.py 产物
    ├── summary.txt
    ├── push_summary.txt    ← analyze_push.py 产物（仅扰动 run）
    └── fig*.png
```

## 内容（45 个 run，11 GB）

| 类别 | 数量 | 命名 | 用途 |
|---|---|---|---|
| **A1 基线** | 9 | `sim_base_v{00,01,03}_2{1,2,3}` | 6.4.1–6.4.4 |
| **A2 扰动** | 33 | `sim_push_{lat\|fwd}_L{1\|2\|3}_{stand\|walk}_2x` | 6.4.5 |
| 冒烟验证 | 3 | `smoke_newsem_0{1,2,3}` | 非交付，验证语义/采样率 |

- `v00/v01/v03` = cmd_vx 0.0 / 0.1 / 0.3 m/s
- `L1/L2/L3` = 冲量 3.1 / 6.2 / 12.4 N·s
- `_24`–`_27` 是补录（补齐 `lat_L3` 两格的未倒样本）
- A2 共 27 个恢复样本 + 6 个 FALLEN（推力致倒，按交付需求 D 保留）

## 批次说明

| 批次 | 位置 | 说明 |
|---|---|---|
| **`_2x`** | **本目录** | 语义修正后正典，**论文报力矩峰值只能用这批** |
| `_0x` | `experiments/` | 旧正典（语义修正前），姿态/位移类指标仍可用 |
| `_1x` / `_11+` | `experiments/` | 探索批（临时力矩界），仅勘验用，不进论文 |

旧批 `/realTorque` 是 29.4 Hz 采样，**漏 86% 力矩峰值**（同格实测 27.89 vs
200.00 Nm）。详见 `deliverables/C6/analysis/push_batch_comparison.md`。

## 本批相对旧批的修正

1. `/realTorque` = `qfrc_actuator[-12:]`（执行器实际出力，500Hz）+ 新增 `/controlInput`
2. 力矩界两层钳制（`ctrlrange` + `actuatorfrcrange`）对齐 `task_.info`
3. `analyze_walk.py` 稳态窗加步态收敛下界（修 v00 duty 被 stance 污染）
4. `analyze_push.py` t_rec 判据改「稳定下来」（修 78% 假阴性）

详见 `项目记忆.md` 与 `deliverables/C6/analysis/NOTE.md` §12/§13。

## 读取方式

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info sim_data/sim_base_v03_21/bag

# 仓库脚本（口径已校准，推荐）
python3 scripts/analyze_walk.py sim_data/sim_base_v03_21/bag --cmd 0.3
python3 scripts/analyze_push.py sim_data/sim_push_lat_L3_stand_25
```

## ⚠️ 画图脚本路径尚未更新

`scripts/make_fig6.py`、`review_data.py`、`gap_sensitivity.py`、`collect.sh`
里共 10 处仍指向 `experiments/`，**已失效**。重画图时须改成 `sim_data/`；
具体行号见 `scripts/make_fig6.py` 文件头的迁移说明。
