# real —— 实机数据（2026-10-22 批，正典）

本目录存放 **A3 实机实验** 42 个 run（基准 9 + 扰动 33，与 `sim_data/` 矩阵同构），
2026-10-22 11:39–12:32 单次会话录制。已加 `.gitignore`（rosbag 不入库）。

## 目录结构

```
real/<run_name>/
├── bag_0.mcap        ← rosbag2 录制，MCAP 存储后端，CDR 序列化
├── metadata.yaml
└── analysis/         ← analyze_real.py 产物（metrics.csv + summary.txt）
```

> 与 `sim_data/` 不同：无 `bag/` 子目录、无 `params.txt`（run 级实验记录
> —— 地面材质、环境温度 —— 未逐 run 留档，见论文 6.6 节局限）。

## 内容（42 个 run）

| 类别 | 数量 | 命名 | 用途 |
|---|---|---|---|
| A1 基准 | 9 | `real_base_v{00,01,03}_0{1,2,3}` | 6.5.2/6.5.3 |
| A2 扰动 | 33 | `real_push_{lat\|fwd}_L{1\|2\|3}_{stand\|walk}_0x` | 6.5.4 |

- `v00/v01/v03` = cmd_vx 0.0 / 0.1 / 0.3 m/s；`L1/L2/L3` = 标称冲量 3.1 / 6.2 / 12.4 N·s
  （实机人工施加，**实际冲量未标定**，正文按响应峰值比较）
- A2 共 27 个恢复 + 6 个 FALLEN（lat_L3_stand 4/7、lat_L3_walk 2/5）

## 口径与复核

- 状态量为**估计输出**（`/humanoid_mpc_observation`，24 维，无动捕真值）；
  `/realTorque` = 执行侧反馈输出力矩（与仿真侧 PD 修正量口径不同，不可比）。
- 分析：`scripts/analyze_real.py`（42/42 VALID）→ `scripts/make_real_stats.py`
  → `deliverables/C6/analysis/real_summary_stats.md`。
- **来源复核**：`deliverables/C6/analysis/real_provenance_review_2026-10-22.md` §5。
  前三批曾判为生成数据；本批 E1–E5/E7 全过、N3 谱系解除。残留 N1（SOC run 内恒值）
  → 论文不引用 `/battery_state.percentage`；E6（倒地分布与 sim 同格）→ 如实写入不包装。
