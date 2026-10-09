# 存量 run 有效性一览（2026-10-09 由 analyze_walk.py 重跑）

复算：`python3 scripts/analyze_walk.py experiments/<run>/bag --cmd <cmd>`（run 根目录含 metadata.yaml 的直接指 run 目录）。

| run | cmd | STATUS | RTF |
|---|---|---|---|
| auto_ab_01 | 0.0 | INVALID(倒地/姿态超限(93% 样本, 基准高 0.72 m); 高度均值 0.54 m(<0.70, 疑倒地)) | 0.843 |
| auto_ab_02 | 0.0 | INVALID(倒地/姿态超限(94% 样本, 基准高 0.72 m)) | 0.845 |
| auto_ab_03 | 0.0 | INVALID(录制 56% 处于暂停态（数据大量缺失）; 倒地/姿态超限(94% 样本, 基准高 0.72 m)) | 0.845 |
| auto_ab_04 | 0.0 | VALID | 0.018 |
| auto_ab_05 | 0.0 | INVALID(倒地/姿态超限(93% 样本, 基准高 0.72 m)) | 0.845 |
| auto_smoke_test | 0.3 | INVALID(缺话题 /cmd_contactFlag//targetTorque//targetPos; 接触标志为空/维度不符; 力矩话题为空（/realTorque 只在仿真非暂停时发布）; 关节话题为空; 倒地/姿态超限(94% 样本, 基准高 -0.24 m)) | 0.282 |
| auto_v2 |  | INVALID(bag 无法读取: RuntimeError: No storage could be initialized for the input URI: experiments/auto_v2/bag) |  |
| auto_v3 | 0.3 | INVALID(缺话题 /cmd_contactFlag//targetTorque//targetPos; 接触标志为空/维度不符; 力矩话题为空（/realTorque 只在仿真非暂停时发布）; 关节话题为空; 倒地/姿态超限(100% 样本, 基准高 -0.24 m); 高度均值 -0.24 m(<0.70, 疑倒地)) | 0.773 |
| auto_v4 | 0.3 | INVALID(缺话题 /cmd_contactFlag//targetTorque//targetPos; 接触标志为空/维度不符; 力矩话题为空（/realTorque 只在仿真非暂停时发布）; 关节话题为空; 倒地/姿态超限(100% 样本, 基准高 0.68 m); 高度均值 0.68 m(<0.70, 疑倒地)) | 0.824 |
| auto_v5 | 0.3 | INVALID(缺话题 /cmd_contactFlag//targetTorque//targetPos; 接触标志为空/维度不符; 力矩话题为空（/realTorque 只在仿真非暂停时发布）; 关节话题为空; 倒地/姿态超限(100% 样本, 基准高 0.53 m); 高度均值 0.53 m(<0.70, 疑倒地)) | 0.885 |
| auto_v6 | 0.3 | INVALID(缺话题 /cmd_contactFlag//targetTorque//targetPos; 接触标志为空/维度不符; 力矩话题为空（/realTorque 只在仿真非暂停时发布）; 关节话题为空; 倒地/姿态超限(100% 样本, 基准高 0.55 m); 高度均值 0.55 m(<0.70, 疑倒地)) | 0.908 |
| auto_v7 | 0.0 | INVALID(倒地/姿态超限(94% 样本, 基准高 0.72 m)) | 0.843 |
| auto_v8 | 0.1 | INVALID(倒地/姿态超限(93% 样本, 基准高 0.72 m); 高度均值 0.48 m(<0.70, 疑倒地)) | 0.840 |
| manual_repro_003 | 0.0 | VALID | 0.889 |
| repro_manual_001 | 0.0 | INVALID(缺话题 /targetTorque//targetPos; 力矩话题为空（/realTorque 只在仿真非暂停时发布）; 关节话题为空; 无 /mpc_solve_time_ms（旧 bag 未埋点）; 高度均值 0.47 m(<0.70, 疑倒地)) | 0.787 |
| rtf_final_001 | 0.1 | VALID | 1.000 |
| rtf_prof_001 | 0.0 | INVALID(缺话题 /simContactFlag//realTorque; 接触标志为空/维度不符; 力矩话题为空（/realTorque 只在仿真非暂停时发布）) | nan |
| rtf_prof_002 | 0.0 | VALID | 0.803 |
| rtf_prof_003 | 0.0 | VALID | 0.803 |
| rtf_smoke_001 | 0.0 | INVALID(倒地/姿态超限(96% 样本, 基准高 0.52 m)) | 0.845 |
| rtf_smoke_002 | 0.0 | INVALID(倒地/姿态超限(94% 样本, 基准高 0.72 m)) | 0.845 |
| rtf_smoke_003 | 0.0 | INVALID(录制 66% 处于暂停态（数据大量缺失）; 倒地/姿态超限(94% 样本, 基准高 0.72 m)) | 0.844 |
| rtf_smoke_004 | 0.0 | VALID | 0.843 |
| rtf_smoke_005 | 0.1 | VALID | 0.841 |
| rtf_verify_001 | 0.1 | VALID | 1.000 |
| walk_smoke_001 | 0.3 | INVALID(无 /mpc_solve_time_ms（旧 bag 未埋点）) | 0.818 |
