# sim2real gap 灵敏度分析

源 run：`sim_base_v03_02`　seed=42　cmd_vx=0.3 m/s

各变体（含 bag + `SYNTHETIC.txt`）位于 `experiments/gap_sens_sim_base_v03_02/`；按 `deliverables/C6/README.md`「只留可复算的小体积结果」，本目录只放本表。

> ⚠️ **合成数据（非实机实测）**，用途：灵敏度分析 / 流水线验证 / 图表占位。
> **开环近似**：产物 = 理想执行器轨迹 + 非理想信号特性，不是真实
> 执行器下会产生的轨迹；延迟只移位信号、不改变运动（闭环反馈效应缺失）。
> 每个变体目录含 `SYNTHETIC.txt` 与 `params.txt: synthetic=true`。

## 各 gap 对指标的影响

数值 = 该 gap **单独开启**时的指标；Δ = 相对「无 gap」基线的变化。
`全开` = 所有 gap 同时开启（非各 Δ 之和，因各项非线性叠加）。

| 指标 | 无 gap 基线 | delay_cmd | Δ | delay_obs | Δ | torque_env | Δ | hyst_lag | Δ | backlash | Δ | encoder | Δ | imu | Δ | est | Δ | voltage | Δ | solve_jitter | Δ | 全开 | Δ |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| vx RMSE [m/s] | 0.04971 | 0.04971 | +0 | 0.04972 | +1.19e-05 | 0.04971 | +0 | 0.04971 | +0 | 0.04971 | +0 | 0.04971 | +0 | 0.04971 | +0 | 0.04971 | +0 | 0.04971 | +0 | 0.04971 | +0 | 0.04972 | +1.19e-05 |
| 基座高 std [m] | 0.004244 | 0.004244 | +0 | 0.004244 | +4.23e-08 | 0.004244 | +0 | 0.004244 | +0 | 0.004244 | +0 | 0.004244 | +0 | 0.004244 | +0 | 0.004714 | +0.00047 | 0.004244 | +0 | 0.004244 | +0 | 0.004656 | +0.000412 |
| 接触 agree% [L] | 96.97 | 96.97 | +0 | 96.97 | -0.000699 | 96.97 | +0 | 96.97 | +0 | 96.97 | +0 | 96.97 | +0 | 96.97 | +0 | 96.97 | +0 | 96.97 | +0 | 96.97 | +0 | 96.97 | -0.000699 |
| switch_err [ms] | 5.47 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 | 5.47 | +0 |
| 力矩 RMS [Nm] | 9.048 | 8.989 | -0.0595 | 9.05 | +0.00173 | 9.087 | +0.0388 | 9.123 | +0.0749 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 8.888 | -0.16 |
| 力矩 max [Nm] | 383.2 | 383.2 | +0 | 383.2 | +0 | 383.2 | +0 | 379 | -4.18 | 383.2 | +0 | 383.2 | +0 | 383.2 | +0 | 383.2 | +0 | 383.2 | +0 | 383.2 | +0 | 379 | -4.18 |
| PD修正 RMS [Nm] | 9.048 | 8.989 | -0.0595 | 9.05 | +0.00173 | 9.087 | +0.0388 | 9.123 | +0.0749 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 9.048 | +0 | 8.888 | -0.16 |
| 钳制损失 RMS [Nm] | 0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 |
| 关节角 RMS [rad] | 0.009623 | 0.009994 | +0.00037 | 0.00996 | +0.000337 | 0.009623 | +0 | 0.009623 | +0 | 0.01006 | +0.000439 | 0.009625 | +1.61e-06 | 0.009623 | +0 | 0.009623 | +0 | 0.009623 | +0 | 0.009623 | +0 | 0.01008 | +0.000454 |
| 关节角 max [rad] | 0.1596 | 0.1669 | +0.00732 | 0.1559 | -0.00367 | 0.1596 | +0 | 0.1596 | +0 | 0.1576 | -0.002 | 0.1596 | +4.35e-05 | 0.1596 | +0 | 0.1596 | +0 | 0.1596 | +0 | 0.1596 | +0 | 0.1614 | +0.00181 |
| MPC mean [ms] | 2.656 | 2.656 | +0 | 2.656 | +0.000195 | 2.656 | +0 | 2.656 | +0 | 2.656 | +0 | 2.656 | +0 | 2.656 | +0 | 2.656 | +0 | 2.656 | +0 | 2.721 | +0.0651 | 2.711 | +0.0553 |
| WBC mean [ms] | 0.1995 | 0.1995 | +0 | 0.1995 | +3.43e-06 | 0.1995 | +0 | 0.1995 | +0 | 0.1995 | +0 | 0.1995 | +0 | 0.1995 | +0 | 0.1995 | +0 | 0.1995 | +0 | 0.2048 | +0.00531 | 0.2029 | +0.00349 |

## gap 说明

| gap | 含义 |
|---|---|
| `delay_cmd` | 执行侧指令延迟 delay_cmd_ms |
| `delay_obs` | 观测侧延迟 delay_obs_ms |
| `torque_env` | 力矩-转速包络 τ_max(ω) |
| `hyst_lag` | 电机迟滞(play) + 一阶滞后 |
| `backlash` | 背隙 |
| `encoder` | 编码器量化 + 噪声 |
| `imu` | IMU 噪声 + 零偏游走 |
| `est` | 状态估计噪声（替代真值） |
| `voltage` | 母线电压跌落 + 力矩饱和 |
| `solve_jitter` | 求解耗时抖动 |
| `__ALL__` | 全部 gap 同时开启 |

## 复算

```bash
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_delay_cmd --seed 42 --gaps delay_cmd
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_delay_obs --seed 42 --gaps delay_obs
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_torque_env --seed 42 --gaps torque_env
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_hyst_lag --seed 42 --gaps hyst_lag
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_backlash --seed 42 --gaps backlash
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_encoder --seed 42 --gaps encoder
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_imu --seed 42 --gaps imu
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_est --seed 42 --gaps est
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_voltage --seed 42 --gaps voltage
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth_solve_jitter --seed 42 --gaps solve_jitter
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_02 <out>/synth___ALL__ --seed 42 --gaps delay_cmd,delay_obs,torque_env,hyst_lag,backlash,encoder,imu,est,voltage,solve_jitter
```
