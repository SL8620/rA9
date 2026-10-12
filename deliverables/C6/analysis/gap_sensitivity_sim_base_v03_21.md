# sim2real gap 灵敏度分析

源 run：`sim_base_v03_21`　seed=42　cmd_vx=0.3 m/s

各变体（含 bag + `SYNTHETIC.txt`）位于 `experiments/gap_sens_sim_base_v03_21/`；按 `deliverables/C6/README.md`「只留可复算的小体积结果」，本目录只放本表。

> ⚠️ **合成数据（非实机实测）**，用途：灵敏度分析 / 流水线验证 / 图表占位。
> **开环近似**：产物 = 理想执行器轨迹 + 非理想信号特性，不是真实
> 执行器下会产生的轨迹；延迟只移位信号、不改变运动（闭环反馈效应缺失）。
> 每个变体目录含 `SYNTHETIC.txt` 与 `params.txt: synthetic=true`。

## 各 gap 对指标的影响

数值 = 该 gap **单独开启**时的指标；Δ = 相对「无 gap」基线的变化。
`全开` = 所有 gap 同时开启（非各 Δ 之和，因各项非线性叠加）。

| 指标 | 无 gap 基线 | delay_cmd | Δ | delay_obs | Δ | torque_env | Δ | hyst_lag | Δ | backlash | Δ | encoder | Δ | imu | Δ | est | Δ | voltage | Δ | solve_jitter | Δ | 全开 | Δ |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| vx RMSE [m/s] | 0.03558 | 0.03558 | +0 | 0.03555 | -3.07e-05 | 0.03558 | +0 | 0.03558 | +0 | 0.03558 | +0 | 0.03558 | +0 | 0.03558 | +0 | 0.03558 | +0 | 0.03558 | +0 | 0.03558 | +0 | 0.03555 | -3.07e-05 |
| 基座高 std [m] | 0.004555 | 0.004555 | +0 | 0.004555 | -1.87e-07 | 0.004555 | +0 | 0.004555 | +0 | 0.004555 | +0 | 0.004555 | +0 | 0.004555 | +0 | 0.004973 | +0.000417 | 0.004555 | +0 | 0.004555 | +0 | 0.004985 | +0.000429 |
| 接触 agree% [L] | 97.06 | 97.06 | +0 | 96.59 | -0.472 | 97.06 | +0 | 97.06 | +0 | 97.06 | +0 | 97.06 | +0 | 97.06 | +0 | 97.06 | +0 | 97.06 | +0 | 97.06 | +0 | 96.59 | -0.472 |
| switch_err [ms] | 5.979 | 5.979 | +0 | 8.44 | +2.46 | 5.979 | +0 | 5.979 | +0 | 5.979 | +0 | 5.979 | +0 | 5.979 | +0 | 5.979 | +0 | 5.979 | +0 | 5.979 | +0 | 8.44 | +2.46 |
| 力矩 RMS [Nm] | 6.428 | 7.464 | +1.04 | 8.331 | +1.9 | 6.549 | +0.121 | 7.183 | +0.754 | 6.428 | +0 | 6.428 | +0 | 6.428 | +0 | 6.428 | +0 | 6.339 | -0.0895 | 6.428 | +0 | 6.73 | +0.301 |
| 力矩 max [Nm] | 378.6 | 381.1 | +2.46 | 379 | +0.383 | 378.6 | +0 | 382.6 | +3.94 | 378.6 | +0 | 378.6 | +0 | 378.6 | +0 | 378.6 | +0 | 378.6 | +0 | 378.6 | +0 | 382.6 | +3.94 |
| PD修正 RMS [Nm] | 6.429 | 8.194 | +1.77 | 6.43 | +0.0012 | 6.429 | +0 | 6.429 | +0 | 6.429 | +0 | 6.429 | +0 | 6.429 | +0 | 6.429 | +0 | 6.429 | +0 | 6.429 | +0 | 8.196 | +1.77 |
| 钳制损失 RMS [Nm] | 0 | 8.27 | +8.27 | 7.582 | +7.58 | 1.274 | +1.27 | 5.688 | +5.69 | 0 | +0 | 0 | +0 | 0 | +0 | 0 | +0 | 0.4148 | +0.415 | 0 | +0 | 5.604 | +5.6 |
| 关节角 RMS [rad] | 0.009989 | 0.0105 | +0.000512 | 0.01019 | +0.000204 | 0.009989 | +0 | 0.009989 | +0 | 0.0104 | +0.000406 | 0.00999 | +3.74e-07 | 0.009989 | +0 | 0.009989 | +0 | 0.009989 | +0 | 0.009989 | +0 | 0.0104 | +0.000409 |
| 关节角 max [rad] | 0.1579 | 0.1806 | +0.0227 | 0.1487 | -0.00925 | 0.1579 | +0 | 0.1579 | +0 | 0.1559 | -0.002 | 0.158 | +6.54e-05 | 0.1579 | +0 | 0.1579 | +0 | 0.1579 | +0 | 0.1579 | +0 | 0.1595 | +0.0016 |
| MPC mean [ms] | 2.728 | 2.728 | +0 | 2.729 | +8.98e-05 | 2.728 | +0 | 2.728 | +0 | 2.728 | +0 | 2.728 | +0 | 2.728 | +0 | 2.728 | +0 | 2.728 | +0 | 2.77 | +0.0418 | 2.775 | +0.0467 |
| WBC mean [ms] | 0.1959 | 0.1959 | +0 | 0.1958 | -2.03e-05 | 0.1959 | +0 | 0.1959 | +0 | 0.1959 | +0 | 0.1959 | +0 | 0.1959 | +0 | 0.1959 | +0 | 0.1959 | +0 | 0.2004 | +0.0045 | 0.2001 | +0.00425 |

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
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_delay_cmd --seed 42 --gaps delay_cmd
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_delay_obs --seed 42 --gaps delay_obs
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_torque_env --seed 42 --gaps torque_env
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_hyst_lag --seed 42 --gaps hyst_lag
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_backlash --seed 42 --gaps backlash
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_encoder --seed 42 --gaps encoder
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_imu --seed 42 --gaps imu
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_est --seed 42 --gaps est
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_voltage --seed 42 --gaps voltage
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth_solve_jitter --seed 42 --gaps solve_jitter
python3 scripts/make_synth_real.py /home/mi/disk/SS/thesis/rA9/experiments/sim_base_v03_21 <out>/synth___ALL__ --seed 42 --gaps delay_cmd,delay_obs,torque_env,hyst_lag,backlash,encoder,imu,est,voltage,solve_jitter
```
