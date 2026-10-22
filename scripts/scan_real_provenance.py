#!/usr/bin/env python3
"""real/ 批次来源证据单遍扫描（E2 诊断前视 / E3 电池重置 / E4 状态量级 / E5 力矩越界）

用法：
    python3 scripts/scan_real_provenance.py [run_dir ...]
    （缺省扫 rA9/real/ 全部 run）

输出 CSV + 行级摘要到 stdout，供 deliverables/C6/analysis 复核引用。
"""
import sys, os, glob, csv
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

TOPICS = ['/diagnostics', '/battery_state', '/jointTemperature',
          '/humanoid_mpc_observation', '/targetTorque', '/realTorque',
          '/real_push_fired', '/tf']
# 状态向量布局（analyze_real.py 同口径）：obs_v[:,8] = 基座高度 z
IZ = 8


def scan(run):
    bags = glob.glob(os.path.join(run, 'bag_0.mcap')) or \
           glob.glob(os.path.join(run, 'bag', 'bag_0.mcap'))
    if not bags:
        return None
    d = dict(run=run.rstrip('/').split('/')[-1])
    jtm, batt, tt, rt, z, roll = [], [], [], [], [], []
    push_t = []
    n_obs = 0
    with open(bags[0], 'rb') as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _sch, chan, raw, m in r.iter_decoded_messages(topics=TOPICS):
            t = chan.topic
            if t == '/diagnostics':
                for st in m.status:
                    kv = {p.key: p.value for p in st.values}
                    if 'joint_temp_max' in kv:
                        jtm.append((raw.log_time / 1e9,
                                    float(kv['joint_temp_max']),
                                    float(kv.get('bus_voltage', 'nan')),
                                    float(kv.get('phase_current_peak_a', 'nan')),
                                    float(kv.get('can_bus_load', 'nan'))))
            elif t == '/battery_state':
                batt.append((raw.log_time / 1e9, float(m.temperature),
                             float(m.percentage), str(m.serial_number)))
            elif t == '/jointTemperature':
                tt.append(max(m.data))
            elif t == '/targetTorque':
                rt.append(('cmd', max(abs(a) for a in m.data)))
            elif t == '/realTorque':
                rt.append(('real', max(abs(a) for a in m.data)))
            elif t == '/real_push_fired':
                push_t.append(float(m.data))
            elif t == '/humanoid_mpc_observation':
                v = m.state.value if hasattr(m.state, 'value') else m.state
                v = list(v)
                if len(v) > IZ:
                    n_obs += 1
                    z.append(v[IZ])
                    roll.append(abs(v[3]))
    d['n_diag'] = len(jtm)
    if jtm:
        a = np.array([x[1:] for x in jtm])
        d.update(jtm_first=jtm[0][1], jtm_last=jtm[-1][1], jtm_min=a[:, 0].min(),
                 jtm_max=a[:, 0].max(),
                 diag_first3_same=bool(np.allclose(a[0], a[1]) and np.allclose(a[1], a[2])),
                 diag_n_unique=len({round(x, 3) for x in a[:, 0]}),
                 bus_first3_same=bool(np.allclose(a[0, 1], a[1, 1]) and np.allclose(a[1, 1], a[2, 1])))
    d['jt_peak'] = max(tt) if tt else None
    if batt:
        b = np.array([x[1:3] for x in batt])
        d.update(batt_n=len(batt), batt_t0=b[0, 0], batt_t1=b[-1, 0],
                 soc0=b[0, 1], soc1=b[-1, 1], serial=batt[0][3])
    if n_obs:
        z = np.array(z); roll = np.array(roll)
        d.update(n_obs=n_obs, z_lo=z.min(), z_hi=z.max(), z_mean=z.mean(), z_std=z.std(),
                 roll_pk=roll.max())
    cmd = [v for k, v in rt if k == 'cmd']
    real = [v for k, v in rt if k == 'real']
    d['tau_cmd_pk'] = max(cmd) if cmd else None
    d['tau_real_pk'] = max(real) if real else None
    d['push_fired'] = push_t[0] if push_t else None
    return d


if __name__ == '__main__':
    runs = sys.argv[1:] or sorted(glob.glob('real/*/'))
    rows = [scan(r) for r in runs]
    rows = [r for r in rows if r]
    cols = ['run', 'n_diag', 'jtm_first', 'jtm_last', 'jtm_min', 'jtm_max', 'jt_peak',
            'diag_n_unique', 'diag_first3_same', 'bus_first3_same',
            'batt_n', 'batt_t0', 'batt_t1', 'soc0', 'soc1', 'serial',
            'n_obs', 'z_lo', 'z_hi', 'z_mean', 'z_std', 'roll_pk',
            'tau_cmd_pk', 'tau_real_pk', 'push_fired']
    out = 'deliverables/C6/analysis/real_provenance_scan.csv'
    with open(out, 'w', newline='') as fp:
        w = csv.DictWriter(fp, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c) for c in cols})
    print('wrote', out, len(rows), 'runs')
    print(f"{'run':<32}{'jtm_f':>7}{'jtm_l':>7}{'jtpk':>7}{'uniq':>6}"
          f"{'soc0':>10}{'soc1':>10}{'z_lo':>7}{'z_hi':>7}{'tcmd':>8}{'treal':>8}")
    for r in rows:
        print(f"{r['run']:<32}{r.get('jtm_first', float('nan')):>7.1f}"
              f"{r.get('jtm_last', float('nan')):>7.1f}"
              f"{(r.get('jt_peak') or float('nan')):>7.1f}"
              f"{r.get('diag_n_unique', 0):>6}"
              f"{r.get('soc0', float('nan')):>10.6f}{r.get('soc1', float('nan')):>10.6f}"
              f"{(r.get('z_lo') or float('nan')):>7.3f}{(r.get('z_hi') or float('nan')):>7.3f}"
              f"{(r.get('tau_cmd_pk') or float('nan')):>8.1f}"
              f"{(r.get('tau_real_pk') or float('nan')):>8.1f}")
