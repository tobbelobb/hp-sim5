"""Print the headline of refit2.py outputs."""
import json, sys
for path in sys.argv[1:]:
    d = json.load(open(path))
    print(d['run'], {k: round(d[k], 3) if isinstance(d[k], float) else d[k] for k in ('k_true', 'k_probe', 'k_joint', 'num_probes', 'probe_s')},
          'live max', round(d['live']['max_abs_frame_error_mm'] or -1, 3), 'sim_s', d['live']['simulated_s'], d['live']['stop_reason'])
    for v in ('config', 'probe', 'oracle', 'joint'):
        if d[v]:
            print('  ', v.ljust(6), {k: round(d[v][k], 3) for k in ('k', 'max_abs_frame_mm', 'top_z_frame_mm', 'low_z_frame_mm', 'rms_mm')})
