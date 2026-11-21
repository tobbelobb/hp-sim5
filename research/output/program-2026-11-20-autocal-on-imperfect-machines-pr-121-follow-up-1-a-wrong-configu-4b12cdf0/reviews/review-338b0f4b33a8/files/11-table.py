"""Per-run and summary table of the force-realisation variants.

usage: table.py RESULTS_DIR...   (reads *.cog.json and *.jointk.json side by side)

Columns (print-frame max |error| / top z, mm, against scene truth):
  live       the live run's own result (configured k; probe not used by the live fit)
  cmd@true   commanded forces, true k            (cog.json)
  map@true   oracle cogging map, true k          (cog.json; bound for dither / calibrated map)
  jk         joint rms k, commanded forces       (jointk.json)
  jk+cog     joint k + fitted per-motor ripple   (jointk.json)
"""
import glob, json, os, sys
import numpy as np

rows = []
for d in sys.argv[1:]:
    for f in sorted(glob.glob(os.path.join(d, '*.jointk.json'))):
        if not os.path.getsize(f):
            continue
        j = json.load(open(f))
        c = json.load(open(f.replace('.jointk.json', '.cog.json')))
        run_dir = j['run']
        live = None
        for base in ('output/fr', 'research/output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-compliance-probe-where-stiffness-really-trades-against-the-top-a-62081729/evidence/runs'):
            p = os.path.join(base, run_dir, 'result.json')
            if os.path.exists(p):
                r = json.load(open(p))
                live = dict(max_abs_mm=r['max_abs_frame_error_mm'], top_z_mm=r['frame_error_mm'][-1][2],
                            simulated_s=r.get('simulated_s'))
                break
        rows.append(dict(run=run_dir, machine=j['machine'], set=os.path.basename(os.path.normpath(d)),
                         k_true=j['k_true'], live=live, cmd=c['commanded_true_k'], map=c['cog_map_true_k'],
                         jk=j['joint_k_plain'], jkc=j['joint_k_cog']))

fmt = lambda v: f"{v['max_abs_mm']:.2f} / {v['top_z_mm']:+.2f}"
print('| set | run | k_true | live | cmd@true | map@true | jk (k/k_true) | jk+cog (k/k_true) |')
print('| --- | --- | --- | --- | --- | --- | --- | --- |')
for r in rows:
    print(f"| {r['set']} | {r['run']} | {r['k_true']:.3f} | {fmt(r['live'])} | {fmt(r['cmd'])} | {fmt(r['map'])} | "
          f"{fmt(r['jk'])} ({r['jk']['k_over_true']:.3f}) | {fmt(r['jkc'])} ({r['jkc']['k_over_true']:.3f}) |")
print()
print('| machine | set | n | live | cmd@true | map@true | jk | jk+cog | k/k_true jk | k/k_true jk+cog |')
print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
for m in sorted({r['machine'] for r in rows}):
    for s in sorted({r['set'] for r in rows}) + ['all']:
        R = [r for r in rows if r['machine'] == m and s in (r['set'], 'all')]
        if not R:
            continue
        ms = lambda key: (lambda a: f'{a.mean():.2f} ± {a.std():.2f} ({a.max():.2f})')(np.array([r[key]['max_abs_mm'] for r in R]))
        ks = lambda key: (lambda a: f'{(a.mean() - 1) * 100:+.1f} ± {a.std() * 100:.1f}%')(np.array([r[key]['k_over_true'] for r in R]))
        print(f'| {m} | {s} | {len(R)} | {ms("live")} | {ms("cmd")} | {ms("map")} | {ms("jk")} | {ms("jkc")} | {ks("jk")} | {ks("jkc")} |')
