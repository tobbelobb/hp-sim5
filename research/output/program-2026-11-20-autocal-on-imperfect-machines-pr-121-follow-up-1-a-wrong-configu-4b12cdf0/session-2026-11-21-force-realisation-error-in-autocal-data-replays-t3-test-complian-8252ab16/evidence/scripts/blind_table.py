"""Table of blind_force_fit outputs: usage blind_table.py FILES..."""
import json, sys
f = lambda v: f"{v['max_abs_mm']:.2f} / {v['top_z_mm']:+.2f} ({v['k_over_true']:.3f})"
print('| run | case | cmd@true | plain jk | blind ripple + jk | blind full + jk | harmonics (full, final) | relative constant N (full) |')
print('| --- | --- | --- | --- | --- | --- | --- | --- |')
for path in sys.argv[1:]:
    d = json.load(open(path))
    case = path.rsplit('/', 1)[-1].split('.')[1]
    h = d['full']['selection'][0]['harmonics']
    rel = {m: v['relative'] for m, v in (d['full']['constant_n'] or {}).items()}
    print(f"| {d['run']} | {case} | {f(d['commanded_true_k'])} | {f(d['plain'])} | {f(d['ripple'])} | {f(d['full'])} | "
          f"{'; '.join(f'{m}: {v}' for m, v in h.items())} | {', '.join(f'{m}: {v:+.2f}' for m, v in rel.items())} |")
