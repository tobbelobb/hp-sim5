"""Plot saved analytical core-fit evidence without running a fit or simulation."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'attempt-3b2e360ec1834eee9bab4e06f6f880fd/PV-E02/results.json'
r = json.loads(SOURCE.read_text())
assert r['complete'] and len(r['rows']) == 32
rows = r['rows']
worlds = list(dict.fromkeys(row['world'] for row in rows))
styles = {('lbfgsb', 'full'): ('#ae4428', 'o', '-'),
          ('lbfgsb', 'half'): ('#d89070', 'x', '--'),
          ('trf', 'full'): ('#155b91', 'o', '-'),
          ('trf', 'half'): ('#79aac9', 'x', '--')}
fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), layout='constrained')
for (method, mode), (color, marker, line) in styles.items():
    subset = [row for row in rows if row['method'] == method and row['point_mode'] == mode]
    label = f'{method}, {mode} points'
    axes[0].plot(range(len(worlds)), [row['anchor_rmse_mm'] for row in subset],
                 color=color, marker=marker, linestyle=line, label=label, linewidth=1.3)
    xs = [row['unseen_prediction']['rms_mm'] for row in subset]
    ys = [row['anchor_rmse_mm'] for row in subset]
    axes[1].scatter(xs, ys, c=color, marker=marker, label=label, s=34)
axes[0].axvspan(3.5, 7.5, color='#e6e6e6', alpha=.6)
axes[0].set_xticks(range(len(worlds)), worlds, rotation=40, ha='right')
axes[0].set_yscale('log')
axes[0].set_ylabel('Anchor RMSE, mm, logarithmic scale')
axes[0].set_title('Paired worlds, holdout worlds shaded')
axes[1].set_xscale('log')
axes[1].set_yscale('log')
axes[1].set_xlabel('Unseen encoder residual RMS, mm, logarithmic scale')
axes[1].set_ylabel('Anchor RMSE, mm, logarithmic scale')
axes[1].set_title('Prediction residual versus geometric error')
for ax in axes:
    ax.grid(True, which='major', alpha=.2)
axes[1].legend(fontsize=8, loc='upper left')
fig.suptitle('HP3 analytical screen, 32 individual core fits', fontsize=13)
fig.supxlabel('Prescribed geometry, compliance, reversal offsets and noise. No native calibration or hardware accuracy claim.', fontsize=9)
for ext in ('png', 'svg'):
    fig.savefig(ROOT / f'analytical-screen.{ext}', dpi=180)
print(ROOT / 'analytical-screen.png')
