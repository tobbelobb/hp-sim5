"""Rebuild exportable figures from the stored simulation evidence."""
import json
from pathlib import Path
from decimal import Decimal, ROUND_HALF_UP

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})


def phase_figure():
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.2), constrained_layout=True)
    for ax, name, title in zip(axes, ['hp3-final', 'hp4-final'], ['Nominal HP3', 'Nominal HP4']):
        data = json.loads((ROOT/name/'sweeps.json').read_text())
        rows = [(i,s,p) for i,s in enumerate(data['sweeps']) for p in s['data_points']]
        sensors = np.array([p.get('source_sensor_anchor', s['sensor_anchor']) for _,s,p in rows])
        all_angles = np.array([(p.get('mu') or p['raw_angles_deg'])[a] for (_,_,p),a in zip(rows,sensors)])
        forces = np.load(ROOT/name/'physics-samples.npz')['effective_force_proxy_n']
        error = forces[np.arange(len(rows)), sensors] - np.array([p['sensor_force_n'] for _,_,p in rows])
        select = sensors == 1
        phase = (50 * all_angles[select]) % 360
        even = np.array([i % 2 == 0 for i,_,_ in rows])[select]
        ax.scatter(phase[even], error[select][even], label='Training sweeps', c='#858e9b', s=23, alpha=.75)
        ax.scatter(phase[~even], error[select][~even], label='Held-out sweeps', c='#007c85', s=30)
        fit = next(row for row in json.loads((ROOT/name/'force-phase.json').read_text())['rows'] if row['sensor_anchor']==1)
        x = np.linspace(0,360,300)
        y = fit['force_sine_coefficient_n']*np.sin(np.deg2rad(x)) + fit['force_cosine_coefficient_n']*np.cos(np.deg2rad(x)) + fit['force_offset_n']
        ax.plot(x, y, c='#14283f', label='Phase model', linewidth=1.5)
        ax.set(title=title+' · sensor B', xlabel='Motor electrical phase (degrees)', ylabel='Actual force proxy − command (N)', xlim=(0,360))
        ax.grid(alpha=.15)
    axes[0].legend(fontsize=9, loc='lower left')
    fig.savefig(ROOT/'force-phase.png', dpi=180)
    plt.close(fig)


def accuracy_figure():
    cases = ['hp3-final','hp3-soft-verified','hp4-final','hp4-worn-verified']
    if any(not (ROOT/name/'summary.json').exists() for name in cases):
        return
    summaries = [json.loads((ROOT/name/'summary.json').read_text()) for name in cases]
    old = np.array([[1.68,1.58],[4.60,2.16],[4.82,2.83],[2.50,1.055]])
    new = np.array([[d['anchors']['max_abs_coordinate_mm'],d['anchors']['max_aligned_abs_coordinate_mm']]
                    if d.get('normal_completion') else [np.nan,np.nan] for d in summaries])
    fig, axes = plt.subplots(1,2,figsize=(10.4,4.4),constrained_layout=True)
    x = np.arange(4)
    for j, ax in enumerate(axes):
        for delta, values, color, label in [(-.18,old[:,j],'#8f9aaa','Earlier hardened'),(.18,new[:,j],'#007c85','Expanded workspace')]:
            bars = ax.bar(x+delta, values, width=.34, color=color, label=label)
            labels = [str(Decimal(f'{v:.6f}').quantize(Decimal('.01'), rounding=ROUND_HALF_UP))
                      if np.isfinite(v) else '' for v in values]
            ax.bar_label(bars, labels=labels, fontsize=10, padding=3)
            for i, value in enumerate(values):
                if not np.isfinite(value):
                    ax.text(x[i]+delta, .1, 'Stopped', rotation=90, ha='center', va='bottom', fontsize=9)
        ax.axhline(1., color='#ac442f', linestyle='--', linewidth=1.3, label='1 mm coordinate target')
        ax.set_xticks(x, ['HP3','HP3\nsoft lines','HP4','HP4\nworn bearings'])
        ax.set_ylabel('Worst coordinate error (mm)')
        ax.set_title(['Absolute coordinates','After removing common translation'][j])
        ax.set_ylim(0, max(old.max(),np.nanmax(new))*1.18)
        ax.grid(axis='y',alpha=.15)
        ax.set_axisbelow(True)
    fig.legend(*axes[0].get_legend_handles_labels(), loc='outside lower center', ncol=3, fontsize=9)
    fig.savefig(ROOT/'accuracy.png',dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    phase_figure()
    accuracy_figure()
