import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
OUT=Path(__file__).resolve().parent
thin=json.loads((OUT/'E02-comparison.json').read_text());native=json.loads((OUT/'E08-native-comparison.json').read_text());reg=json.loads((OUT/'E09-regressions.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(12.5,4),layout='constrained')
a=axes[0];x=np.arange(4);w=.34
a.bar(x-w/2,[r['baseline']['anchor_rmse_mm'] for r in thin],w,label='Full points',color='#365f94');a.bar(x+w/2,[r['half']['anchor_rmse_mm'] for r in thin],w,label='Half points',color='#cb7644');a.set_xticks(x,['Bad points','10-point','HP3 seventh','HP3 April'],rotation=22);a.set_ylabel('Anchor RMSE (mm)');a.set_title('Thinning worsens 3 of 4 fixtures');a.legend(fontsize=8)
a=axes[1];labels=['Baseline','Recent window','Guarded window'];v=[native[k]['simulated_s'] for k in ['baseline','unguarded','final_guarded']];bars=a.bar(labels,v,color=['#365f94','#aaa','#4f8d65']);a.bar_label(bars,fmt='%.1f s',padding=3);a.set_ylim(0,480);a.set_ylabel('Simulated collection time (s)');a.set_title('Same 36 points; guarded gain 25.4%');a.tick_params(axis='x',labelrotation=18)
a=axes[2];v=[json.loads((OUT/p).read_text())['max_drift_to_later_recorded_end_deg'] for p in ['E06-recorded-settling.json','E06-recorded-rate-settling.json']];bars=a.bar(['Recent window','Guarded window'],v,color=['#aaa','#4f8d65']);a.bar_label(bars,fmt='%.3f°',padding=3);a.set_ylabel('Max later encoder drift (degrees)');a.set_title('Trace replay motivates drift guard');a.set_ylim(0,2.2);a.text(.75,.82,'118 vs 114 pairs\n3 vs 7 censored',transform=a.transAxes,va='top',ha='center',fontsize=8);a.tick_params(axis='x',labelrotation=18)
for a in axes:a.spines[['top','right']].set_visible(False)
fig.savefig(OUT/'results.png',dpi=180);fig.savefig(OUT/'results.pdf')
