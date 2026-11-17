"""Read archived integrated-HP5 runs; never launch a simulator or modify a run."""
import json,re,sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
ROOT=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent/'runs'
TRUTH=np.array([[0.,-1900.,-280.],[1900.,0.,-280.],[0.,1900.,-280.],[-1900.,0.,-280.],[0.,0.,1900.]])
results={}
for phase in ('pass1','pass2','pass3'):
    results[phase]={}
    for machine in ('direct','reduced'):
        folder=ROOT/phase/machine
        data=json.loads((folder/'sweeps.json').read_text())
        manifest=json.loads((folder/'sweeps.headless/manifest.json').read_text())
        rows=[json.loads(line) for line in (folder/'sweeps.full_auto_log.jsonl').read_text().splitlines()]
        iterations=[row for row in rows if 'iteration' in row]
        command=next(c for c in manifest['applied_parameters'] if c.startswith('M669 '))
        anchors=np.array([[float(v) for v in re.search(r'\b'+axis+r'([-+\d.eE:]+)',command)[1].split(':')] for axis in 'ABCDI'])
        delta=anchors-TRUTH
        points=[point for sweep in data['sweeps'] for point in sweep['data_points']]
        extras=[extra for point in points for extra in point.get('extra_sensor_ranges',[])]
        first_good=None;trajectory=[]
        for i,row in enumerate(iterations):
            metrics=next(run['metrics'] for run in row['runs'] if run['id']==row['selected_run'])
            error=float(np.max(abs(np.array(metrics['pose_fit_anchors'])-TRUTH)))
            sweeps=len(data['sweeps'])-len(iterations)+i+1
            trajectory.append(dict(iteration=row['iteration'],sweeps=sweeps,max_coordinate_error_mm=error,
                rms_mm=metrics['pose_fit_rms_mm'],max_std_mm=metrics['max_std_mm'],decision=row['decision']))
            if error<=2 and first_good is None:
                first_good={'sweeps':sweeps,'max_coordinate_error_mm':error}
                if phase!='pass3':
                    started=json.loads((folder/'run-start.json').read_text())['start_unix']
                    logged=datetime.fromisoformat(row['timestamp']).replace(tzinfo=ZoneInfo('Europe/Stockholm')).timestamp()
                    first_good['elapsed_wall_s']=logged-started
        results[phase][machine]=dict(normal_completion=manifest['normal_completion'],stop_reason=manifest['stop_reason'],
            sweeps=len(data['sweeps']),points=len(points),accepted_extra=sum(e['accepted'] for e in extras),
            applied_max_coordinate_error_mm=float(abs(delta).max()),
            applied_aligned_max_coordinate_error_mm=float(abs(delta-delta.mean(axis=0)).max()),
            wall_s=manifest['wall_s'],simulated_s=manifest['simulated_s'],
            first_logged_refinement_within_2mm=first_good,trajectory=trajectory)
print(json.dumps(results,indent=2))
