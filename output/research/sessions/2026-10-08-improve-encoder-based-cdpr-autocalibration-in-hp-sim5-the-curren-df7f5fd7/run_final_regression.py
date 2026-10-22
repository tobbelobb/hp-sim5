from pathlib import Path
import subprocess,time,json,sys
s=Path(__file__).resolve().parent
command=[sys.executable,'autocal/tools/regress_calibration_logs.py','--jobs','2','--keep-going','--no-fail-score-mismatch','--color','never']
started=time.perf_counter()
with (s/'E08-regression.log').open('w') as log:
    run=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
(s/'E08-results.json').write_text(json.dumps(dict(command=command,seconds=time.perf_counter()-started,returncode=run.returncode),indent=2))
print(run.returncode)
