# Autocal

Current state of the hp-sim auto calibration art:

Collect data in a nice pattern with:
```
node scripts/collect_sweep_data.mjs --sweep-method torque-ramp --speedup 4 --trace --torque-low 0.03 --torque-min 0.03 --torque-max 0.3 --torque-step 0.05 --feed 400 --superSweepRange 600 --superSweepPoints 4
```

Compare the ellipse and point optimization methods with:
```
python autocal/calibrate.py ellipse autocal/data/big_even_pattern.json
python autocal/calibrate.py point autocal/data/big_even_pattern.json
```
