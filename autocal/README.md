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

Combine datasets with:
```
python autocal/active_calibrate.py merge autocal/data/big_even_pattern.json sweep_data_slideprinter_1766060855696.json -o autocal/data/big_even_pattern_active.json
```

Create a new optimal command to collect one more sweep witht he most valuable data to the ellipse algorithm:
```
python autocal/active_calibrate.py ellipse autocal/data/big_even_pattern_active.json --collector-args --speedup 4 --trace --torque-low 0.03 --torque-min 0.03 --torque-max 0.3 --torque-step 0.05 --feed 400
```

Run this "combine" -> "suggest" -> "collect data" loop automatically:
```
python autocal/active_calibrate.py ellipse-loop autocal/data/big_even_pattern.json --work-dataset autocal/data/big_even_pattern_active.json --collector-args --speedup 4 --trace --torque-low 0.03 --torque-min 0.03 --torque-max 0.3 --torque-step 0.05 --feed 400
```

If you don't have a starting point data set you can just get started with
```
python autocal/active_calibrate.py ellipse-loop --work-dataset autocal/data/my_active.json --collector-args --speedup 4 --trace --torque-low 0.03 --torque-min 0.03 --torque-max 0.3 --torque-step 0.05 --feed 400
```
