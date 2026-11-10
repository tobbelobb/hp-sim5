# Encoder geometry numerical checks

These standalone ideal range-difference experiments accompany the HP3 autocal research session `attempt-c18177d871fb404895e32c03363c8a5e`. They use revision `9de0ac2ae01edf2a879c63e42ef269419d349cf0` and NumPy/SciPy from the repository `.venv`; set `OPENBLAS_NUM_THREADS=1`.

`gauge.py` gives a full-rank known-radius local Jacobian for one generic tetrahedral configuration and an exact scale null when the common radius is also unknown. It also probes noise, axis-dependent bias and a separately assumed exact span. The span is a theoretical contrast, not an operational calibration measurement. `gauge-results.txt` preserves the output. The script prints two consecutive JSON objects.

`heldout_design.py` compares narrow and mixed-coverage ideal position sets under independent 0.03 mm range-difference noise for seeds 101–110. Median maximum anchor error falls from 24.09 to 0.826 mm; the mixed design still misses 1 mm in four of ten cases. `heldout-design.json` contains each anchor error. These points do not enforce physical cable motion or tension, and local fits initialize near the generating geometry. This is an information experiment, not a field or end-to-end accuracy claim.

Run from the repository root:

```sh
OPENBLAS_NUM_THREADS=1 .venv/bin/python research/encoder-observability-2026-11-06/gauge.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python research/encoder-observability-2026-11-06/heldout_design.py
```
