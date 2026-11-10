# Verification

- `.venv/bin/python -m pytest autocal/tests/test_jax_cache_settings.py autocal/tests/test_ellipse_objective_jax.py -q`: 12 passed in 12.98 seconds.
- Fresh-process JIT tests cover the restored default cache, explicit disabling, custom directory/compile threshold, quiet native logging, verbose override, visible Python warnings and unrelated stderr.
- `warning-repro.json`: the same warm-cache JIT emitted three PJRT warnings at native level 1, zero at level 2, and three after restoring level 1; every output was 100.0 and every process exited zero.
- Regression batteries use the production `regress_calibration_logs.py` main entry point, observed without changing its solver, fixtures, options or scheduling. Per-child complete stdout/stderr is saved alongside numerical summaries and iterations.
