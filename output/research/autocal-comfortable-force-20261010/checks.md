# E4 regression checks

- `npm test -- --runInBand autocal/control/tests/behaviors autocal/control/tests/primitives autocal/control/tests/cli`: 19 suites, 95 tests passed.
- `.venv/bin/python -m pytest autocal/tests/test_jax_cache_settings.py autocal/tests/test_ellipse_objective_jax.py -q`: 10 passed in 7.74 seconds.
- `git diff --check`: passed.
- Initial slip test mock incorrectly reset encoders on position hold, hiding the return move. Corrected the mock to retain encoder angles until commanded return; the final suite passed.
