# HP3 comfortable force tuning and PJRT warnings

Objective: fix the max-force definition and JAX warning spam before another manual reference run.
Hypotheses: chasing a travel plateau selects excessive force; automatically enabling persistent JAX caching causes the native warning.
Constraints: small code changes; preserve supplied reference recordings; use local Python.
Budget: offline reference replay, one isolated no-buildup HP3 tuning run, targeted collector/JAX tests. No hardware commands or new full calibration.
Experiments: E1 reference-curve replay; E2 isolated headless HP3 tuner; E3 JAX default versus warm disk cache; E4 regression suites.
Accepted steering: user will repeat full manual collection; do not claim improved calibration accuracy from tuning alone.
Next decision: compare measured limits and check warnings after removing automatic disk-cache activation.

Completed: E1 and E2 both selected 2.3957 N. E3 confirmed warm disk-cache warning emission and clean defaults. E4 passed 95 collector and 10 JAX tests. Preserved the initial socket-permission failure and corrected slip-test mock failure. No further experiment is needed for the two requested fixes; next decision belongs to the user's fresh full manual calibration.

Follow-up: [full cache timing investigation](../autocal-jax-cache-20261010/report.md) measured a 43.64% warm-cache saving and restored caching by default with overridable native logging. The earlier opt-in decision described here was provisional and is superseded.
