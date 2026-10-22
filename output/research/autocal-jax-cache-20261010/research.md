# Autocal disk-cache timing and selective warning suppression

Objective: measure full calibration-log regression batteries with cache on/off, retain caching if it saves time without numerical changes, and silence the specific PJRT warning.
Accepted steering: user prefers preserving useful caching; the previous opt-in default is provisional and should be reversed if evidence supports it.
Hypotheses: warm persistent caching reduces end-to-end replay time; numerical outputs are unchanged; the PJRT warning is an unconditional backend compatibility notice.
Constraints: .venv/bin/python; unchanged regression runner and datasets; no simulator movement; no dependency upgrades for the comparison.
Budget: one cold cache population battery plus two interleaved pairs (off/on), all 11 datasets and normal default concurrency; targeted warning/cache tests. Preserve failures. No hardware or full online collection.
Experiment IDs: C1 cold-on; C2 off; C3 warm-on; C4 off; C5 warm-on. D1 inspect upstream warning source; D2 verify selective suppression.
Status: in progress. No performance result yet. Next decision: compare timing and complete numerical summaries/iterations before changing defaults.

Interim C1/C2/C3: 83.071s cold-on, 97.823s off, 53.676s warm-on; all 33 calibrations exited zero, all batteries passed references, parsed final and per-iteration numerical outputs exactly identical. Restored default disk cache and added overridable TF_CPP_MIN_LOG_LEVEL=2 before JAX imports. This suppresses all native INFO/WARNING logs, not only the specific PJRT line; errors and Python warnings remain visible. No dependency upgrade or stderr interception.
C4 off repetition runs while the small targeted cache/JAX test suite executes; preserve this timing caveat. C5 warm repetition will use final production defaults and verify warning counts. Next decision: complete repeats, tests, numerical comparison, and final report.

C4 completed: 96.478s, all 11 references passed, zero failures and warnings. This is close to C2 despite the small concurrent test workload. C5 runs alone using final cache/logging defaults. No extra battery is necessary: the measured warm-cache benefit is large relative to timing variation, with repeated full-output equality checks.

Completed C5: 55.829s, all 11 references passed, no child failures and zero native compatibility warnings. All five batteries had identical parsed numerical outputs. Final cache-off mean 97.151s; warm-on mean 54.753s; 42.398s / 43.641% saved. Kept caching on, made native severity setting overridable, documented its broad INFO/WARNING scope. No further benchmark is required for this decision.
