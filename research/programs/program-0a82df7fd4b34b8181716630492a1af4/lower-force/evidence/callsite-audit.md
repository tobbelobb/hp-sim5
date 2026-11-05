# Force-helper call-site audit at a68d6adf

The isolated candidate changes only resolveCollectionForce's default multiplier5 to4. All three collector files were copied from current upstream and every import resolved against the current worktree, preserving component class identity.

- primitives/uncalibrated_actions.mjs:377 uses the helper for actual measurementSensorForce. The explicit sensorCollectionForce override has first priority. buildDataPointModes keeps direct2*forceMid preloads.
- behaviors/sweep_data_collection.mjs:276,1142,1482,1541,1665,1734 pass explicit preloadMultiplier20 for pullouts. The candidate leaves those unchanged.
- behaviors/sweep_data_collection.mjs:1204 records per-point sensor_force_n. Lines1499 and1704 record force selection and sweep metadata. Line1664 records directionForces via resolveCollectionForce(sweepResult.forces), including recovered force values. These use the same default helper as actual measurement commands and change consistently in the isolated candidate.
- CLI collect_sweep_data.mjs calls through the copied sweep behavior and uncalibrated helper. All other imported modules remain current upstream.
- Unit tests fixed_anchor_guard.test.mjs cover the default helper and explicit20/override/fallback cases. These are unchanged in research-only copies.

The research harness runs only hangprinter_3. Changing this generic production default globally would also affect HP4 and slideprinter and is outside evidence scope. Production adoption, if justified after evaluation, must carry HP3 identity into measurement resolution and all metadata, preserve explicit overrides and explicit20 pullouts, and keep untested machine defaults5. No production change has been made.
