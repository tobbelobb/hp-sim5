# Fuller Rust simulation engine

Status: complete.

Objective: move the shared hp-sim-3d physics pipeline into a continuing Rust world, default to it in browser/headless/profile consumers, and keep the same core buildable for native execution.

Hypotheses:
- E01: persistent Rust state and physics systems improve throughput beyond the old Wasm cable kernel.
- E02: Rust math retains close physical trajectories without calling JavaScript trig functions.
- E03: the core runs as a native Rust library without browser or JS dependencies.

Constraints: retain authored geometry, fixed 0.002 s RRF clock, solver iterations, motor modes, layering and cancellation boundaries. JavaScript retains scene parsing, commands/transport, edits, observations and rendering. No hardware movement or live adaptive collection is requested. Use frozen logo and autocal traces from the 2026-10-13 profiling session. Run plain timing processes sequentially, without overlapping tests/profiles.

Development evidence: 76 initial differential/pipeline Node tests passed; three native core tests passed. A provisional 20,000-step sinusoidal trial measured 2.60x HP3 and 4.91x HP4 against JS. These exploratory results are not the final performance matrix. Changes since those trials include correct knot initialization, shared-joint handling and source fingerprints. Failed trials (knot initialization, dt resource vs update argument, and observation shape) were corrected; verification logs are retained in this directory when final checks finish.

Next decision: finish integration tests, freeze the source and binary, then measure the original workloads against the previous default and current JS. Report numerical errors and host limitations along with throughput.

E01 first matrix trial: layered HP3 logo was 2.39x faster than the previous kernel. Unlayered HP3 logo failed the initial pointwise tolerance. Its maximum sampled position/point difference was about 4 mm, encoder difference about 0.002 rad, and force difference up to 1.55 N. This is not accepted as pointwise parity. E02 one-step fork trial: synchronize the Rust world from the exact JS state before each of 3,000 frozen unlayered HP3 logo steps. Maximum one-step component difference was 2.863e-12 N, with no error above 1e-8. The long feedback trajectory amplifies small arithmetic differences; physics equations agree in this bounded test. Evidence: one-step-forks.json and matrix.log/matrix/.

The profiler also retained references to live velocity components in earlier checkpoints. Those now become detached values at observation time in both comparison harnesses. This changes velocity checkpoint hashes, not stepping. The final matrix uses report-only numerical drift after this explicit review; failed tolerance checks remain false and retain error statistics. Clocks, IDs, command metadata, structures and finite values remain mandatory. Core source/binary have not changed between the failed matrix and this review.

Final outcome: all consumers use the full Rust engine; native builds/tests pass. E01 supported: final full-engine medians improve by 2.37–2.86x against the previous default. E02 mixed: bounded one-step/short-scene checks agree tightly; five long replay cases fail pointwise tolerance, with errors retained. E03 supported: identical core source builds as rlib/cdylib and runs three native tests. The final coefficient guard and matrix/friction arithmetic changes were remeasured (24 full-engine runs); all stock trajectory hashes match the preceding full-engine matrix. No further physical source changes remain planned. No live adaptive calibration or hardware result is claimed.

The late-fork copy helper initially failed on a null extruder observation field; that failure is retained in late-forks-first-failure.txt. The corrected trial compared 1,000 steps beginning at step 260,000 and measured a 4.585e-15 rad/s maximum difference. The final precision matrix is complete; source/binary are frozen under final-source/.
