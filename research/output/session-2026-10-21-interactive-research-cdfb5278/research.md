# HP4 Print Squares substep and PR 77 review

Objective: Review PR #77 against current main, reproduce the greater missed-step counts at finer substeps, and make a useful validated commit.

Accepted steering: User requests experiments and implementation, with small source changes. Headless production native/Wasm backends only. Baseline revision 2c0b86ff; PR head fcdaf7fed43a4cb1a9efdd289a58980ea2840984, physics commit 63b847c8. The attached fresh supervisor world is not needed for these isolated replays.

Hypotheses:
- H1: Open-loop effective-inertia inflation violates static torque balance. Analytic spring-loaded spool can disprove this claim.
- H2: Standalone axis-only rotor displacement is double-counted in cable damping. A pure-damper implicit update can disprove this claim.
- H3: Finer-substep squares pull-out persists after both fixes; one substep conceals it.
- H4: Both PR corrections can safely land together on current main. The mandatory default-one-substep logo and loaded-state gate can disprove this claim.

Constraints and budgets: Up to 20 short fresh-world replay cases, each at most 20 simulated seconds; at most 16 supported physics substeps. No autocalibration, hardware motion, live collection or browser experiments. Numerical recordings only (no RRD) for deterministic cheap comparisons; retain source hashes, command inputs, JSON samples and failed test logs. Initial experiment compute budget 10 CPU minutes, reassess before a longer run. Actual execution wall time is recorded separately from prior recording benchmarks.

Experiments:
- E01: Baseline PR analytic torque/damping tests, failed log retained in E01-baseline-analytic.log.
- E02: Baseline native bundled Squares startup/early motion at 1,2,4,8,16 substeps (planned).
- E03: Both corrections, analytic checks and same replay (planned).
- E04: Mandatory npm run test:physics gate against candidate (planned).

Prior evidence consulted: research/hp4-logo-stability.md and research/faster-motor-update.md; PR body and diff archived in pr77-review-input.json. Research index torque search links earlier HP4 motor-load and substep sessions. Command socket probe passed socket creation, loopback bind and HTTP round trip; this is tool health only, not a dynamics result.

Next decision: Compare analytic failures and baseline motion; test minimal PR physics corrections before selecting the delivery scope. No conclusion about hardware or calibration accuracy is available.

## E01–E04 findings and updated decision

E01: All 30 baseline analytic cases fail. The 1-substep JS spring equilibrium is 0.00238600218 rad versus the analytic 0.00594649047 rad. Both static balance and pure damping defects are reproduced across JS, Wasm cable kernel and full Wasm.

E02: At 0.1/2 s, total peak missed magnetic periods are 0/0, 0/0, 2/2, 3/3, 3/3 for 1,2,4,8,16 substeps. Native production commands and engine hashes are in E02-baseline.json. B phase lag at 0.1 s is -0.01095, 0.00668, 0.25306, 0.24604, 0.24647 rad respectively.

E03: Both PR corrections pass all 30 analytic checks. Squares still reports the same total counts on current main (not the PR's old two-loss result at every finer setting); B phase at 0.1 s is -0.01043, 0.00640, 0.25213, 0.24698, 0.24650 rad. Source patch preserved as E03-full-physics-candidate.patch. This reproduces timestep sensitivity, not a hardware pull-out threshold.

E04: Mandatory gate FAILS loaded-state settling at ONE substep, with 0.00500792772 m vertical range in its last five simulated seconds. Logo/native/Wasm/JS and other substeps pass, but cannot substitute for the failure. E04-candidate-physics-gate.log retained. H4 rejected for simply transplanting both corrections.

E05 underway: restore open-loop effective inertia, retain only standalone rotor damping correction, run the same unmodified mandatory gate. Next decision depends on this separation. If it fails, do not relax limits or increase default substeps; investigate consistent motor/cable integration or choose a narrower non-dynamics commit.

## Final scope and validation

E05 rejects damping-only integration with the same 5.008 mm settling failure as E04. E06 restores original damping and removes only open-loop cable stiffness: all 11 mandatory gate cases pass. This isolates the destabilizing change without altering the gate. Final source scope: physical inertia correction in JS/Python/Rust, shared Warp preparation, retired ABI option slot, rebuilt Wasm, analytic fixture/tests, CI wiring and research documentation. No HUD change.

E07 adds native analytic coverage. Initial exact multi-projection assertions fail at coarse timesteps; a separate 1/2 s duration audit shows a small non-transient projection error (up to 7.34e-8 rad angle / 3.76e-8 Nm torque residual). H6 exact projection invariance rejected. Final independent tests use one projection to isolate the original torque bias, retaining strict original tolerances; 20 JS/Rust paths/timesteps pass, 117 Python/Warp checks pass (5 GPU skips), 141 backend/mounted/substep checks pass (1 GC skip), and full Jest passes 801 (1 skip). Initial Warp cache failures are preserved and resolved with WARP_CACHE_PATH under /tmp.

E08: retained native Squares at 1/4 substeps for 20 s stays bounded; zero/two B periods persist. This is a 17-case motion replay total, within the initial 20-case cap. No live world or hardware was advanced. Compute stayed within the intended order of the 10 CPU-minute initial budget; no timed speedup claim is made. All tests and experiments are complete. Accepted decision: land only the inertia correction, explicitly defer damping and HUD work. Next research decision: resolve damping-only loaded-state failure with a consistent coupled motor/cable formulation; no further experiment is planned in this turn.

## Commit artifact

The original repository’s `.git` is mounted read-only by this workspace. Creating `fix/hp4-open-loop-torque-balance` there failed before any ref/index mutation. The tested tree was committed through a separate Git directory inside this session as `838133d4ed03662a55581322367a9c309ac47e38`. The [complete commit patch](hp4-open-loop-torque-balance.patch) and [incremental Git bundle](hp4-open-loop-torque-balance.bundle) are ready to import; the bundle depends on original main `2c0b86ff189d27859b83543ed3fa837cfa658a3b`. Original working-tree source edits remain present and uncommitted. No remote branch was created or message posted.
