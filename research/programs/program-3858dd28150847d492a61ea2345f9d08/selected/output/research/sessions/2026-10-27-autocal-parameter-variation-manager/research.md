# HP3 autocal parameter variation program

Program: program-3858dd28150847d492a61ea2345f9d08

Status: in progress. No new scientific result yet.

Objective: improve encoder-only HP3 calibration speed and robustness across plausible physical setups, preserving one-click operation and automatic stopping. The user allows two hours of simulation/compute and requests cloud environment repairs without permission questions.

Budget started approximately 2026-10-27 08:01 UTC. Stop numerical work by 10:01 UTC. The finite controller allocation runs 105 minutes from 08:04 UTC with 190 seconds cleanup, two tasks, one worker, and separate 10-minute review reservations. First worker contract is 55 minutes, at most 32 fresh fits, four collections and four million physics steps. These numerical limits are worker contracts, not aggregate command meters.

Constraints: HP3 only, fixed known radii, buildup zero, simulator layering disabled. Ground truth may evaluate simulated results offline but may never select deployed candidates. Use headless native production physics, no recording for large numerical sweeps and events for routine collection. No physics changes planned.

Competing explanations:

- Nominal-world overfitting hides useful improvements that a cheap varied-world test can identify.
- Force-dependent model inadequacy requires staged inference or new measurements.
- Existing candidates contain the answer but encoder-only ranking selects poorly.

The first discriminator will reproduce ranking mismatch on available recorded HP3 data, inspect relevant earlier negative results, then test a small candidate on explicit parameter variation. A useful evaluation tool is an acceptable retained change if calibration rules regress. The supplied reference path output/extended-autocal/no-buildup-reference-003 is absent in this checkout. Its provenance or a preserved equivalent must be investigated; no direct victory over that run may be claimed without its data.

Environment observations:

- Managed cloud environment became connected/running. Network policy is enforced and unrestricted, with no configured secrets, runtime variables, outbound identities or VPN.
- Controller runtime_check passed Python dependencies, Node dependencies, Rust/Cargo/LLVM tools, native physics, firmware and launcher socket checks. No installation repair is indicated by these checks.
- Independent agent-command socket creation, loopback binding and HTTP round-trip passed in 0.015 seconds. This establishes tool health, not calibration accuracy.
- Old cloud bridge requests from October 24 are completed/cancelled historical records. They were not redispatched.

Worker attempt: attempt-3b2e360ec1834eee9bab4e06f6f880fd, assignment variation-first. Review is pending. Next decision: inspect the worker's concrete experiment plan and evidence, then submit fixed evidence to a fresh reviewer.

## 08:14 UTC update

The investigator predeclared eight analytical worlds, four development and four holdout, with fixed world specs before fitting. PV-E02 completed 32 individual core fits. Its measured loop took 118.24 seconds and analytical generation took 0.022 seconds. These times exclude full calibration, collection and physical dynamics. A plot of saved rows is [analytical-screen.png](analytical-screen.png); [plot_results.py](plot_results.py) runs no fits.

The matrix does not support a default solver or thinning change. In one clean world, full-data L-BFGS-B predicts unseen measurements at 0.147 mm RMS with 1883.69 mm anchor RMSE. Full-data TRF gives 5.89 mm anchor RMSE, while half-data TRF gives 40.20 mm and slightly lower unseen residual. Other varied cases also regress. These are core-fit outcomes from a data-derived cold seed, not final full-auto consensus errors. The complete coverage and validity criteria must be checked before admitting a result.

PV-E03 completed two 2,000-step, 4-second native Rust stiffness probes, with identical commands and 20,000 vs 8,000 N axial stiffness. They are sensitivity probes, not complete calibration or a numerical validation of the analytical error model. Exact command and scene hashes are retained by the worker.

The cloud resource snapshot shows a two-CPU quota and 8 GiB memory limit. BLAS and OMP threads are both one. No dependency, socket or process environment failure has appeared. Direct publisher web requests returned HTTP 429; the literature note uses available indexed text and an institutional abstract and records this retrieval limit.

No exact reference-003 dataset hash match was found in 218 candidate JSON files, totalling 9.7 MB. This is a bounded archive search, not proof that no equivalent observations exist anywhere. Historical reports preserve the reference's 2.828 mm result but cannot replace its missing raw data.

Next decision: freeze the worker's evidence and candidate, obtain a separate review, and decide whether a targeted native collection followup would resolve the remaining analytical-to-production gap.

## Cloud actor interruption

After creating candidate cc52307f and reports, the actual cloud worker ended with the platform error "Selected model is at capacity" before writing handoff.json/final-message.md. The parent did not fabricate a child response or acknowledge successful bridge completion. It resumed the same actor and original bounded attempt solely to finish the handoff. No model, authentication, dependency or controller mode was changed, and no new numerical experiment was commissioned by this recovery. The original measurements and commit remain intact.

## Cloud review path repair

The first review guard exited with code 2 before creating a cloud request. Its stderr was "Final message must be inside repository output". The controller's normal default store places reviewer responses under .research-programs/reviews, outside output. This was a real path-validation mismatch in scripts/cloud/cloud_client_bridge.py.

Commit ae0b20c4 adds a narrow allowance for final-message.md inside repository-contained .research-programs*/reviews/review-run-<32 hex digits> paths, after symlink resolution. Existing output paths remain accepted. Tests reject control files, malformed run names, paths outside the checkout and review-store symlink escapes. The exact default-store completion fixture fails before the patch; 51 transport/review-controller checks pass after it. Logs are cloud-review-path-before.log and cloud-review-path-after.log. These fixture checks are distinct from real model dispatch.

The failed review consumed its reserved allowance. The manager stopped the completed worker allocation and explicitly funded a new 10-minute review allowance. The retry created bridge request 697c5e9996044784bbecc1eb24088011 in the default store and was bound to actual fresh reviewer /root/autocal_screen_reviewer. This verifies the corrected launch boundary. Its evidence assessment remains pending. No store, authentication, model or historical evidence was replaced to bypass the error.
