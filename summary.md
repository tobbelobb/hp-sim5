# Autocal benchmark history

Accuracy is maximum applied anchor coordinate error in mm. Precision is the largest split-fit anchor coordinate difference in mm. Convergence is simulated seconds to within 2 mm of truth. Total is simulated seconds for the whole run.

Rows group measurements by tested commit and scoring protocol. Values are medians of successful runs. Convergence requires a majority to report a time. ▲ marks a review threshold crossed since the preceding successful input under the same protocol. Individual results, wall times, and failures are in `results.jsonl`.

## hangprinter_3

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 7a1b0e84c1 | v2 daf2711f048f74674685 | 2026-11-18T21:19 | 1/1 | 0.96 | 1.22 | 3574 | **5265 ▲** | 9 | stable-estimate | ok | Merge pull request #117 from tobbelobb/t3/fix-worker-quality-history-timing |
| f373f1f010 | v2 a86ca215a4df1b6c8760 | 2026-11-18T21:17 | 1/1 | 0.96 | 2.27 | 3574 | 4547 | 7 | stable-estimate | ok | Merge pull request #114 from tobbelobb/t3/isolate-recorder-failures |
| f2be134944 | v2 83c9b259b9ba93896b0a | 2026-11-18T21:16 | 1/1 | 0.96 | 2.27 | 3574 | 4547 | 7 | stable-estimate | ok | Merge pull request #113 from tobbelobb/t3/fix-noop-payload-resume |
| 20da983feb | v2 5cb6a2120b678a5e1b10 | 2026-11-18T18:17 | 1/1 | 0.96 | 2.27 | 3574 | 4547 | 7 | stable-estimate | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 21a14da94822c449f3df | 2026-11-18T16:53 | 1/1 | 0.96 | 1.22 | 3574 | 5265 | 9 | stable-estimate | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |
| 0623060e22 | v1 918282e8dd16861d93d9 | 2026-11-18T11:48 | 1/1 | 0.96 | 1.23 | 3574 | 5255 | 9 | stable-estimate | ok (legacy) | Track autocal benchmark per machine and warn on regressions |
| ce4afd7eb4 | v1 06db97d84a35ba5aa433 | 2026-11-18T11:39 | 1/1 | 0.96 | 1.23 | 3574 | 5255 | 9 | stable-estimate | ok (legacy) | Merge pull request #101 from tobbelobb/t3/rrf-host-standstill-bookkeeping |
| 97091d345c | v1 a022a67f9914a70868d5 | 2026-11-18T11:39 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Merge pull request #100 from tobbelobb/t3/fix-ambient-layering-plan |
| 13433a9eb5 | v1 2d946c3b5cbb6748c8fd | 2026-11-18T11:12 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Keep one browser collector session across autocal sweeps |
| 99c8820d66 | v1 6d3d0ef8d007f46b5e5b | 2026-11-18T10:28 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Honor equal-strain collection in fresh HP5 bootstraps |
| 0652a47b95 | v1 59a27dffd43f5c1131c2 | 2026-11-18T10:27 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Read the spare HP5 encoder during ordinary sweep motion |
| 6826087176 | v1 db2dae9cd5e2d3ec6092 | 2026-11-17T16:57 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Use HP5 spare ranges throughout calibration and stop after agreeing good fits |
| 089a7ad371 | v1 26d7b439505144353268 | 2026-11-17T15:26 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Measure the redundant HP5 line during profiled autocal sweeps |
| c2450975d7 | v1 8bf07189f234b1820d1b | 2026-11-17T11:00 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Match firmware anchor workspaces and select gravity-side forward solutions |
| 77351be962 | v1 ef4bc0e5253bed6847bb | 2026-11-17T10:12 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Correct HP5 per-pose stretch fitting and validate guarded forward transforms |
| f7b8845863 | v1 0daf68fab727dab5973f | 2026-11-17T09:19 | 1/1 | 0.96 | 1.23 | 3573 | 5255 | 9 | stable-estimate | ok (legacy) | Fix cable initialization, stream resets, layering edits and angle unwrap (#99) |
| 84645fdc2d | v1 ee722f128e61799a660a | 2026-11-16T18:08 | 1/1 | 0.89 | 1.42 | 3572 | 5519 | 10 | stable-estimate | ok (legacy) | Rigid body render edges: name the polyline, allow prim paths |
| 0ba1b72250 | v1 86d054ded7b0725b37fd | 2026-11-16T16:52 | 1/1 | 0.89 | 1.42 | 3572 | **5519 ▲** | 10 | stable-estimate | ok (legacy) | Fix firmware flex force solvers, harden HP5 autocal planning, sync dev repos |
| e5593f4487 | v1 a712bacfb38f9d76592c | 2026-11-16T15:39 | 1/1 | 0.92 | 2.27 | 3573 | 4552 | 7 | stable-estimate | ok (legacy) | Add square five-anchor Hangprinters with firmware print support |
| 876e22d2b0 | v1 4f48d3182c1f999361fa | 2026-11-16T12:25 | 1/1 | 0.92 | **2.27 ▲** | 3573 | 4551 | 7 | stable-estimate | ok (legacy) | Stop autocal when the estimate stops moving, keep sweeps clear of anchors, drop  |
| f0efe6cc21 | v1 d0b4db068a2f6e80ff5e | 2026-11-16T11:19 | 1/1 | 0.86 | 0.78 | 3573 | 6765 | 14 | patience | ok (legacy) | Allow one motor step when checking that the drive reached a spool-reserve target |
| baf71e7224 | v1 b62bd5cb2b9cb06c5ecd | 2026-11-16T10:17 | 1/1 | 0.86 | 0.78 | 3573 | 6765 | 14 | patience | ok (legacy) | Adds speed readout |
| 97971144ed | v1 977f1ca62c86dce51722 | 2026-11-15T21:50 | 1/1 | 0.86 | 0.78 | 3573 | 6765 | 14 | patience | ok (legacy) | Document measured workspace exploration and remove obsolete spread import |
| 5251f749b2 | v1 06fff392479e57bb1b9e | 2026-11-15T17:36 | 1/1 | 0.86 | 0.78 | 3573 | **6765 ▲** | 14 | patience | ok (legacy) | Verify commanded returns and abort collection on failed recovery |
| dfc0e3e3f4 | v1 0eb2dee794f335c46083 | 2026-11-15T17:23 | 0/1 | – | – | – | – | – |  | retry (legacy) | Learn safely rejected workspace probes and preserve feasible strain profiles |
| 8338be8f58 | v1 b79e8c72e15a54a4e3e3 | 2026-11-15T16:54 | 0/1 | – | – | – | – | – |  | failed (legacy) | Probe later sweep points locally and account for reeving in force profiles |
| ecf052eba1 | v1 d8f8bb09a0f7e959179d | 2026-11-15T16:26 | 0/1 | – | – | – | – | – |  | failed (legacy) | Scope reserve evidence to measured samples, intervals and preload |
| 6ab979c367 | v1 58bce49a378dd960ca32 | 2026-11-15T16:16 | 0/1 | – | – | – | – | – |  | failed (legacy) | Explore successive measured workspace and verify spool reserve in production |
| 60efc2915f | v1 9d4bc55ff5dbe47f97b8 | 2026-11-15T13:34 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Withdraw live terminal pilot and harden collection writes and interruption |
| a2f47d14fa | v1 9a3170ca6e268bfd2ebc | 2026-11-15T13:06 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Pilot a common fresh terminal sweep for frozen candidate validation |
| 7038fa0502 | v1 5b81c7c70fac9387d18e | 2026-11-15T12:48 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Preserve interrupted status and update spool-search examples and checks |
| cbc8058185 | v1 da038aa8f8c6c682cf93 | 2026-11-15T12:32 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Clarify that fit quality does not certify anchor accuracy |
| 129b50686c | v1 e75cadebc057c47d7aeb | 2026-11-15T12:32 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Refuse unfinished bootstrap restarts and prevent mixed-model collection |
| e3568c086d | v1 10bb188413c4264dc3a7 | 2026-11-15T12:16 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Add autocal sanity battery and baseline evidence |
| 3d41c608ab | v1 15e24f6eea1d46104f2e | 2026-11-15T12:14 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Preserve measurement-model consistency and log final compensation decisions |
| 65e074796f | v1 e43f9558145a4af09863 | 2026-11-15T12:10 | 1/1 | 1.68 | 2.60 | unavailable | 4457 | 8 | spread-consensus | ok (legacy) | Guard physical home correction with independent extension evidence |
| 2fbf80cbc7 | v1 77ee44701f0c07a21cdf | 2026-11-15T12:10 | 1/1 | **1.68 ▲** | 2.60 | unavailable | **4457 ▲** | 8 | spread-consensus | ok (legacy) | Check rough geometry before equal-strain collection and preserve offset semantic |
| ae476f3339 | v1 68f0d5d484197af5d762 | 2026-11-15T11:13 | 1/1 | 1.37 | **2.79 ▲** | unavailable | **3961 ▲** | 7 | spread-consensus | ok (legacy) | Measure Hangprinter sweeps at equal strain with a per-point sensor force |
| 0d700bade6 | v1 07e1f988fb78e4ea497d | 2026-11-15T11:00 | 1/1 | 8.44 | 2.12 | unavailable | 2952 | 7 | spread-consensus | ok (legacy) | Rank Hangprinter sweeps by position information so the planner shortens D |
| aae32d13a1 | v1 1c504b5ead3aff7fdfe3 | 2026-11-15T10:50 | 1/1 | 8.28 | 4.91 | unavailable | 3092 | 7 | spread-consensus | ok (legacy) | Run a point-based anchor fit beside the ellipse fit, with optional home length o |
| 1254ef00db | v1 c3c9ebdd3c95bc584281 | 2026-11-14T23:49 | 1/1 | 8.28 | 4.91 | unavailable | 3092 | 7 | spread-consensus | ok (legacy) | Default simulated autocal to 20 cable solver iterations per catalogued scene |
| 8a00ceef1b | v1 32392b8fe03c66c48510 | 2026-11-14T21:30 | 1/1 | 10.14 | 17.31 | unavailable | 3131 | 7 | spread-consensus | ok (legacy) | Stop collection when a held motor drifts more than a full turn |
| 0cfc3a593a | v1 290cbdd78988379881c1 | 2026-11-14T21:26 | 1/1 | 10.14 | 17.31 | unavailable | 3131 | 7 | spread-consensus | ok (legacy) | Rebase browser motor commands on each bridge connection |
| 36cd39de18 | v1 273c73673913a724a185 | 2026-11-14T20:38 | 1/1 | **10.14 ▲** | 17.31 | unavailable | 3131 | 7 | spread-consensus | ok (legacy) | Fix autocal timeout due to standstill bugfix in RRF |
| 3dd66bc678 | v1 bc515003b03e376bcf20 | 2026-11-13T19:23 | 0/1 | – | – | – | – | – |  | failed (legacy) | Add worn wheel bearings and an HP4 scene with them |
| aedacd999e | v1 3856327ecd9bd8c5dd46 | 2026-11-13T15:06 | 0/1 | – | – | – | – | – |  | failed (legacy) | Verify cable stick/slip states against the solved step |
| 83de968add | v1 378350437126d0a09f19 | 2026-11-13T12:46 | 0/1 | – | – | – | – | – |  | failed (legacy) | Balance moments in CubeCorners flex compensation |
| 89690122f8 | v1 5c6b5c15cf511452f5fd | 2026-11-13T12:12 | 0/1 | – | – | – | – | – |  | failed (legacy) | CubeCorners needs 5 solver iterations by default |
| 70d35ae6dd | v1 f4eb32769c7855b8f276 | 2026-11-12T21:53 | 0/1 | – | – | – | – | – |  | failed (legacy) | Speed up Finish ASAP without changing physics results |
| 0579861185 | v1 fe8f3983b1703e3727a7 | 2026-11-12T15:15 | 0/1 | – | – | – | – | – |  | failed (legacy) | Restyle hp-sim-3d with a graphite studio look and compact controls |
| 3bfd9fefc2 | v1 5e6edafae5a4b2a0aa29 | 2026-11-12T13:51 | 0/1 | – | – | – | – | – |  | failed (legacy) | Use one HP4 substep and expose cable solver iterations |
| 6ee13986bf | v1 5048f475a5ee91473b91 | 2026-11-12T13:30 | 0/1 | – | – | – | – | – |  | failed (legacy) | Select machine-specific physics substeps from the catalogue |
| 9f4351b9b6 | v1 9ab11c1885b09a3d9bcf | 2026-11-12T12:53 | 0/1 | – | – | – | – | – |  | failed (legacy) | Preserve Hangprinter position across flex-mode changes |
| c73cdfaded | v1 d6ac5546be653049e476 | 2026-11-12T12:46 | 0/1 | – | – | – | – | – |  | failed (legacy) | Adds machine catalogue and defers research world selection |
| 3fcf618b23 | v1 95ff685ab3fc9851f262 | 2026-11-12T12:33 | 0/1 | – | – | – | – | – |  | failed (legacy) | Fix firmware flex scaling and regenerate bundled motion streams |
| 8e2df52ba5 | v1 dee44c33502704f639fe | 2026-11-12T11:22 | 0/1 | – | – | – | – | – |  | failed (legacy) | Merge branch 'quality-monitor-missed-steps-and-perf' |
| b497fbab04 | v1 cab0e24ea2a05eb749cd | 2026-11-12T10:21 | 1/1 | 9.04 | 18.33 | unavailable | 3132 | 7 | spread-consensus | ok (legacy) | Merge pull request #94 from tobbelobb/remove-experimental-hp4-pinhole-gates |
| f5d4631ef2 | v1 b81ca57c5b0143446056 | 2026-11-12T09:18 | 1/1 | **9.04 ▲** | **18.33 ▲** | unavailable | 3132 | 7 | spread-consensus | ok (legacy) | Makes stepper motors stable at any holding torque; raises all scenes to 1 Nm |
| 50e826bf77 | v1 a708ff502aa140467606 | 2026-11-11T22:29 | 1/1 | 4.08 | 5.61 | unavailable | 3425 | 7 | spread-consensus | ok (legacy) | Uses lines-per-axis flex compensation and regenerates command files |
| 67d0e82c9f | v1 8d417a9cae575d756954 | 2026-11-11T21:06 | 1/1 | 4.08 | 5.61 | unavailable | 3425 | 7 | spread-consensus | ok (legacy) | Updates print Quality Control feature |
| a136534500 | v1 abad199193daf9cba6b6 | 2026-11-11T19:58 | 1/1 | 4.08 | 5.61 | unavailable | 3425 | 7 | spread-consensus | ok (legacy) | Fixes some bugs. Let's motors yield whe we grab and drag the effector. |
| a92b6c45f0 | v1 733b0cf35bacd690de65 | 2026-11-11T15:32 | 1/1 | 4.13 | **28.40 ▲** | unavailable | 3426 | 7 | spread-consensus | ok (legacy) | Updates CableJoints with help of upstream implementation from Nvidia |
| bd779e93d2 | v1 3fe35a4b2953376b0be8 | 2026-11-11T09:32 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Be more user friendly if user forgets to open gcode websocket in browser |
| 2739fb0404 | v1 ae7dd8f5a8d488696fb1 | 2026-11-11T08:38 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Merge pull request #92 from tobbelobb/t3/test-suite-weeding |
| 678f765911 | v1 f5ddc118844fbf4fdfcf | 2026-11-06T21:28 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Fix projected encoder data selection and preserve autocal research |
| 1eaf01c1b0 | v1 e802b1bc249f7f8f8e1c | 2026-11-06T11:11 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Updates node version. |
| 087cdd180a | v1 f0edb53a6d407466fd1e | 2026-11-05T22:16 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Merge branch 'research/bounded-research-assignment-ca60a579' into research/autoc |
| fbd2c74427 | v1 b25a2755197f8db7452b | 2026-11-05T17:59 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Make cloud research sequential and recording failures non-fatal |
| 49c907a55e | v1 203c41279914d4326b5b | 2026-11-04T18:25 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Merge pull request #86 from tobbelobb/codex/autocal-reliability-fixes |
| e8b520199b | v1 784bc82e2c13e9279ce6 | 2026-11-04T17:58 | 1/1 | 3.84 | 13.54 | unavailable | 4504 | 8 | spread-consensus | ok (legacy) | Audits research program usefulness |

## hangprinter_4

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 7a1b0e84c1 | v2 034990ae1e90591c1630 | 2026-11-18T21:19 | 1/1 | 5.75 | 0.94 | unavailable | 4693 | 8 | stable-estimate | ok | Merge pull request #117 from tobbelobb/t3/fix-worker-quality-history-timing |
| f373f1f010 | v2 2234bd438f698ff07c56 | 2026-11-18T21:17 | 1/1 | 5.75 | 0.94 | unavailable | 4693 | 8 | stable-estimate | ok | Merge pull request #114 from tobbelobb/t3/isolate-recorder-failures |
| f2be134944 | v2 52a29666a46f60479f53 | 2026-11-18T21:16 | 1/1 | 5.75 | 0.94 | unavailable | 4693 | 8 | stable-estimate | ok | Merge pull request #113 from tobbelobb/t3/fix-noop-payload-resume |
| 20da983feb | v2 fdc9c52ee787dbdde295 | 2026-11-18T18:17 | 1/1 | 5.75 | 0.94 | unavailable | 4693 | 8 | stable-estimate | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 db83d3f94b9573bc9a39 | 2026-11-18T16:53 | 1/1 | 4.42 | 1.31 | unavailable | 5731 | 12 | stable-estimate | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |
| 0623060e22 | v1 71c1dc453fa01d516923 | 2026-11-18T11:48 | 1/1 | 4.57 | 1.51 | unavailable | 5912 | 13 | stable-estimate | ok (legacy) | Track autocal benchmark per machine and warn on regressions |
| ce4afd7eb4 | v1 1bf51bd4dae4c5bed89b | 2026-11-18T11:39 | 1/1 | 4.57 | 1.51 | unavailable | 5912 | 13 | stable-estimate | ok (legacy) | Merge pull request #101 from tobbelobb/t3/rrf-host-standstill-bookkeeping |
| 97091d345c | v1 f67656c2426a8a4434d8 | 2026-11-18T11:39 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Merge pull request #100 from tobbelobb/t3/fix-ambient-layering-plan |
| 13433a9eb5 | v1 67b5f6061530f804dab0 | 2026-11-18T11:12 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Keep one browser collector session across autocal sweeps |
| 99c8820d66 | v1 5a63200ba995cd8a916a | 2026-11-18T10:28 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Honor equal-strain collection in fresh HP5 bootstraps |
| 0652a47b95 | v1 58540811c6ba33372899 | 2026-11-18T10:27 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Read the spare HP5 encoder during ordinary sweep motion |
| 6826087176 | v1 269fea16fd84110c3763 | 2026-11-17T16:57 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Use HP5 spare ranges throughout calibration and stop after agreeing good fits |
| 089a7ad371 | v1 f0f77dd2667aea1f53d7 | 2026-11-17T15:26 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Measure the redundant HP5 line during profiled autocal sweeps |
| c2450975d7 | v1 e7a109ead22c58da32f4 | 2026-11-17T11:00 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Match firmware anchor workspaces and select gravity-side forward solutions |
| 77351be962 | v1 4796d0aaba3e736c60b2 | 2026-11-17T10:12 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Correct HP5 per-pose stretch fitting and validate guarded forward transforms |
| f7b8845863 | v1 da6fa5c9a1daa0d1c667 | 2026-11-17T09:19 | 1/1 | 4.57 | 1.50 | unavailable | 5916 | 13 | stable-estimate | ok (legacy) | Fix cable initialization, stream resets, layering edits and angle unwrap (#99) |
| 84645fdc2d | v1 bb4e8c8bd609102c71e7 | 2026-11-16T18:08 | 1/1 | 4.56 | 1.45 | unavailable | 6337 | 14 | stable-estimate | ok (legacy) | Rigid body render edges: name the polyline, allow prim paths |
| 0ba1b72250 | v1 47f61f2ee6782d2f550c | 2026-11-16T16:52 | 1/1 | 4.56 | 1.45 | unavailable | **6337 ▲** | 14 | stable-estimate | ok (legacy) | Fix firmware flex force solvers, harden HP5 autocal planning, sync dev repos |
| e5593f4487 | v1 10c56a4ef4a44b265f20 | 2026-11-16T15:39 | 1/1 | 4.39 | **1.48 ▲** | unavailable | 5911 | 13 | stable-estimate | ok (legacy) | Add square five-anchor Hangprinters with firmware print support |
| 876e22d2b0 | v1 ad82bfc21463e7760a6f | 2026-11-16T12:25 | 1/1 | 4.60 | 1.26 | unavailable | 5909 | 13 | stable-estimate | ok (legacy) | Stop autocal when the estimate stops moving, keep sweeps clear of anchors, drop  |
| f0efe6cc21 | v1 edfa6c0115009083fc9b | 2026-11-16T11:19 | 1/1 | 5.36 | 1.06 | unavailable | 6339 | 14 | patience | ok (legacy) | Allow one motor step when checking that the drive reached a spool-reserve target |
| baf71e7224 | v1 a2699243b3295d68c659 | 2026-11-16T10:17 | 0/1 | – | – | – | – | – |  | retry (legacy) | Adds speed readout |
| 97971144ed | v1 b7f36dfe6060b47ffd7e | 2026-11-15T21:50 | 1/1 | 5.36 | 1.06 | unavailable | 6339 | 14 | patience | ok (legacy) | Document measured workspace exploration and remove obsolete spread import |
| 5251f749b2 | v1 a1c24ada8f0a5920f653 | 2026-11-15T17:36 | 1/1 | **5.36 ▲** | 1.06 | unavailable | 6339 | 14 | patience | ok (legacy) | Verify commanded returns and abort collection on failed recovery |
| dfc0e3e3f4 | v1 b688562d765da77c8160 | 2026-11-15T17:23 | 1/1 | 4.46 | **1.04 ▲** | unavailable | 6150 | 14 | patience | ok (legacy) | Learn safely rejected workspace probes and preserve feasible strain profiles |
| 8338be8f58 | v1 395f826abc7e723bb9fc | 2026-11-15T16:54 | 0/1 | – | – | – | – | – |  | failed (legacy) | Probe later sweep points locally and account for reeving in force profiles |
| ecf052eba1 | v1 80a6516c8b89bc6eee3f | 2026-11-15T16:26 | 0/1 | – | – | – | – | – |  | failed (legacy) | Scope reserve evidence to measured samples, intervals and preload |
| 6ab979c367 | v1 640f33d415fe82faa689 | 2026-11-15T16:16 | 1/1 | 4.64 | 0.33 | unavailable | **6633 ▲** | 15 | patience | ok (legacy) | Explore successive measured workspace and verify spool reserve in production |
| 60efc2915f | v1 593832f7c2de42f935ee | 2026-11-15T13:34 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Withdraw live terminal pilot and harden collection writes and interruption |
| a2f47d14fa | v1 fe97c7ea201533dfd611 | 2026-11-15T13:06 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Pilot a common fresh terminal sweep for frozen candidate validation |
| 7038fa0502 | v1 a53738d981ed86e1db80 | 2026-11-15T12:48 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Preserve interrupted status and update spool-search examples and checks |
| cbc8058185 | v1 fdf0fd79c909789d2eac | 2026-11-15T12:32 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Clarify that fit quality does not certify anchor accuracy |
| 129b50686c | v1 99b3d2465d8024273bd3 | 2026-11-15T12:32 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Refuse unfinished bootstrap restarts and prevent mixed-model collection |
| e3568c086d | v1 17a89fdf179d83e2f5c8 | 2026-11-15T12:16 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Add autocal sanity battery and baseline evidence |
| 3d41c608ab | v1 ed707a9251154778b662 | 2026-11-15T12:14 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Preserve measurement-model consistency and log final compensation decisions |
| 65e074796f | v1 0ab83c6920565c73d3a0 | 2026-11-15T12:10 | 1/1 | 4.82 | 1.63 | unavailable | 4215 | 8 | spread-consensus | ok (legacy) | Guard physical home correction with independent extension evidence |
| 2fbf80cbc7 | v1 20ce1c494c2dc4d0ce76 | 2026-11-15T12:10 | 1/1 | 4.82 | **1.63 ▲** | unavailable | **4215 ▲** | 8 | spread-consensus | ok (legacy) | Check rough geometry before equal-strain collection and preserve offset semantic |
| ae476f3339 | v1 92175225cb753e85bdc3 | 2026-11-15T11:13 | 1/1 | 4.83 | 1.23 | unavailable | **3805 ▲** | 7 | spread-consensus | ok (legacy) | Measure Hangprinter sweeps at equal strain with a per-point sensor force |
| 0d700bade6 | v1 c914b709554104be2e83 | 2026-11-15T11:00 | 1/1 | 5.13 | 1.22 | unavailable | 2847 | 7 | spread-consensus | ok (legacy) | Rank Hangprinter sweeps by position information so the planner shortens D |
| aae32d13a1 | v1 d2c202e4ce302c9d80ae | 2026-11-15T10:50 | 1/1 | 5.49 | 11.22 | unavailable | 3009 | 7 | spread-consensus | ok (legacy) | Run a point-based anchor fit beside the ellipse fit, with optional home length o |
| 1254ef00db | v1 638cc6ae19d2ad2e94b9 | 2026-11-14T23:49 | 1/1 | **5.49 ▲** | 11.22 | unavailable | 3009 | 7 | spread-consensus | ok (legacy) | Default simulated autocal to 20 cable solver iterations per catalogued scene |
| 8a00ceef1b | v1 9f0a3cb959114dda5632 | 2026-11-14T21:30 | 1/1 | 3.34 | 12.72 | unavailable | 3021 | 7 | spread-consensus | ok (legacy) | Stop collection when a held motor drifts more than a full turn |
| 0cfc3a593a | v1 8a3cd8289c8ef7aed4bc | 2026-11-14T21:26 | 1/1 | 3.34 | 12.72 | unavailable | 3021 | 7 | spread-consensus | ok (legacy) | Rebase browser motor commands on each bridge connection |
| 36cd39de18 | v1 1cb9cbea72906d616449 | 2026-11-14T20:38 | 1/1 | 3.34 | **12.72 ▲** | unavailable | 3021 | 7 | spread-consensus | ok (legacy) | Fix autocal timeout due to standstill bugfix in RRF |
| 3dd66bc678 | v1 79710ff4722ecf4a40bf | 2026-11-13T19:23 | 0/1 | – | – | – | – | – |  | failed (legacy) | Add worn wheel bearings and an HP4 scene with them |
| aedacd999e | v1 fbb74f6d265d86623abd | 2026-11-13T15:06 | 0/1 | – | – | – | – | – |  | failed (legacy) | Verify cable stick/slip states against the solved step |
| 83de968add | v1 771bb7c300a5c8e94c6f | 2026-11-13T12:46 | 0/1 | – | – | – | – | – |  | failed (legacy) | Balance moments in CubeCorners flex compensation |
| 89690122f8 | v1 8162ab8a3170105bbfa0 | 2026-11-13T12:12 | 0/1 | – | – | – | – | – |  | failed (legacy) | CubeCorners needs 5 solver iterations by default |
| 70d35ae6dd | v1 414b5a15341e9991d689 | 2026-11-12T21:53 | 0/1 | – | – | – | – | – |  | failed (legacy) | Speed up Finish ASAP without changing physics results |
| 0579861185 | v1 4d753142b4a9158aaf03 | 2026-11-12T15:15 | 0/1 | – | – | – | – | – |  | failed (legacy) | Restyle hp-sim-3d with a graphite studio look and compact controls |
| 3bfd9fefc2 | v1 33a831616bfebeee911e | 2026-11-12T13:51 | 0/1 | – | – | – | – | – |  | failed (legacy) | Use one HP4 substep and expose cable solver iterations |
| 6ee13986bf | v1 0b624e44e97b642bdd92 | 2026-11-12T13:30 | 0/1 | – | – | – | – | – |  | failed (legacy) | Select machine-specific physics substeps from the catalogue |
| 9f4351b9b6 | v1 1cebae29740cc2fdf007 | 2026-11-12T12:53 | 0/1 | – | – | – | – | – |  | failed (legacy) | Preserve Hangprinter position across flex-mode changes |
| c73cdfaded | v1 db99e47bd5fbd9a3a68c | 2026-11-12T12:46 | 0/1 | – | – | – | – | – |  | failed (legacy) | Adds machine catalogue and defers research world selection |
| 3fcf618b23 | v1 5619aed837a2e448c5d2 | 2026-11-12T12:33 | 0/1 | – | – | – | – | – |  | failed (legacy) | Fix firmware flex scaling and regenerate bundled motion streams |
| 8e2df52ba5 | v1 ed803b14adf65f09d262 | 2026-11-12T11:22 | 0/1 | – | – | – | – | – |  | failed (legacy) | Merge branch 'quality-monitor-missed-steps-and-perf' |
| b497fbab04 | v1 d05481a2d924e1db698b | 2026-11-12T10:21 | 1/1 | 6.15 | 8.32 | unavailable | 3047 | 7 | spread-consensus | ok (legacy) | Merge pull request #94 from tobbelobb/remove-experimental-hp4-pinhole-gates |
| f5d4631ef2 | v1 bed5ea665cb343980cfa | 2026-11-12T09:18 | 1/1 | **6.15 ▲** | **8.32 ▲** | unavailable | 3047 | 7 | spread-consensus | ok (legacy) | Makes stepper motors stable at any holding torque; raises all scenes to 1 Nm |
| 50e826bf77 | v1 bb68d4a72330b587b5b4 | 2026-11-11T22:29 | 1/1 | 4.57 | 2.36 | unavailable | 3511 | 7 | spread-consensus | ok (legacy) | Uses lines-per-axis flex compensation and regenerates command files |
| 67d0e82c9f | v1 9d832ab9cd6f90f7d159 | 2026-11-11T21:06 | 1/1 | 4.57 | 2.36 | unavailable | 3511 | 7 | spread-consensus | ok (legacy) | Updates print Quality Control feature |
| a136534500 | v1 81d762e6d7a0feca4066 | 2026-11-11T19:58 | 1/1 | **4.57 ▲** | 2.36 | unavailable | 3511 | 7 | spread-consensus | ok (legacy) | Fixes some bugs. Let's motors yield whe we grab and drag the effector. |
| a92b6c45f0 | v1 5efcb2d0780eae182b8e | 2026-11-11T15:32 | 1/1 | 2.98 | 6.34 | unavailable | 3424 | 7 | spread-consensus | ok (legacy) | Updates CableJoints with help of upstream implementation from Nvidia |
| bd779e93d2 | v1 c1a078467a752a669739 | 2026-11-11T09:32 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Be more user friendly if user forgets to open gcode websocket in browser |
| 8d1883b95b | v1 81d902a9216dc9abbd86 | 2026-11-10T19:19 | 0/1 | – | – | – | – | – |  | retry (legacy) | Merge pull request #91 from tobbelobb/t3/combine-88-89-90 |
| 678f765911 | v1 f9acf95e346f840b8acf | 2026-11-06T21:28 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Fix projected encoder data selection and preserve autocal research |
| 798b1aadd9 | v1 b707b474bde3b49421fb | 2026-11-06T11:16 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Use latest LTS Node version rather than hardcoded majors |
| 087cdd180a | v1 5ec6f87b4b3c255043ee | 2026-11-05T22:16 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Merge branch 'research/bounded-research-assignment-ca60a579' into research/autoc |
| fbd2c74427 | v1 64acd1ef754edf965765 | 2026-11-05T17:59 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Make cloud research sequential and recording failures non-fatal |
| b4817c7f0a | v1 d318f89f2b852992de44 | 2026-11-04T22:42 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Shortens research/program-workers.md, research/program.md, and research/prompts/ |
| e8b520199b | v1 43f9cd3315aee37fb192 | 2026-11-04T17:58 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Audits research program usefulness |

## hangprinter_5

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 7a1b0e84c1 | v2 c0f8c96686e0b9124e17 | 2026-11-18T21:19 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #117 from tobbelobb/t3/fix-worker-quality-history-timing |
| f373f1f010 | v2 41f48cc42e74eaa75cfa | 2026-11-18T21:17 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #114 from tobbelobb/t3/isolate-recorder-failures |
| f2be134944 | v2 6969effe4a19ee84a039 | 2026-11-18T21:16 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #113 from tobbelobb/t3/fix-noop-payload-resume |
| 20da983feb | v2 c9af9758cc9c363ae274 | 2026-11-18T18:17 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 ec76c752d8af462b4731 | 2026-11-18T16:53 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |
| 0623060e22 | v1 0b587a4e4d93452c8475 | 2026-11-18T11:48 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok (legacy) | Track autocal benchmark per machine and warn on regressions |
| ce4afd7eb4 | v1 b9de9a094b2533e0d106 | 2026-11-18T11:39 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok (legacy) | Merge pull request #101 from tobbelobb/t3/rrf-host-standstill-bookkeeping |
| 97091d345c | v1 c21f5a0573abd0c26274 | 2026-11-18T11:39 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok (legacy) | Merge pull request #100 from tobbelobb/t3/fix-ambient-layering-plan |
| 13433a9eb5 | v1 6a74e48c9553d96b8c4b | 2026-11-18T11:12 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok (legacy) | Keep one browser collector session across autocal sweeps |
| 99c8820d66 | v1 3babaac5eca17ab5d3e7 | 2026-11-18T10:28 | 1/1 | **1475.55 ▲** | **23.43 ▲** | **unavailable ▲** | **6564 ▲** | 4 | max-steps | ok (legacy) | Honor equal-strain collection in fresh HP5 bootstraps |
| 0652a47b95 | v1 a61526a74763ad13d597 | 2026-11-18T10:27 | 1/1 | **2.04 ▲** | 1.35 | 2493 | 2571 | 5 | good-fit | ok (legacy) | Read the spare HP5 encoder during ordinary sweep motion |
| 6826087176 | v1 346165a8bd745e2dae03 | 2026-11-17T16:57 | 1/1 | **1.36 ▲** | **1.55 ▲** | 3972 | 4705 | 6 | good-fit | ok (legacy) | Use HP5 spare ranges throughout calibration and stop after agreeing good fits |
| 089a7ad371 | v1 568c589cea94c4a644ba | 2026-11-17T15:26 | 1/1 | 1.07 | 1.20 | **5173 ▲** | **8664 ▲** | 12 | stable-estimate | ok (legacy) | Measure the redundant HP5 line during profiled autocal sweeps |
| c2450975d7 | v1 5ae9f53f72b11bd72663 | 2026-11-17T11:00 | 1/1 | 1.25 | 2.67 | 4272 | 5868 | 11 | stable-estimate | ok (legacy) | Match firmware anchor workspaces and select gravity-side forward solutions |
| 77351be962 | v1 f541b8ab64f4f78d1648 | 2026-11-17T10:12 | 1/1 | 1.25 | **2.67 ▲** | **4272 ▲** | 5868 | 11 | stable-estimate | ok (legacy) | Correct HP5 per-pose stretch fitting and validate guarded forward transforms |
| f7b8845863 | v1 8a52a2153433e4ed4176 | 2026-11-17T09:19 | 1/1 | 1.64 | 1.36 | 3958 | 5843 | 11 | stable-estimate | ok (legacy) | Fix cable initialization, stream resets, layering edits and angle unwrap (#99) |
| 84645fdc2d | v1 d1436c98d410fb7d9833 | 2026-11-16T18:08 | 1/1 | 1.64 | 1.37 | 3958 | 5842 | 11 | stable-estimate | ok (legacy) | Rigid body render edges: name the polyline, allow prim paths |
| 0ba1b72250 | v1 bb55877f91c26ad12219 | 2026-11-16T16:52 | 1/1 | **1.64 ▲** | 1.37 | 3958 | 5842 | 11 | stable-estimate | ok (legacy) | Fix firmware flex force solvers, harden HP5 autocal planning, sync dev repos |
| e5593f4487 | v1 187dc29dfdab965ddc57 | 2026-11-16T15:39 | 1/1 | 1.39 | 3.26 | 3958 | 6174 | 12 | stable-estimate | ok (legacy) | Add square five-anchor Hangprinters with firmware print support |
| 876e22d2b0 | v1 f336cb2dc713594eabe5 | 2026-11-16T12:25 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Stop autocal when the estimate stops moving, keep sweeps clear of anchors, drop  |
| f0efe6cc21 | v1 a31cfb0222366b688875 | 2026-11-16T11:19 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Allow one motor step when checking that the drive reached a spool-reserve target |
| baf71e7224 | v1 897bbf122c2271370c8a | 2026-11-16T10:17 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Adds speed readout |
| 97971144ed | v1 642fea00172b9b509e6e | 2026-11-15T21:50 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Document measured workspace exploration and remove obsolete spread import |
| 5251f749b2 | v1 bd80b6d22db7eafc064e | 2026-11-15T17:36 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Verify commanded returns and abort collection on failed recovery |
| dfc0e3e3f4 | v1 a9486f4559fcbd64911d | 2026-11-15T17:23 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Learn safely rejected workspace probes and preserve feasible strain profiles |
| 8338be8f58 | v1 5bab62c4877c149b1c1d | 2026-11-15T16:54 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Probe later sweep points locally and account for reeving in force profiles |
| ecf052eba1 | v1 3178120a22d7554bc42a | 2026-11-15T16:26 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Scope reserve evidence to measured samples, intervals and preload |
| 6ab979c367 | v1 3c7636d8cafdda0482bb | 2026-11-15T16:16 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Explore successive measured workspace and verify spool reserve in production |
| 60efc2915f | v1 fd2a420aa57c26bbc8be | 2026-11-15T13:34 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Withdraw live terminal pilot and harden collection writes and interruption |
| a2f47d14fa | v1 1dda27e9f3d8cd0560d5 | 2026-11-15T13:06 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Pilot a common fresh terminal sweep for frozen candidate validation |
| 7038fa0502 | v1 87359169b10e44135418 | 2026-11-15T12:48 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Preserve interrupted status and update spool-search examples and checks |
| cbc8058185 | v1 90494bdd7af0799d4d3a | 2026-11-15T12:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Clarify that fit quality does not certify anchor accuracy |
| 129b50686c | v1 5cdef44dbbb40abce1bc | 2026-11-15T12:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Refuse unfinished bootstrap restarts and prevent mixed-model collection |
| e3568c086d | v1 8832325438b1c3fab5fd | 2026-11-15T12:16 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Add autocal sanity battery and baseline evidence |
| 3d41c608ab | v1 d9d3804716787dafabf9 | 2026-11-15T12:14 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Preserve measurement-model consistency and log final compensation decisions |
| 65e074796f | v1 45d621265f2cec2b95e0 | 2026-11-15T12:10 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Guard physical home correction with independent extension evidence |
| 2fbf80cbc7 | v1 cb1fc5a144eb60646521 | 2026-11-15T12:10 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Check rough geometry before equal-strain collection and preserve offset semantic |
| ae476f3339 | v1 a8c353f617bd594d1095 | 2026-11-15T11:13 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Measure Hangprinter sweeps at equal strain with a per-point sensor force |
| 0d700bade6 | v1 744b5c3dfe099376d9b0 | 2026-11-15T11:00 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Rank Hangprinter sweeps by position information so the planner shortens D |
| aae32d13a1 | v1 27e6f399ead3b527adee | 2026-11-15T10:50 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Run a point-based anchor fit beside the ellipse fit, with optional home length o |
| 1254ef00db | v1 cbce24a48e98400309aa | 2026-11-14T23:49 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Default simulated autocal to 20 cable solver iterations per catalogued scene |
| 8a00ceef1b | v1 dc2ec542315e2f632a8d | 2026-11-14T21:30 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Stop collection when a held motor drifts more than a full turn |
| 0cfc3a593a | v1 b91f8cb0f165000e1f46 | 2026-11-14T21:26 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Rebase browser motor commands on each bridge connection |
| 36cd39de18 | v1 51fc0feabe7c9619c998 | 2026-11-14T20:38 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fix autocal timeout due to standstill bugfix in RRF |
| 3dd66bc678 | v1 89791f711ff1aaf59979 | 2026-11-13T19:23 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Add worn wheel bearings and an HP4 scene with them |
| aedacd999e | v1 a09368664bfc86792187 | 2026-11-13T15:06 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Verify cable stick/slip states against the solved step |
| 83de968add | v1 73336c1657363b67136a | 2026-11-13T12:46 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Balance moments in CubeCorners flex compensation |
| 89690122f8 | v1 364a847187410f7d066d | 2026-11-13T12:12 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | CubeCorners needs 5 solver iterations by default |
| 70d35ae6dd | v1 4a2c4b70c7e3c26190f5 | 2026-11-12T21:53 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Speed up Finish ASAP without changing physics results |
| 0579861185 | v1 83519b7dab37259265f7 | 2026-11-12T15:15 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Restyle hp-sim-3d with a graphite studio look and compact controls |
| 3bfd9fefc2 | v1 ecace39da6ab72fc0485 | 2026-11-12T13:51 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Use one HP4 substep and expose cable solver iterations |
| 6ee13986bf | v1 2966116659c05baa27c7 | 2026-11-12T13:30 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Select machine-specific physics substeps from the catalogue |
| 9f4351b9b6 | v1 8e00aad1680228445779 | 2026-11-12T12:53 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Preserve Hangprinter position across flex-mode changes |
| c73cdfaded | v1 79ea2d50d63cac112e86 | 2026-11-12T12:46 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Adds machine catalogue and defers research world selection |
| 3fcf618b23 | v1 2cccb80d6ab9a765efcc | 2026-11-12T12:33 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fix firmware flex scaling and regenerate bundled motion streams |
| 8e2df52ba5 | v1 4f8fd37df7467a62e3ef | 2026-11-12T11:22 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge branch 'quality-monitor-missed-steps-and-perf' |
| b497fbab04 | v1 1e4d4ce3fd6213a5eae3 | 2026-11-12T10:21 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge pull request #94 from tobbelobb/remove-experimental-hp4-pinhole-gates |
| f5d4631ef2 | v1 91c6ea12ad4fb72163b0 | 2026-11-12T09:18 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Makes stepper motors stable at any holding torque; raises all scenes to 1 Nm |
| 50e826bf77 | v1 fc9ebd094d5e1d289e94 | 2026-11-11T22:29 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Uses lines-per-axis flex compensation and regenerates command files |
| 67d0e82c9f | v1 802fa8847aba31d78761 | 2026-11-11T21:06 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Updates print Quality Control feature |
| a136534500 | v1 4ba26c4e513dbd53d9dc | 2026-11-11T19:58 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fixes some bugs. Let's motors yield whe we grab and drag the effector. |
| a92b6c45f0 | v1 cb3faebfb30a59fc1d21 | 2026-11-11T15:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Updates CableJoints with help of upstream implementation from Nvidia |
| bd779e93d2 | v1 d2f14db1908ae04c61fb | 2026-11-11T09:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Be more user friendly if user forgets to open gcode websocket in browser |
| 2739fb0404 | v1 c4fdd0b5549024026243 | 2026-11-11T08:38 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge pull request #92 from tobbelobb/t3/test-suite-weeding |
| 678f765911 | v1 c141c8dbdf1d5215c433 | 2026-11-06T21:28 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fix projected encoder data selection and preserve autocal research |
| 1eaf01c1b0 | v1 a89042b4bf8c70db7762 | 2026-11-06T11:11 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Updates node version. |
| 087cdd180a | v1 00d556867efb4588d2e9 | 2026-11-05T22:16 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge branch 'research/bounded-research-assignment-ca60a579' into research/autoc |
| fbd2c74427 | v1 18c0644634339dec84ce | 2026-11-05T17:59 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Make cloud research sequential and recording failures non-fatal |
| 49c907a55e | v1 00ca24c0333b6c80b87c | 2026-11-04T18:25 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge pull request #86 from tobbelobb/codex/autocal-reliability-fixes |
| e8b520199b | v1 891a5077c85219ba938b | 2026-11-04T17:58 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Audits research program usefulness |

## hangprinter_5_pulley_reduced

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 7a1b0e84c1 | v2 e461e15ee118b9f2a873 | 2026-11-18T21:19 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #117 from tobbelobb/t3/fix-worker-quality-history-timing |
| f373f1f010 | v2 27a3b76de8c462d1f830 | 2026-11-18T21:17 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #114 from tobbelobb/t3/isolate-recorder-failures |
| f2be134944 | v2 494a309671e53c1297b9 | 2026-11-18T21:16 | 1/1 | 2.57 | 0.63 | 4132 | 4201 | 6 | good-fit | ok | Merge pull request #113 from tobbelobb/t3/fix-noop-payload-resume |
| 20da983feb | v2 1a2005fcdd1766204c2c | 2026-11-18T18:17 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 cd2405c0977da2d289d1 | 2026-11-18T16:53 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |
| 0623060e22 | v1 575da6ac75202ecb3332 | 2026-11-18T11:48 | 1/1 | 2.57 | 0.63 | 4130 | 4200 | 6 | good-fit | ok (legacy) | Track autocal benchmark per machine and warn on regressions |
| ce4afd7eb4 | v1 6ce673e6678886210219 | 2026-11-18T11:39 | 1/1 | 2.57 | 0.63 | 4130 | 4200 | 6 | good-fit | ok (legacy) | Merge pull request #101 from tobbelobb/t3/rrf-host-standstill-bookkeeping |
| 97091d345c | v1 d93dfc90f859c80d5de6 | 2026-11-18T11:39 | 1/1 | 2.57 | 0.63 | 4130 | 4200 | 6 | good-fit | ok (legacy) | Merge pull request #100 from tobbelobb/t3/fix-ambient-layering-plan |
| 13433a9eb5 | v1 1cd7910f8a2ebc2d00f4 | 2026-11-18T11:12 | 1/1 | 2.57 | 0.63 | 4130 | 4200 | 6 | good-fit | ok (legacy) | Keep one browser collector session across autocal sweeps |
| 99c8820d66 | v1 96f1056652ec1c8a6c20 | 2026-11-18T10:28 | 1/1 | 2.57 | 0.62 | **4130 ▲** | **4200 ▲** | 6 | good-fit | ok (legacy) | Honor equal-strain collection in fresh HP5 bootstraps |
| 0652a47b95 | v1 0c2016d2cc94f54a3d76 | 2026-11-18T10:27 | 1/1 | **2.38 ▲** | 0.75 | 2787 | 2862 | 6 | good-fit | ok (legacy) | Read the spare HP5 encoder during ordinary sweep motion |
| 6826087176 | v1 5e86bcf81da7ed00cd3d | 2026-11-17T16:57 | 1/1 | 1.64 | 0.88 | 3381 | 4583 | 6 | good-fit | ok (legacy) | Use HP5 spare ranges throughout calibration and stop after agreeing good fits |
| 089a7ad371 | v1 a2de0250610fb2db49b9 | 2026-11-17T15:26 | 0/1 | – | – | – | – | – |  | step-cap (legacy) | Measure the redundant HP5 line during profiled autocal sweeps |
| c2450975d7 | v1 1bedf42d45b134624b30 | 2026-11-17T11:00 | 1/1 | 2.36 | 0.78 | unavailable | 4640 | 7 | stable-estimate | ok (legacy) | Match firmware anchor workspaces and select gravity-side forward solutions |
| 77351be962 | v1 d8c796bb411368602015 | 2026-11-17T10:12 | 1/1 | 2.36 | 0.78 | **unavailable ▲** | 4640 | 7 | stable-estimate | ok (legacy) | Correct HP5 per-pose stretch fitting and validate guarded forward transforms |
| f7b8845863 | v1 877cc3575f4299d83885 | 2026-11-17T09:19 | 1/1 | 2.33 | 1.99 | 4816 | 6879 | 15 | patience | ok (legacy) | Fix cable initialization, stream resets, layering edits and angle unwrap (#99) |
| 84645fdc2d | v1 2890422232ba787651ad | 2026-11-16T18:08 | 1/1 | 2.33 | 2.04 | 4820 | 6885 | 15 | patience | ok (legacy) | Rigid body render edges: name the polyline, allow prim paths |
| 0ba1b72250 | v1 bf3a7498916bcfcb5254 | 2026-11-16T16:52 | 1/1 | 2.33 | **2.04 ▲** | 4820 | 6885 | 15 | patience | ok (legacy) | Fix firmware flex force solvers, harden HP5 autocal planning, sync dev repos |
| e5593f4487 | v1 f085b094d29540e67971 | 2026-11-16T15:39 | 1/1 | 2.35 | 1.16 | 4819 | 6912 | 15 | patience | ok (legacy) | Add square five-anchor Hangprinters with firmware print support |
| 876e22d2b0 | v1 f336cb2dc713594eabe5 | 2026-11-16T12:25 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Stop autocal when the estimate stops moving, keep sweeps clear of anchors, drop  |
| f0efe6cc21 | v1 a31cfb0222366b688875 | 2026-11-16T11:19 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Allow one motor step when checking that the drive reached a spool-reserve target |
| baf71e7224 | v1 897bbf122c2271370c8a | 2026-11-16T10:17 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Adds speed readout |
| 97971144ed | v1 642fea00172b9b509e6e | 2026-11-15T21:50 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Document measured workspace exploration and remove obsolete spread import |
| 5251f749b2 | v1 bd80b6d22db7eafc064e | 2026-11-15T17:36 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Verify commanded returns and abort collection on failed recovery |
| dfc0e3e3f4 | v1 a9486f4559fcbd64911d | 2026-11-15T17:23 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Learn safely rejected workspace probes and preserve feasible strain profiles |
| 8338be8f58 | v1 5bab62c4877c149b1c1d | 2026-11-15T16:54 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Probe later sweep points locally and account for reeving in force profiles |
| ecf052eba1 | v1 3178120a22d7554bc42a | 2026-11-15T16:26 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Scope reserve evidence to measured samples, intervals and preload |
| 6ab979c367 | v1 3c7636d8cafdda0482bb | 2026-11-15T16:16 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Explore successive measured workspace and verify spool reserve in production |
| 60efc2915f | v1 fd2a420aa57c26bbc8be | 2026-11-15T13:34 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Withdraw live terminal pilot and harden collection writes and interruption |
| a2f47d14fa | v1 1dda27e9f3d8cd0560d5 | 2026-11-15T13:06 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Pilot a common fresh terminal sweep for frozen candidate validation |
| 7038fa0502 | v1 87359169b10e44135418 | 2026-11-15T12:48 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Preserve interrupted status and update spool-search examples and checks |
| cbc8058185 | v1 90494bdd7af0799d4d3a | 2026-11-15T12:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Clarify that fit quality does not certify anchor accuracy |
| 129b50686c | v1 5cdef44dbbb40abce1bc | 2026-11-15T12:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Refuse unfinished bootstrap restarts and prevent mixed-model collection |
| e3568c086d | v1 8832325438b1c3fab5fd | 2026-11-15T12:16 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Add autocal sanity battery and baseline evidence |
| 3d41c608ab | v1 d9d3804716787dafabf9 | 2026-11-15T12:14 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Preserve measurement-model consistency and log final compensation decisions |
| 65e074796f | v1 45d621265f2cec2b95e0 | 2026-11-15T12:10 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Guard physical home correction with independent extension evidence |
| 2fbf80cbc7 | v1 cb1fc5a144eb60646521 | 2026-11-15T12:10 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Check rough geometry before equal-strain collection and preserve offset semantic |
| ae476f3339 | v1 a8c353f617bd594d1095 | 2026-11-15T11:13 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Measure Hangprinter sweeps at equal strain with a per-point sensor force |
| 0d700bade6 | v1 744b5c3dfe099376d9b0 | 2026-11-15T11:00 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Rank Hangprinter sweeps by position information so the planner shortens D |
| aae32d13a1 | v1 27e6f399ead3b527adee | 2026-11-15T10:50 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Run a point-based anchor fit beside the ellipse fit, with optional home length o |
| 1254ef00db | v1 cbce24a48e98400309aa | 2026-11-14T23:49 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Default simulated autocal to 20 cable solver iterations per catalogued scene |
| 8a00ceef1b | v1 dc2ec542315e2f632a8d | 2026-11-14T21:30 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Stop collection when a held motor drifts more than a full turn |
| 0cfc3a593a | v1 b91f8cb0f165000e1f46 | 2026-11-14T21:26 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Rebase browser motor commands on each bridge connection |
| 36cd39de18 | v1 51fc0feabe7c9619c998 | 2026-11-14T20:38 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fix autocal timeout due to standstill bugfix in RRF |
| 3dd66bc678 | v1 89791f711ff1aaf59979 | 2026-11-13T19:23 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Add worn wheel bearings and an HP4 scene with them |
| aedacd999e | v1 a09368664bfc86792187 | 2026-11-13T15:06 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Verify cable stick/slip states against the solved step |
| 83de968add | v1 73336c1657363b67136a | 2026-11-13T12:46 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Balance moments in CubeCorners flex compensation |
| 89690122f8 | v1 364a847187410f7d066d | 2026-11-13T12:12 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | CubeCorners needs 5 solver iterations by default |
| 70d35ae6dd | v1 4a2c4b70c7e3c26190f5 | 2026-11-12T21:53 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Speed up Finish ASAP without changing physics results |
| 0579861185 | v1 83519b7dab37259265f7 | 2026-11-12T15:15 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Restyle hp-sim-3d with a graphite studio look and compact controls |
| 3bfd9fefc2 | v1 ecace39da6ab72fc0485 | 2026-11-12T13:51 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Use one HP4 substep and expose cable solver iterations |
| 6ee13986bf | v1 2966116659c05baa27c7 | 2026-11-12T13:30 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Select machine-specific physics substeps from the catalogue |
| 9f4351b9b6 | v1 8e00aad1680228445779 | 2026-11-12T12:53 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Preserve Hangprinter position across flex-mode changes |
| c73cdfaded | v1 79ea2d50d63cac112e86 | 2026-11-12T12:46 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Adds machine catalogue and defers research world selection |
| 3fcf618b23 | v1 2cccb80d6ab9a765efcc | 2026-11-12T12:33 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fix firmware flex scaling and regenerate bundled motion streams |
| 8e2df52ba5 | v1 4f8fd37df7467a62e3ef | 2026-11-12T11:22 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge branch 'quality-monitor-missed-steps-and-perf' |
| b497fbab04 | v1 1e4d4ce3fd6213a5eae3 | 2026-11-12T10:21 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge pull request #94 from tobbelobb/remove-experimental-hp4-pinhole-gates |
| f5d4631ef2 | v1 91c6ea12ad4fb72163b0 | 2026-11-12T09:18 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Makes stepper motors stable at any holding torque; raises all scenes to 1 Nm |
| 50e826bf77 | v1 fc9ebd094d5e1d289e94 | 2026-11-11T22:29 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Uses lines-per-axis flex compensation and regenerates command files |
| 67d0e82c9f | v1 802fa8847aba31d78761 | 2026-11-11T21:06 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Updates print Quality Control feature |
| a136534500 | v1 4ba26c4e513dbd53d9dc | 2026-11-11T19:58 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fixes some bugs. Let's motors yield whe we grab and drag the effector. |
| a92b6c45f0 | v1 cb3faebfb30a59fc1d21 | 2026-11-11T15:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Updates CableJoints with help of upstream implementation from Nvidia |
| bd779e93d2 | v1 d2f14db1908ae04c61fb | 2026-11-11T09:32 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Be more user friendly if user forgets to open gcode websocket in browser |
| 2739fb0404 | v1 c4fdd0b5549024026243 | 2026-11-11T08:38 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge pull request #92 from tobbelobb/t3/test-suite-weeding |
| 678f765911 | v1 c141c8dbdf1d5215c433 | 2026-11-06T21:28 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Fix projected encoder data selection and preserve autocal research |
| 1eaf01c1b0 | v1 a89042b4bf8c70db7762 | 2026-11-06T11:11 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Updates node version. |
| 087cdd180a | v1 00d556867efb4588d2e9 | 2026-11-05T22:16 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge branch 'research/bounded-research-assignment-ca60a579' into research/autoc |
| fbd2c74427 | v1 18c0644634339dec84ce | 2026-11-05T17:59 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Make cloud research sequential and recording failures non-fatal |
| 49c907a55e | v1 00ca24c0333b6c80b87c | 2026-11-04T18:25 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Merge pull request #86 from tobbelobb/codex/autocal-reliability-fixes |
| e8b520199b | v1 891a5077c85219ba938b | 2026-11-04T17:58 | 0/1 | – | – | – | – | – |  | unsupported (legacy) | Audits research program usefulness |
