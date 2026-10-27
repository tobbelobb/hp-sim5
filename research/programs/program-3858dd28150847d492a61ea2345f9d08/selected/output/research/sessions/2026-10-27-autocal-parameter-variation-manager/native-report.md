# Native HP3 stiffness pair and bootstrap fitting

Status: manager closure of a cancelled worker attempt; separate review supported/keep, code ready.

## Abstract

Two production native-Rust HP3 bootstrap collections, with 20000 and 8000 N cable stiffness, each completed three sweeps and 60 points. Together they took 75.02 measured world seconds and 602454 physics steps. Four ordinary fit-stage calls and four geometry-metadata-stripped checks survived a worker usage-limit interruption. All stripped checks produced numerically identical anchors, metrics and optimizer histories. Conventional scene-home anchor RMSE ranges from 2003 to 3648 mm, despite finite score and covariance checks. These are bootstrap fit stages, not complete automatic calibration. Encoder-zero pose and rotation were unavailable, so the scene-home metric has a reference limitation. A retained collection helper now checks pinned Node before movement and preserves partial failures. Two earlier zero-step crashes resulted from an unsourced Node24 shell and were corrected with the documented Node23 environment. No new physics or fitting was performed during manager closure.

## Authorship and interruption

The worker completed collection and eight fit-stage calls before its model usage limit stopped its conversation. Its launcher reached the original deadline at 09:01 UTC and exited130 with launcher_finally_completed cleanup. No worker final response or handoff was produced. This report and summary were written by the manager from preserved artifacts after the user's continue message. They are not a fabricated worker final output. No new simulation or optimization was run after resuming.

## Experiment and observations

Each scene has fixed30mm spool radii, zero line half-width and buildup, and layering disabled. The two scene copies differ only in nine stiffness values. The production collector uses the same three representative bootstrap configs, ten points per physical direction, feed1400, speedup25, manual low/mid/max forces0.01/0.05/0.5N and1N sensor preload. Noise settings request16samples at40Hz. These are matched settings, not identical realized trajectories. The drive span is94.106mm nominal and about234.918mm soft. An earlier1.269mm value was the fixed-anchor size target, not drive travel.

Nominal collection took26.324 world seconds,192909steps and385.818 simulated seconds. Soft took48.699 world seconds,409545steps and819.09 simulated seconds. Both saved3x20 finite points, the same native artifact66a8451b... and complete manifests with kept_alive=false. These single-run timings exclude separate fits and do not measure full automatic calibration or a speedup over reference003.

The first four ordinary fit stages used raw records. The final four removed only config.m669. The input audit shows identical data-derived seeds, and the saved anchors, metrics and full optimizer histories are exactly identical. Known geometry was used only for this report's offline conventional RMSE calculation.

| World | Solver | Primary cost | Conventional anchor RMSE, mm | Geometry helper valid |
| --- | --- | ---: | ---: | --- |
| nominal | lbfgsb | 12.6980 | 2023.36 | True |
| nominal | trf | 12.6989 | 2003.45 | True |
| soft | lbfgsb | 17.7371 | 3648.26 | False |
| soft | trf | 4.6793 | 2676.18 | False |

The saved driver's valid flag is a finite primary/rank score and covariance test. It is not a complete automatic-acceptance decision. All four underconstrained_penalty values are false, so this bounded driver made no restoration/recovery fit calls. The nominal fits are information-rank deficient; both soft fits fail the separate recovery_geometry_valid helper and hit geometry bounds. This exposes poor bootstrap estimation but cannot establish that the full loop would accept these anchors. Planning, later collection, history consensus and automatic stopping were not executed.

Conventional truth comes from the repository's scene-home effective anchors. The standalone status does not expose the settled encoder-zero effector pose or rotation. This limits precise physical-accuracy attribution. Manual force selection, adaptive size tuning, changed trajectories and two conditions also limit any claim that stiffness alone explains all fitted error.

## Retained code and checks

Candidate a4da0d73 adds collect_native_parameter_pair.py, tests and documentation. It reuses the production headless collector and keeps calibration/physics defaults unchanged. It refuses existing output and unsupported Node before movement, records actual inputs and identity, checks finite3x20 point completion, preserves failed worlds and returns failure for incomplete results. Its one-million-step limit applies per world. The540-second collector timeout excludes service startup/cleanup. The collector process is included in owned cleanup, and temporary step-limit environment settings are restored.

The measured driver is preserved separately. Delivery changes add portability, early runtime validation, recorded Node identity, stricter completion checks, environment restoration and failure handling. No new collection ran on the delivery commit. Nineteen focused mocked/data-contract tests passed in1.41seconds. These are not a new native reproduction or physics stability run. No engine, motor or production-scene implementation changed.

## Cloud faults

The two initial standalone launches inherited system Node24.19.0 and aborted in napi_detach_arraybuffer before completing a physics step. Supervised preflight had used pinned Node23, which did not transfer into the command shell. Sourcing scripts/cloud/env.sh and asserting the driver's Node23.11.1 identity enabled both replacements. Original failures and the explicit bounded replacement decision remain intact. This correction used no new backend, engine rebuild, model authentication or force retuning.

The later model usage limit prevented the worker's final narrative. The owner deadline still stopped its supervisor. The parent acknowledged cancellation only after observing actual actor failure and confirmed launcher cleanup. These execution failures are separate from the native data findings.

## Next decision

Keep the native datasets, measurement scripts and bounded helper. Do not change solver, point-count or stopping defaults on this pair. The next scientific question is whether automatic force tuning and additional independent sweeps repair the poor bootstrap models. It is not tested here. The original numerical budget is closed.

## Evidence

The fixed submission selects the two raw and stripped datasets, all four clean fit snapshots, raw and stripped outcomes/contracts, measured scripts, copied and baked scenes, checks, and a manager provenance bundle with exact original JSON text and hashes. [Saved summary](native-saved-summary.json) and [analysis script](analyze_native_saved.py) contain the manager arithmetic. Original worker artifacts remain at ../attempt-982dc44e4c9b4e5da9b3232c26d54bf3/.


Separate review completed as review-5f62ee3093f94f7a9af98157ecb24066, applied by admission-7d84479eb45c4688a07fd8c62f9f3cf0. The fixed reviewed bytes remain unchanged in the submission store.
