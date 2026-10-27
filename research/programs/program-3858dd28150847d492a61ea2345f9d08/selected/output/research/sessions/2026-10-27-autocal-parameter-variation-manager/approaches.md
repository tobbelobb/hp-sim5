# Autocal approach ledger

Updated after the user's request to continue and prune weak routes. Existing reviewed work is published on origin/codex/autocal-parameter-variation-guards at703e1a68. The current bounded cycle has45 worker minutes,8 reserved review minutes and60 total manager minutes. It is not an unattended restart loop.

| Approach | State | Evidence and reason | Next decision or reopening condition |
| --- | --- | --- | --- |
| Unconditional TRF solver replacement | Retired within tested scope | Reviewed varied-world coverage gates fail; earlier legacy-fixture regressions remain. A lower mean on one scope is insufficient. | Only a materially narrower truth-free policy with paired nonworsening evidence. |
| Uniformly halve points on the same sweeps | Retired within tested scope | Reviewed per-world errors regress, including714.56mm on a TRF holdout. | Reopen for a different sampling policy, not the same thinning under a new name. |
| Rank by lower common-unseen encoder residual alone | Parked | Reviewed matrix has17 prediction/geometry reversals, including a complete-coverage counterexample. | A new measurement or adequacy rule that distinguishes these counterexamples without truth. |
| More independent fixed-length constraints at60 total points | Tested variant cut; broader route parked, review supported | Existing native3x20-point bootstrap estimates are poor. Previous thinning did not test redistributing points across circles. | Actual6x10 tests failed the paired20percent signal in all4 fits;2 worsened. Reopen only after force adequacy or a materially changed design. |
| Automatic force tuning / force adequacy | Promising, reviewed with limits | Learned-force replay gives nominal3.29mm after existing recovery and soft10-12mm, with same60points. Encoder-zero/state-history limits remain. | Keep existing tuning/recovery. Persist completed force selection before later collection can fail; investigate force-search cost in a future bounded comparison. |
| Broader initialization scales | Unfunded idea | Source inspection shows remaining starts are uniform over full bounds, not merely small perturbations around the small data-derived seed. | Require a narrower evidence-backed initialization hypothesis before spending fits. |
| Analytical varied-world screen | Kept, published |32 bounded fits in118.24s expose weaknesses. It is a diagnostic tool, not calibration accuracy improvement. | Use existing inputs to check promising candidates; do not redesign infrastructure this cycle. |
| Native collection helper and Node guard | Kept, published | Two complete60-point datasets under pinnedNode23; unsupportedNode24 now fails before movement. | Reuse for experiments. |
| Cloud review path fix | Kept, published |51 checks and two real separate reviews passed after repair. | Revisit only on a concrete runtime failure. |

The current collection hypothesis must use encoder-derived fixed offsets, never true anchor positions. Conventional scene-home geometry is offline evaluation only and has an encoder-zero pose/rotation limitation. Any claim about one-click stopping requires a complete automatic run, not a max-step budget stop.

Evidence: [first review](review-first.json), [native review](review-native.json), [native summary](native-saved-summary.json), [notebook](research.md). The current worker is attempt-902d9713dcbe436d8ddc1ba23fd82baa and assignment coverage-vs-force. A separate review will inspect its fixed evidence before adoption or pushing a new candidate.


Current-cycle refinements: a new solver fallback is not needed for the demonstrated nominal sentinel. Existing restore_sweeps then restore_points recovers about3.29mm in0.46seconds. Force-selection checkpointing is the retained robustness candidate. It snapshots tuned/manual/fallback selection before sweeps, keeps final force values updated after recovery, and is never automatically reused. Frozen cross-run predictions are useful but do not remove unknown encoder-zero pose/state-history differences.


Final review supports/keeps checkpoint25822cbb and optional39510991, code ready. Both are published on the same branch. No solver or acquisition default was changed. The next unfunded investigation is force-search cost with data-quality validation.
