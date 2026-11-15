# Common terminal validation: mixed accuracy and withdrawn live pilot

## Question and result

Can one additional sweep, withheld from every fit and scored against all frozen candidates, reduce final-selection regret? The hardened soft-HP3 run had selected a 4.60 mm candidate even though its last printed fit was 2.44 mm from the simulator reference. That latest fit had no future sweep for the ordinary selector.

Three complete production-path simulations on frozen revision `a2f47d14` tested the bounded pilot. They used simulated firmware, collector and physics. Each fitted eight sweeps / 160 observations, then collected one separate 20-point sweep without refitting any candidate. All stopped normally; all failed the battery's 1 mm accuracy threshold.

| Scene | Original selection error, mm | Terminal selection error, mm | Terminal translation-aligned error, mm |
| --- | ---: | ---: | ---: |
| HP3 nominal | 1.68 | 1.91 | 1.79 |
| HP3, stiffness 10000 | 4.60 | 4.60 | 2.16 |
| HP4 nominal | 4.82 | 4.33 | 2.61 |

The original-selection column is a frozen counterfactual from the **same training data and candidate history**, not another independent simulation. The terminal selector reduced fresh-sweep prediction cost on nominal HP3 from 1.476729 to 1.351448 while worsening coordinate accuracy. HP4 improved from 0.565240 to 0.525322 prediction cost alongside the modest coordinate improvement. Soft HP3 chose the same candidate and retained cost 2.131498. [Results](evidence/results.json) record the selected iterations, run IDs and point counts.

One proposed remedy for selection regret therefore did not consistently improve accuracy. The strongest alternative explanation is an uninformative terminal sweep: only one planned sweep was sampled per scene, so these results do not rule out a more diverse held-out collection design. Model bias can also make prediction cost and physical accuracy disagree. These are deterministic single trials, not a variance estimate or a robustness bound.

## Code readiness and withdrawal

The [independent review](../session-2026-11-15-independently-review-the-default-off-terminal-validation-pilot-f-8b45b54e/report.md) found live-path defects involving output aliases, collector preparation changing spool settings, stop/interrupt handling, force and projection compatibility, and post-scoring home transformations. The synthetic scorer checks did not establish live collection safety or physical accuracy.

The live option and its integration were removed in `60efc291`. Current production CLI does not expose `--terminal-validation`. Frozen offline scoring and `autocal.tools.replay_stage terminal` remain for inspecting these recordings; their identifier-overlap check cannot independently establish freshness. A caller must supply compatible machine, force, projection and length models. This experiment did not enable home compensation or spool fitting, override output paths or force settings, or request a stop mid-run; the reviewed branches do not invalidate its recorded negative accuracy comparison.

The review also motivated small shared production fixes: planner-owned output/configuration/fixed-target fields cannot be overridden by forwarded aliases; repeat collections omit the bootstrap reset flag; Python dataset replacement is atomic; a stop arriving during the last fit remains a user stop; and interruption during application returns 130 without resending an old candidate. Ordinary RRF spool fitting still prints fitted M666 for manual application, as documented. Removing the pilot avoids promising a validated-model application path across those differing firmware contracts.

## Reproduction and evidence

Each `evidence/{hp3,hp3-soft,hp4}/input.json` contains the exact autocal command and scene override. Each directory retains input scene, training and terminal datasets, profile files, stage artifacts, logs, summaries and the headless manifest. The manifests identify `a2f47d14` and source hashes. Reproduce live collection in a separate checkout of that revision; the historical flag is intentionally absent from current code. The original wrapper commands were:

```bash
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT --machine hangprinter_3 --autocal-args --terminal-validation
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT --machine hangprinter_3 --stiffness 10000 --autocal-args --terminal-validation
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT --machine hangprinter_4 --autocal-args --terminal-validation
```

Use a fresh output directory for each command. Wrapper exit status 1 indicates failed accuracy, while autocal itself stopped normally with status 0. On current code, inspect each saved `sweeps.terminal_validation.stage.json` without collection:

```bash
.venv/bin/python -m autocal.tools.replay_stage terminal SAVED_STAGE --output NEW_REPLAY
```

Run `PYTHONPATH=. .venv/bin/python PATH_TO_THIS_DIRECTORY/summarize.py` from the repository root to regenerate results and journal inventory. The summary reconstructs the original history selection and scores it against the same terminal data. Anchor accuracy uses the catalogued scenes' initial nominal firmware geometry, not a measured hardware reference. Rounded M669 coordinates set the reporting resolution.

Large local event journals are excluded from Git; [inventory](evidence/journals.json) records paths, sizes and SHA-256 hashes. They have no off-host backup. All owned services stopped according to their manifests. No hardware or full simulated encoder-noise trials were performed. New evidence that could change the conclusion would include matched-budget repeats with genuinely informative independent terminal poses and a reviewed collection/application protocol; those are future work, not part of the delivered implementation.
