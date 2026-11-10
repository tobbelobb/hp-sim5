# Autocal research recovery, 2026-11-05

Update: the original files were subsequently recovered and pushed to the
`research/autocal-originals-2026-11-05-0a82df7f` and
`research/autocal-recovery-2026-11-05-0a82df7f` branches. The text below
records the earlier conversation-only recovery phase, not the current data state.

This is a recovery archive published through the GitHub connector after command access to the research workspace failed. The contents were recovered from conversation tool output and the completed worker/reviewer responses. It is not the original sealed 32-file submission, and it does not contain the raw datasets or journals.

## Outcome

No production autocal improvement was delivered. A bounded preload change was tested in an isolated research harness and rejected as a robustness improvement. It preserved nominal accuracy but did not recover collection with active guide friction. Production source was unchanged.

| Completed calibration | A / B / C / D error, mm | Points | Simulated seconds |
| --- | --- | ---: | ---: |
| Nominal, default | 3.841 / 3.599 / 3.464 / 1.861 | 140 | 4183.208 |
| Nominal, revised preload | 3.873 / 3.719 / 3.366 / 1.980 | 140 | 4152.904 |
| Matched noise-only, default | 3.709 / 4.349 / 3.987 / 1.478 | 140 | 4278.630 |

All three stopped automatically with every anchor below 5 mm. Four active-guide trials stopped during collection and produced no applied geometry. Neither preload variant fixed the development condition.

A native probe found load torque alongside macroscopic spool-side cable slack. The guide model therefore needs validation before treating those collection failures as evidence about real machines. The probe was a constructed state, not exact replay or a full energy audit.

The separate reviewer returned `supported / keep / report_only`, after inspecting all 32 hashes and independently checking saved-data arithmetic. That response is preserved in [review-response.json](review-response.json). Recording the verdict into the live controller was interrupted. This archive does not invent a controller admission.

## What is preserved

- [Measured results reconstructed from recorded output](results.json), including failed trials and limits.
- The separate review response, program IDs, original source/artifact/submission hashes and recovery instructions.
- The refreshed parameter-variation prompt, including the human's robustness priority and per-anchor thresholds.
- Proposed next experiments, clearly marked unexecuted.

## What remains unrecovered

The original datasets, complete command/noise journals, fit/history artifacts, exact sealed source bundle and program database were on the workspace. Their current existence could not be checked after execution became unavailable. Unavailable access does not prove deletion.

The local startup commit `5bb8eb955fa680ddc631b6fb10f988ec3efb497a` was not found on GitHub. The recovery branch starts from the verified original source commit `865c9ab8aac926e258e9fe797141dd26e41e259e`.

## Durability gap

The workflow left completed research evidence local until end-of-iteration saving. That is insufficient protection against a runtime interruption. Small experiment checkpoints should be committed and pushed after each completed job, with datasets, result hashes, source snapshots and failures. Controller bookkeeping and final review should not be the only opportunity to preserve evidence.

## Next experiment

Test existing motor Coulomb/stiction variation with guide friction disabled, using per-motor changes and noisy encoder reads. Require concrete per-anchor results and a supported change before claiming an improvement. This was prepared but not launched. See [next-discriminator.md](next-discriminator.md).

Program: `program-0a82df7fd4b34b8181716630492a1af4`.
