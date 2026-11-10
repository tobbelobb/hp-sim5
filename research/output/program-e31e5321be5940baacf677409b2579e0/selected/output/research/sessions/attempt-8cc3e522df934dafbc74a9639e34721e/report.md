# CM-E1 saved-source audit: aggregate improvement fails per-dataset nonworsening

Status: complete. Assignment `cm-saved-audit`; inspection `CM-SAVED-A1`; historical experiment `CM-E1`. Review submission awaits a fresh reviewer.

## Abstract

Do four archived CM-E1 sources support unconditional finite-prediction-only final-consensus nonworsening? This audit inspected their bytes and JSON, copied 219,323 bytes of immutable source evidence, and checked hashes, cross-file identities, stored summary arithmetic and saved training/fold sets. Among eight dataset names beyond the motivating screen, the inspected producer evidence records one improvement, two regressions and five unchanged outcomes. Summed anchor error decreases from 401.100276 to 400.797031 mm, while flexible_lines_2000 and ten_points_w_buildup worsen by 0.502998 and 0.094156 mm. Thus the aggregate gain does not satisfy per-dataset nonworsening. For all three changed selections, the baseline trained on every available sweep; each variant leaves one terminal sweep unseen, but none is unseen by both models. Predictive superiority remains unresolved. All 91 structural consistency checks pass. These are inspected historical simulation measurements, with no independent numerical replay, fresh fits, physics, collection or hardware validation. Source identities and internal agreement do not validate the producer's numerical implementation or current-code equivalence.

## Question, method and allocation

Only [the archived report](sources/report.md), [results](sources/results.json), [audit](sources/audit.json) and [input manifest](sources/inputs.json) supplied scientific evidence. Their links to other materials were not followed. [extract.py](extract.py) uses only the Python standard library to copy bytes, hash files, parse JSON and check stored identities, sums and sweep-ID sets. It imports no autocal or simulation code. Run it from the repository with `.venv/bin/python /absolute/path/to/this/session/extract.py`; [evidence.json](evidence.json) records the exact pointers and checks. The compact JSON is deliberate to keep the review bundle small. [submission.json](submission.json) lists the portable review files; launcher logs and private attachment files are excluded.

The allocation and actual numerical usage are both zero for collections, fresh fits, physics runs/steps, selection evaluations and prediction evaluations. Recording is none. No probe.py, audit.py, preflight, doctor or supervisor/world command was invoked. No source, dependency, shared runtime, commit or manager graph change was made. CM-E1's saved 22 selections and 49 predictions belong to its producer, not this inspection.

## Per-dataset condition versus aggregate result

The archived method changes only final-consensus eligibility: retain candidates with finite stored own-model prediction, then apply consensus. The report says ranking, chosen single fit/spool, 0.01 mm merge distance, 5 mm spread limit and single-fit fallback were fixed. That method description is producer testimony, not source-code verification by this audit.

The relevant condition is that the variant's sum of per-anchor Euclidean distances to fixture simulation truth, in mm, does not exceed baseline for **each** dataset. This is neither coordinate RMSE nor an aggregate-only target. The following are inspected producer values from `results.json#/rows/{i}`. Positive delta means worse. They are rounded here; exact saved values and arm subobjects appear in evidence.json.

| Additional dataset | Row i | Baseline members | Finite-only members | Baseline mm | Variant mm | Saved delta mm |
| --- | ---: | --- | --- | ---: | ---: | ---: |
| 23_april_2026 | 0 | 4,5 | 4,5 | 99.579733 | 99.579733 | 0 |
| fifth_hp3_dataset | 1 | 2,3 | 2,3 | 65.908288 | 65.908288 | 0 |
| nineteenth_hp3_dataset_even_more_pressure | 2 | single 2 | single 2 | 165.005839 | 165.005839 | 0 |
| sixteenth_hp3_dataset_even_more_pressure | 4 | 4,5 | 4,5 | 37.934862 | 37.934862 | 0 |
| flexible_lines_2000 | 6 | 3,4 | 2,3 | 23.533389 | 24.036387 | +0.502998 |
| ten_points_bigger_deltas | 8 | 1,4 | 1,4 | 2.513340 | 2.513340 | 0 |
| ten_points_w_buildup | 9 | 3,5 | 3,4 | 2.380698 | 2.474855 | +0.094156 |
| ten_points_w_buildup2 | 10 | 4,7 | 3,6 | 4.244127 | 3.343729 | -0.900399 |

`results.json#/summary/additional_dataset` stores 8 datasets, 1 improved, 2 worsened, 5 unchanged; baseline sum 401.10027562823973 mm, variant sum 400.7970309548157 mm and delta -0.303244673424079 mm. Our arithmetic checks confirm that the stored rows agree with this summary; they do not recalculate anchor errors. The two positive saved deltas reject blanket per-dataset nonworsening within this archive even though the total decreases.

Rows 3, 5 and 7 are the three motivating-name overlaps, not eight additional confirmations. `results.json#/summary/motivating_dataset_overlap` stores three unchanged outcomes with both sums 214.76568608926382 mm. The report's all-eleven totals, 615.865962 and 615.562717 mm, agree with the rounded cohort arithmetic. Its earlier-screen gains are claims in this permitted report only: the initial screen's own sources were not inspected and are not independent evidence here. All three changed cases are manifest-labeled slideprinter; the saved signs are mixed even within that label.

## Exact common-unseen-fold gap

The applied model comprises the arm's anchors plus the chosen single fit's spool parameters. A fair common fold must exclude training sweeps of the chosen fit and every member in either arm. We checked the stored future-fold IDs against the union of these saved training-ID sets in `audit.json#/rows/{i}/candidates`. These are set operations on saved IDs, not prediction evaluations.

For the changed cases, `results.json#/rows/{i}/common_unseen_fold_ids` is `[]`. Both arm objects store `common_unseen_fold = {"status":"no_unseen_sweeps","score":null,"sweeps":0}`. This is absent evidence, not a failed or infinite prediction.

| Row / dataset | Chosen fit | Chosen-fit future fold (saved IDs) | Baseline training overlap | Variant training overlap | Variant-only unseen terminal ID |
| --- | ---: | --- | --- | --- | --- |
| 6 / flexible_lines_2000 | 2 | sweep_005, sweep_006 | sweep_005, sweep_006 | sweep_005 | sweep_006 |
| 9 / ten_points_w_buildup | 2 | sweep_005, sweep_006, sweep_007 | sweep_005, sweep_006, sweep_007 | sweep_005, sweep_006 | sweep_007 |
| 10 / ten_points_w_buildup2 | 1 | sweep_004, sweep_005, sweep_006, sweep_007, sweep_008, sweep_009 | all six listed | sweep_004 through sweep_008 | sweep_009 |

The newest baseline member is iteration 4, 5 or 7 respectively. Its saved training list covers sweep_001 through sweep_006, sweep_007 or sweep_009: all 6, 7 or 9 sweeps recorded in that audit row. Thus even the variant's one remaining unseen terminal sweep was used by the baseline. The chosen spool fits trained on sweep_001 through sweep_004, sweep_004 and sweep_003 respectively. Full individual candidate training lists are retained in evidence.json's `changed_rows[*].saved_training_coverage`, copied from the cited audit rows.

The saved chosen-fit-future diagnostics are finite:

| Dataset | Baseline saved diagnostic | Variant saved diagnostic |
| --- | ---: | ---: |
| flexible_lines_2000 | 21.72321317762027 | 21.977461365662236 |
| ten_points_w_buildup | 22.188338680710814 | 22.534494176891833 |
| ten_points_w_buildup2 | 21.495200525122375 | 21.845175051544803 |

Their exact paths are `/rows/{i}/{all_candidates,finite_prediction_only}/chosen_fit_future_fold/score` in results.json; overlap paths replace the last component with `future_fold_training_overlap_ids` at the arm level. Every changed diagnostic contains training overlap for both arms. The entire-fold scores therefore establish neither predictive harm nor benefit on common unseen data. In particular, the geometry improvement at row 10 coexists with a higher overlapping diagnostic score.

The eight unchanged rows have nonempty saved common folds, respectively: row 0 `[sweep_008]`; row 1 `[sweep_006,sweep_007,sweep_008]`; row 2 `[sweep_005,sweep_006,sweep_007]`; row 3 `[sweep_007]`; row 4 `[sweep_008]`; row 5 `[sweep_008]`; row 7 `[sweep_009]`; row 8 `[sweep_007,sweep_008]`. Both applied anchors and spool payloads are identical between arms on these rows. Their saved finite predictions cannot discriminate changed policies.

## Provenance and consistency limits

All four files were present and copied byte for byte. SHA-256 identities of the inspected source bytes are:

| Source | Bytes | SHA-256 |
| --- | ---: | --- |
| report.md | 10751 | `2e47bc589aea727b2e76387ff8fc3339b664134145f15d61df7fbbc7f46ef96a` |
| results.json | 178725 | `bdd85e0f758ce3a4f0ef4f5be3c0fe094fb830dfa6f3c059d39d7a2d804ab1cd` |
| audit.json | 20046 | `6750c2ac25c1f99a90cc10dd3d98389422e95a726e725bfdd1b1e6c8aac9939e` |
| inputs.json | 9801 | `f58ef95d783a265b7153cefc07d2d8785f77896846bb07d58317b8a009634b44` |

The actual manifest hash matches `results.json#/inputs_manifest_sha256`. All eleven fixture orders, cohorts, input paths/hashes and producer revision references align across the JSON files. Eleven distinct **recorded** sweep-content hashes agree with `audit.json#/unique_observation_datasets`; original sweep bytes were excluded by scope. Candidate iteration lists and chosen-fit errors agree with saved audit entries. All stored row deltas and cohort sums/counts are consistent. The recorded source-hash maps agree with their recorded changed-file lists. No substantive numerical/report discrepancy was found.

The fit producer revision in all manifest entries is `72523b9dc64155bf58fc1b8f0ea7241266ff73d4`. `results.json#/source_revision` records CM-E1 evaluation revision `5400cefc0238007406f5cd572d21ef5a72d2958d`; its `current_source_sha256` means current **then**, not the assignment revision `a1b4e559887550da6cac6a4c67819a618a766f65`. Each saved row lists 20 changed source files. We checked those saved hash-map relationships, not source-code contents.

The saved `single_fit_prediction_matches_saved` flags are true for all eleven; failure lists are empty. The producer report claims matching eleven single-fit predictions within 1e-8 absolute/1e-10 relative tolerance, finite 49 evaluations and disabled flexibility (multiplier 1.0). These remain producer evidence. Likewise, `audit.json#/archived_inputs_and_script_match_results = true` is an archived assertion: this review did not read raw/gzip envelopes or producer scripts and cannot reverify their byte identity, typed decoding, candidate immutability or numerical correctness. Hash agreement is integrity evidence, not accuracy evidence.

Manifest caveat: the five Hangprinter entries all say `machine: hangprinter_4`, including the hp3-named fixtures. We preserve those labels and do not infer historical live-world configuration from names. The report describes shared nominal geometry within fixture families; distinct dataset hashes do not establish eleven independent physical worlds. Three dataset names overlap the motivating screen. The four-source package supports archived final-selection comparisons, not fresh fitting, changed stopping trajectories, hardware accuracy, current-code equivalence or broad solver/model inadequacy. A truth-based error metric can audit this rule but does not supply a deployable truth-free discriminator.

## Hypotheses and decision

| Hypothesis actually inspected | Evidence ID | Outcome | Evidence |
| --- | --- | --- | --- |
| Blanket finite-only final consensus never worsens each additional CM-E1 dataset | CM-SAVED-A1 / historical CM-E1 | Rejected within archived scope | Rows 6 and 9; [extract](evidence.json) |
| The additional cohort's aggregate error decreases | CM-SAVED-A1 / historical CM-E1 | Supported as inspected producer evidence | `/summary/additional_dataset`; [results](sources/results.json) |
| Changed comparisons demonstrate common-unseen predictive superiority | CM-SAVED-A1 / historical CM-E1 | Inconclusive | Empty common folds at rows 6, 9, 10; [audit](sources/audit.json) |
| Archived rows support withholding unconditional adoption while fair holdout evidence is unresolved | CM-SAVED-A1 | Supported within archived scope | Regressions and training/fold overlap above |
| A narrower truth-free rule improves fresh fits, stopping or hardware | None | Not tested | Outside this allocation |

Retire only the unconditional nonworsening default-change proposal within CM-E1's eleven archived simulated histories, including the eight additional dataset names, for the sum-of-per-anchor-distance metric. The saved aggregate improvement does not rescue that condition. Keep fair common-unseen evaluation as the next prerequisite for a narrower membership proposal; no production or graph decision was applied by this worker.

The next useful saved-evidence observation is whether a compatible later whole sweep exists for each changed case, especially rows 6 and 9, whose exact ID/content and collection/settings provenance place it outside training of **all** members and the chosen spool fit in both arms. Inspect that provenance first under a separate allocation. This audit did not search beyond the four files. If no compatible saved sweep exists, a separately budgeted prefix/terminal-holdout study would be required before assessing common-unseen predictive superiority. Neither follow-up was tested here.

## References

- [CM-E1 archived report, 2026-10-23](sources/report.md): producer method, rounded tables, compatibility claims and scope; preceding session `2026-10-23-consensus-membership`. Its outgoing links are historical references, not evidence inspected in this audit.
- [CM-E1 results](sources/results.json): exact stored errors, membership, diagnostic/fold IDs, evaluation revision and producer budget. JSON pointers above and in evidence.json identify each claim.
- [CM-E1 audit](sources/audit.json): recorded candidate training coverage, distinct observation hashes and producer integrity assertion.
- [CM-E1 input manifest](sources/inputs.json): recorded input identities, cohort, nominal simulation truth, machine labels and fit producer revision.
- [Inspection extract](evidence.json) and [standard-library extractor](extract.py): CM-SAVED-A1 hashes, changed rows and 91 consistency checks; no independent numerical result.
- [Research notebook](research.md) and [searchable abstract](abstract.md): assignment, constraints, inspection history and handoff.
