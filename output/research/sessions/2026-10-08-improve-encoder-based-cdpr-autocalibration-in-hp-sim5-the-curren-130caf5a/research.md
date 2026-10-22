# Encoder-only CDPR calibration: exact reuse, ranking and informative points

Objective: reproduce slow/fragile fitting and score/physical-accuracy disagreements; implement the best supported improvement, with particular interest in faster/fewer measured points. Preserve one-click operation and automatic stopping.

Constraints: .venv/bin/python; small source changes; ground truth evaluation only; no hardware claim. Fresh native world, not restored physics. No outer prompt loop. No sub-agents authorized.

Accepted steering: literature and first-principles ideas, prior-report search, then current-code learning; measure baseline before implementation; keep failed hypotheses.

Initial ideas from first principles/literature: redundant distance closure with latent-pose elimination; circle/ellipse invariants; joint anchor/spool/offset estimation with explicit scale/gauge; geometry-aware point placement instead of uniform thinning; whole-sweep validation on comparable movements; hysteresis-aware bidirectional checks; coarse-to-fine fitting and warm starts; reuse encoder settling/noise windows; continuous acquisition with a dynamics model. Arbitrary rigid transformations and encoder/spool scale cannot all be identified without references.

Prior evidence: simple halving worsened 3/4 fixtures; several score/optimizer changes worsened some results; guarded settling and persistent JAX cache already implemented. Common validation sets and informative point placement remain useful distinctions from earlier trials.

Current algorithm: production collector measures both drive directions, nonlinear Sampson residual in squared lengths, staged robust filtering, optional nested spool/anchor fitting; geometry-only D-optimal next sweep; rank/age/coverage patience; final candidate-specific future sweeps with own-model validity gate and shared-length anchor score. Candidates currently use different future-sweep sets. Minimum 10 points/sweep can invalidate aggressively thinned data.

Budgets (researcher imposed, revisable): up to 12 offline groups, 90 CPU minutes, up to 3 simulated collection jobs if needed; no hardware movement. Initially no movement authorized beyond simulation. Record actual consumption in run manifests. Bounded sequential fits to avoid known parallel compiler crashes.

Initial plan before experiments:

Hypotheses: H1 current score agrees with physical error (baseline falsification); H2 common future sweeps improve final ranking; H3 information-aware thinning is more robust than uniform thinning; H5 redundant encoder settling can safely be avoided (proposed, not tested).

Experiments: E01 fresh 11-fixture full-auto replay baseline; E02 common-last-sweep shared anchor comparison, own-model validity unchanged; E03 informative points, scope pending E01/E02.

Next decision: quantify E01 ranking, recovery and timing, then test E02 and E03 before choosing production edits. No result available yet.

## Evidence-driven steering
E00 exploratory re-score of prior c812e7b9 histories: common-last-sweep comparison improves 4/11, worsens 2/11; paired all-shared-sweeps dominance with minimum 2 movements still worsens five_points. Reject both as general defaults; reproduce against E01 rather than treating archived evidence as fresh fits.
E01 fresh replay in progress; first 10 fixtures reproduce selected-anchor regret up to 15.232 mm RMSE (seventh HP3).
E03 informative retrospective thinning: endpoints and >=5 points per physical direction, full-data-whitened anchor Jacobian at bootstrap-derived geometry and radius prior midpoint; no ground truth in selection. Two HP3 selection attempts fail because nominal seed invalidates some sweeps; preserve failed trial and test uniform fallback for invalid geometry. This method uses already measured observations and is an offline screening experiment, not a ready acquisition planner.
New H4/E04: exact transformed-dataset reuse within a fixed spool/filter pass may avoid repeated conversion and unlock existing identity-keyed residual caches. Source inspection found all transformed datasets rebuilt even for identical radii/buildup, although downstream caches key by identity. Profile before mutation; test exact numerical equivalence and bounded memory, not rounded parameter matching. This is a compute-speed alternative if ranking/point reduction lacks robust support. No simulated collection needed for an acquisition-equivalent computation cache.

E01 complete: 11 successful fresh replays; 407.459 total wall seconds with bounded overlapping experiment work (not isolated timing). Twenty adjacent rank/physical-error disagreements across nine fixtures; nine recoveries, six failed. Final anchor regret against all fitted candidates is 15.232 mm RMSE on seventh HP3 and 37.641 on April HP3. Truth is browser simulation effective anchors/radii.
E02 complete: fresh E01 histories reproduce E00 common-set and paired-dominance outcomes; reject both general defaults. No source ranking changes.
E03 complete: 18 analytical synthetic fits, 4 recorded informative half-point replays and 4 uniform half-point controls. Informative thinning improves bad-points but worsens ten-points, seventh and April HP3. Synthetic results remain mixed; no collection-time claim from retrospective deletion. Failed initial HP3 Jacobian trials preserved as v1; v2 uses uniform fallback when seed geometry is invalid.
E04 profiles: bad-points 5330 transform calls, 2283 duplicates, 41.000 s profiled conversion time (profiler inflates runtime). Same model parameters create new objects that miss identity-based evaluation caches.
E05 prototype exact bounded pass-local cache: all 11 reference/fresh-baseline comparisons pass at 0.01 mm, 5184 non-timing numeric event fields exactly equal, all decisions equal. Aggregate replay 360.880 s, 11.4% below E01, with workload scheduling differences; require isolated repeated timing before claiming speedup.
Implementation decision: retain 19-line exact-key, 16-entry FIFO cache in filter_pass.py. No change to collection, score, optimizer, validation or stopping. Updated existing eval-bundle test to cover near-distinct radius/buildup, eviction and cache isolation across filter passes. User steering "continue" accepted; objective unchanged.
Next decision: targeted/full tests, isolated AB/BA timings and final production replay matrix; finish report/catalog. No movement budget used; no hardware or fresh-physics accuracy result claimed.


## Completion and current decision
E06 complete: isolated three-repeat AB/BA timing, no competing calibration or physics jobs. Bad-points median 48.866 -> 47.626 s (2.5% reduction); seventh HP3 32.770 -> 29.376 s (10.4%). All twelve timing replays pass both reference and fresh-baseline comparisons; 971 non-timing numeric fields per two-fixture pair match exactly in every repetition. See E06-isolated-summary.json and E06-repeated-regressions.json. These timings include counter instrumentation and warmed JAX disk compilation cache; three repeats are limited evidence.
E07 complete: 71 targeted spool tests; 322 full non-slow production tests, two slow tests deselected; diff whitespace checks pass. Production filter source is byte-identical to the eleven-fixture tested candidate snapshot. There is no need for a second identical full matrix. Final report, searchable abstract, source diff and manifest now describe the retained exact 16-entry pass-local cache and failed ranking/thinning trials.

No fresh physics or hardware collection was used. Experiment groups E00–E07 consumed zero movement budget. Recorded replay subprocess wall time sums to 1,671.580 s (27.86 min), including profiles and all timing repetitions; this is neither CPU accounting nor total research elapsed time. Analytical synthetic fit times are separately recorded in E03-synthetic-results.json. The offline work stayed bounded and no outer prompt loop was built.

Accepted result: a small computation-only speed improvement with unchanged one-click operation and automatic stopping. Physical score disagreement and reliable point-count reduction remain unresolved. Future decision, not required unfinished work: test joint anchor/spool nuisance-aware acquisition with independent configuration holdout and actual collection-time budgets in fresh simulation worlds. Proposed settling/continuous-acquisition hypotheses remain not tested. Regenerate the catalog before handing off.
