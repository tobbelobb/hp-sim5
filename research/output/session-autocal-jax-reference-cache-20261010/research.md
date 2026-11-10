# JAX cache transfer to the HP3 extended reference

Objective: replay `output/extended-autocal/no-buildup-reference-001/sweeps.json`
and compare cache disabled, cache warmed only on the regression battery, and
cache warmed on this dataset. Accepted steering: determine whether the warm
cache is a battery-specific shortcut or a general compilation optimization.

Hypotheses: (H1) cache modes preserve all numerical calibration outputs;
(H2) regression-warmed executables transfer to this dataset; (H3) warming on
this dataset reduces replay time even when H2 does not hold.

Constraints: no collection or machine movement; immutable source dataset,
original log and prior regression cache. Isolated dataset and cache copies.
Unchanged regression runner functions and production autocal. Seven sequential
replays, two each for off/regression-warm/target-warm plus one target-cold.
Budget: seven replays, at most 30 minutes of compute. Compiler debug logging is
enabled equally in all arms to distinguish objective hits from utility hits.

Experiment IDs: D1 off, D2 regression-warm, D3 cold-empty, D4 target-warm,
D5 off, D6 regression-warm from another pristine copy, D7 target-warm.
Regression comparisons use D1 as baseline, separately compare the original
recorded log to D1.

Completed observations and adjustments:

- Initial regression parser rejected anchor-only fixed-radius logs, including
  the self-comparison. Retained preliminary results. `checks.py` supplies only
  the known fixed CLI radii (30 mm each), then calls unchanged parser/reporting
  helpers. Raw logs remain unmodified. All comparisons pass after adaptation.
- D1/D5 cache-off averaged 24.18894 s; D3 cold-empty took 24.96011 s;
  D4/D7 target-warm averaged 16.79403 s. Dataset warm saved 7.39492 s (30.57%).
- D8-D11 added diagnostic logging and full-precision fit checkpoints. Copies
  of caches at different paths continued missing even after warming. This
  falsified the assumption that copying cache files preserves usable keys.
- Local JAX 0.9.2 code revealed why: the default GPU auxiliary cache option
  contains the persistent cache directory, is set even for CPU compilations,
  and is hashed in compile options. D2/D6/D8/D9/D11 are relocated-cache
  controls, excluded from transfer conclusions. No logging-induced numerical
  change occurred. No production settings were changed.
- Expanded compute budget from seven to fifteen replays (still below thirty
  minutes, no movement) to correct transfer methodology and capture precision.
- D12/D13 temporarily moved the original regression cache aside, copied it
  back to the same path, replayed, archived the mutated copy, then restored
  the original. Byte manifests match before/after. They averaged 23.34588 s.
- D14, the same-path diagnostic, reused 4 objective variants plus one utility
  executable, compiled 12 objective variants. D15, at the target cache's
  original path, reused all 16 objective variants and the utility executable,
  with zero misses. D15 produced full-precision checkpoints for comparison.
- All fifteen replays exit zero and pass five-iteration plus final-summary
  comparisons. Full-precision run metrics match the original recording;
  checkpoint anchors/costs/covariance and other numerical fit fields match
  between cache-off D10, warm D15 and the original recording. Summary remains
  score 8161 and the original bad anchors. Cache does not cause or fix this fit.

Decision: retain current cache settings. The optimization transfers according
to compiled shapes/settings, with partial battery-to-reference reuse and a
larger benefit once this dataset's variants are cached. No production change
is needed. See report and analysis.json for valid comparisons and limitations.
