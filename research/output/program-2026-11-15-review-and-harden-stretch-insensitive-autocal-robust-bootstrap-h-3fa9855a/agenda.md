# Autocal hardening and robustness

## Completed scope

Reviewed `1254e..5f718` and the Fable conversation; implemented default offsets, checked/adaptive rough collection, model-purity/restart protections, correct profile lookup and offset-aware held-out predictions. Retired new zero-tension collection. Independent reviews and focused regressions are preserved in the program investigations.

Built full simulator and recorded-data comparison tools. Baseline and hardened batteries cover nominal HP3/HP4, two softer HP3 variants and worn HP4 bearings. Same-data arms compare offsets/filtering and both production fitters; a separate joint-range least-squares baseline fits its own offsets and point positions. Seeded recorded-length corruption is covered. Full simulated encoder noise and hardware are not covered.

Home correction is implemented only for independently justified extension evidence. Ordinary collection has no such reference and therefore does not apply it. The four-high/cube pilot establishes concrete model blockers and a staged extension plan.

## Findings that change the next decision

Accuracy remains uneven: the hardened cases range from 1.68 to 4.82 mm worst coordinate error, all above the 1 mm gate. Half-stiffness HP3 improves relative to the earlier trial, while nominal HP3 and worn HP4 worsen. Different collection costs prevent attribution to one intervention. Offsets clearly help one common multi-height recording but harmed old single-height data. Filtering can lower cost while worsening geometry.

The rough gate can accept estimates about 25 mm from truth. It checks local observability and convergence for planning, not systematic physical bias. Independent joint least squares retains substantial errors on problematic recordings; solver replacement alone has not removed them.

A bounded terminal-validation pilot collected one fresh sweep without refitting. It worsened nominal HP3, left soft HP3 unchanged and modestly improved HP4. Independent review found live application defects. The live option was removed; the negative evidence and offline scorer/replay remain. Shared alias, write and interrupt issues were fixed.

## Priority after the Fable crossover

The offline reference replay and cross-fitting are complete; the [next experiment plan](next_steps.md) prioritizes reproducing the passing fixed plan in current physics, then passing exactly that plan through firmware. The current point fitter passes the best saved observations, while the original Fable fitter misses 1 mm on current full-autocal observations. A ~390 mm planning budget and reconstructed 40–423 mm height coverage differ materially from the 0–1200 mm direct reference. Resolve that collection/protocol gap before running another broad unchanged battery. Shape and absolute-home requirements remain separate.

## Open research beyond this delivery

- Establish which force tracking, compliance and observation-model errors explain the remaining shape bias, with paired collection budgets and repeated/noise trials.
- Measure an independent compatible home reference without double-counting existing firmware flex compensation.
- Before new-method four-high support, solve affine gravity equilibrium with nonnegative tensions and branch/gauge tests. Before cube support, enforce all fixed-length constraints with active/slack sets and bounded multidimensional tension allocation.

No live investigation or simulator job remains pending. The canonical report and topic synthesis distinguish implemented checks, observed benefits, counterexamples and unvalidated physical behavior.
