# Autocal hardening and robustness

## Completed scope

Reviewed `1254e..5f718` and the Fable conversation; implemented default offsets, checked/adaptive rough collection, model-purity/restart protections, correct profile lookup and offset-aware held-out predictions. Retired new zero-tension collection. Independent reviews and focused regressions are preserved in the program investigations.

Built full simulator and recorded-data comparison tools. Baseline and hardened batteries cover nominal HP3/HP4, two softer HP3 variants and worn HP4 bearings. Same-data arms compare offsets/filtering and both production fitters; a separate joint-range least-squares baseline fits its own offsets and point positions. Seeded recorded-length corruption is covered. Full simulated encoder noise and hardware are not covered.

Home correction is implemented only for independently justified extension evidence. Ordinary collection has no such reference and therefore does not apply it. The four-high/cube pilot establishes concrete model blockers and a staged extension plan.

## Findings that change the next decision

Accuracy remains uneven. The expanded collector reaches actual heights near 1.73 m and passes the nominal HP3 coordinate gate at 0.91 mm (0.63 mm aligned), while nominal HP4 remains 4.65/2.99 mm. Half-stiffness HP3 and worn HP4 give 1.63/0.895 mm and 2.33/0.818 mm with final return checks. Worn HP4 exercises real rejected-boundary recovery and finishes normally. Different collection costs prevent attribution to one intervention. Offsets strongly help the new HP3 recording, but point/joint fits on new HP4 can improve when offsets are removed. Filtering can lower cost while worsening geometry.

The rough gate can accept estimates about 25 mm from truth. It checks local observability and convergence for planning, not systematic physical bias. Independent joint least squares retains substantial errors on problematic recordings; solver replacement alone has not removed them.

A bounded terminal-validation pilot collected one fresh sweep without refitting. It worsened nominal HP3, left soft HP3 unchanged and modestly improved HP4. Independent review found live application defects. The live option was removed; the negative evidence and offline scorer/replay remain. Shared alias, write and interrupt issues were fixed.

## Priority after the Fable crossover

The offline reference replay and cross-fitting remain useful controls. The user's subsequent steering led to [production progressive exploration](validation/workspace-expansion/report.md), removing the permanent ~390 mm ceiling while retaining initial half-turn reserve and verifying later transmission locally. Point-geometric substitution on new recordings removes the local fitting error. A fresh review confirms that the 50-pole motor cogging term explains almost all recorded nominal force-error variance, without establishing its causal share of anchor bias. The [next experiment plan](next_steps.md) now prioritizes matched phase averaging/dither and same-axis opposite-motion observations using learned targets, followed by an identifiable physical-home protocol. Physical truth and actual forces remain evaluation inputs only.

## Open research beyond this delivery

- Establish which force tracking, compliance and observation-model errors explain the remaining shape bias, with paired collection budgets and repeated/noise trials.
- Measure an independent compatible home reference without double-counting existing firmware flex compensation.
- Before new-method four-high support, solve affine gravity equilibrium with nonnegative tensions and branch/gauge tests. Before cube support, enforce all fixed-length constraints with active/slack sets and bounded multidimensional tension allocation.

The canonical report and topic synthesis distinguish implemented checks, observed benefits, counterexamples and unvalidated physical behavior.

This delivery is closed: all four final simulations and independent reviews completed, no owned service remains running, and selected replay stages plus local-only raw-file hashes are preserved. The next agenda items are research work, not unfinished background jobs.
