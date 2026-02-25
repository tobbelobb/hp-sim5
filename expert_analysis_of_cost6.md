Yes — this run is **exactly** the kind of counterexample that teaches how `score_ui` should be tuned.

## Why the wrong candidate (score 5.28) wins over the better fit (score 12.34)

I compared the two relevant iterations in your log:

### Candidate A (the one you consider better fit) — `score_ui = 12.34`

* `cost_raw = 0.7044`
* anchors are **quite close** to the expected slideprinter geometry (visually plausible, near previous truth-ish values)
* `R = 38.69` (reasonable)
* `chi2_red_tau_3bin_debiased = 154.9`
* `N_trim = 80 / 80` (**no trimming**)
* `rel_std = 8.286`
* strong distance bias:

  * `bias_vs_distance_slope ≈ +0.06794 mm/mm`
  * `chi2_tau_d = 13.59` (very low after tau_d)

### Candidate B (the worse fit) — `score_ui = 5.28`

* `cost_raw = 0.6125` (slightly lower raw fit)
* anchors are **geometrically wrong** (large deformation / wrong scale/shape)
* `R = 37.6` (worse)
* `chi2_red_tau_3bin_debiased = 132.5` (**lower than A**, so current score prefers it)
* `N_trim = 84 / 100` (**16% trimmed away**)
* `rel_std = 36.23` (**much worse confidence / identifiability**)
* almost no bias trend:

  * `bias_vs_distance_slope ≈ 0`
  * but `chi2_tau_d = 3234` (terrible under tau_d)

---

# What this tells us about your current `score_ui`

## The current score is too “residual-only”

Your current `score_ui` mainly rewards the candidate with the lowest **debiased + heteroscedastic-rescored residual cost**.

That worked on easier datasets, but here it gets fooled because Candidate B achieves a lower residual-based score by:

* trimming away a meaningful chunk of points
* landing in a geometrically wrong basin that still fits many points well
* having poor identifiability (huge `rel_std`), which the score currently doesn’t punish enough

### In one sentence:

**The score is underweighting reliability/identifiability and overtrusting trimmed residual fit.**

---

## Also: using `score_ui` directly for optimization magnifies the issue

This is a second (important) factor.

Because your display mapping is steep (power law), a modest core-metric difference (e.g. `154.9` vs `132.5`) becomes a **much larger score gap** (`12.34` vs `5.28`).

So the optimization starts preferring the “wrong-but-cleaner-on-trimmed-residuals” basin.

### This does *not* mean your idea was bad

It means:

* `score_ui_display` is good for UX
* but it needs a **more reliability-aware internal core** if it will also be used for ranking/optimization

---

# The key signals in this run that should be added to ranking

These three are the strongest discriminators between the two candidates:

## 1) Trim instability (very important)

Candidate B has:

* `chi2_red = 12760`
* `chi2_red_trim = 191`

That’s a **huge gap** (≈ 67×).

Candidate A has:

* `chi2_red = 653.7`
* `chi2_red_trim = 653.7` (no gap)

This tells you Candidate B only looks good **after** heavy trimming.

### Insight:

A low `chi2_tau_3bin_debiased` is not enough if it is achieved by trimming away a lot of contradictory evidence.

---

## 2) Coverage / trimming fraction

Candidate B:

* `N_trim = 84/100` (16% dropped)

Candidate A:

* `N_trim = 80/80` (0% dropped)

That’s another strong “this fit is brittle / cherry-picked” signal.

---

## 3) Uncertainty / identifiability penalty (`rel_std`)

Candidate B:

* `rel_std = 36.23`

Candidate A:

* `rel_std = 8.286`

That is a huge difference and lines up with your intuition that A is the better basin.

### Insight:

The residual fit alone is ambiguous, but uncertainty says the solution is much less trustworthy.

---

# Bonus signal (very interesting)

## `tau_d` vs `tau_3bin` disagreement

This run shows a striking contrast:

* Candidate A (better geometry): `chi2_tau_d ≈ 13.6`, `chi2_tau_3bin ≈ 154.9`
* Candidate B (worse geometry): `chi2_tau_d ≈ 3234`, `chi2_tau_3bin ≈ 132.5`

That suggests:

* Candidate A has a strong **coherent distance-dependent bias** (tau_d helps a lot)
* Candidate B does **not**, and only looks good under the more flexible 3-bin discrepancy

I wouldn’t make this a primary term yet, but it’s a valuable **suspicion flag**.

---

# What to change in `score_ui` (practical tuning)

## Keep the same core, but multiply by a reliability penalty

Use your existing core (good choice overall):

* `m_core = chi2_red_tau_3bin_debiased`

Then define:

### A) Trim penalty (must add)

```python
trim_frac = max(0.0, 1.0 - N_trim / max(N, 1))
trim_gap = max(1.0, chi2_red / max(chi2_red_trim, 1e-9))

p_trim = 1.0 + 0.8 * trim_frac + 0.5 * math.log10(trim_gap)
# optionally cap log term, e.g. min(log10(trim_gap), 2.0)
```

This will strongly penalize “looks good only after trimming” cases like the 5.28 candidate.

---

### B) Confidence penalty (must add)

```python
p_conf = 1.0 + 0.3 * max(0.0, math.log10(rel_std / 10.0))
```

This is mild for good runs, but meaningful when `rel_std` explodes.

It should flip your two candidates in this log (in the direction you want).

---

### C) Optional geometry plausibility penalty (nice to have, machine-type specific)

If you know the machine is a Slideprinter and have a reasonable prior shape envelope, add:

* pairwise distance asymmetry penalty
* origin norm spread penalty
* R plausibility band penalty

This would catch the 5.28 candidate very effectively.

But this is optional if you want the score to stay machine-agnostic.

---

# Recommended split (important)

## 1) `rank_score_internal` for optimization / candidate selection

Use a smoother, reliability-aware metric:

```python
m_rank = chi2_red_tau_3bin_debiased * p_trim * p_conf
rank_score = math.log1p(m_rank)   # smaller is better
```

This is what I’d optimize / rank on.

## 2) `score_ui_display` for the user

Map from `m_rank` to your UI score scale:

```python
score_ui = (m_rank / M0) ** alpha
```

Then re-calibrate `M0` and `alpha` so your desired semantics hold:

* ~1 for best runs
* > 10 for clear bad fits

This preserves your UX while making optimization sane.

---

# Why this run is valuable (big-picture insight)

It shows that on hard datasets, there are at least two kinds of “good-looking” minima:

1. **Physically/plausibly good geometry with systematic residual structure**
2. **Geometrically wrong but aggressively trimmed / flexible-discrepancy-compatible fits**

Your current score mostly distinguishes by (2)-style residual fit, so it can pick the wrong one.

### The fix is not to abandon `score_ui`

It’s to make it **two-dimensional in spirit**:

* fit quality
* reliability / trustworthiness

---

# Concrete next-step patch (coder-ready direction)

Add these metrics into the score computation path (same place you compute `score_ui` now):

* `N`, `N_trim`
* `chi2_red`, `chi2_red_trim`
* `rel_std`

Then update:

```python
m_core = chi2_red_tau_3bin_debiased

trim_frac = max(0.0, 1.0 - N_trim / max(N, 1))
trim_gap = max(1.0, chi2_red / max(chi2_red_trim, 1e-9))
p_trim = 1.0 + 0.8 * trim_frac + 0.5 * min(2.0, math.log10(trim_gap))

p_conf = 1.0 + 0.3 * max(0.0, math.log10(rel_std / 10.0))

m_rank = m_core * p_trim * p_conf
rank_score = math.log1p(m_rank)          # use for optimization/ranking
score_ui = (m_rank / M0) ** alpha        # use for display (recalibrate M0, alpha)
```

---

## One last practical note

If you want a fast safety check before fully retuning constants:

* log both old and new `m_rank`
* compare just on this hard run and the previous “good” logs

You should see the 12.34 candidate move ahead of the 5.28 one **without** wrecking your earlier cases.

Set M0 to 95 and alpha to 5 initially.
