# Fixed encoder associations and absolute scale in the current calibration model

Status: complete, awaiting separate review.

## Abstract

Three offline experiments tested fixed encoder sensitivity, absolute scale and excitation at source revision `c0f111c310cc562b7122bf606a59c60de758adbb`. Reordering fixed encoder observations across points changed no residual in either NumPy or JAX. Each condition retained 75 synthetic or 60 frozen simulated HP3 residuals per backend. Positive controls changed residuals by 0.16 to 1.13 mm. Scaling anchors, spool radii and the dimensionful buildup coefficient together preserved clean synthetic consistency, with RMS residual below 1e-12 mm across 300 points. Holding buildup or one radius fixed broke that invariance. Diverse fixed-length sweeps improved the weakest observable local direction by 7.51 times, while the common-scale direction remained unobservable. One radius observation with assumed 0.05 mm uncertainty raised local rank from 13 to 14. An initially inactive noise-normalization comparison was preserved and corrected after identifying the production 0.05 mm floor. These results concern source sensitivity and synthetic observation symmetry. Production priors, fixed spool values and noise choices can constrain scale separately. No fitting, physics, collection, hardware accuracy or implemented fix is claimed.

## Topics

encoder-only calibration, HP3, Hangprinter, NumPy, JAX, fixed encoder association, spool buildup, absolute scale, identifiability, excitation, radius constraint

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Per-point fixed encoder association is discarded while aggregate fixed information remains active | E1 | Supported | [E1 residuals and controls](E1.json) |
| Free anchors, radii and dimensionful buildup admit a common-scale observation symmetry | E2 | Supported | [E2 raw data](E2.json.gz) |
| Anchors/radii alone always remain invariant with fixed nonzero buildup | E2 | Rejected | [Dimensional controls](summary.json) |
| Diverse encoder-only sweeps remove that scale direction | E3 | Rejected in tested model | [Jacobians](E3.json) |
| One independently known radius detects the scale direction | E3 | Supported locally | [Radius control](summary.json) |

## Decision handoff

- Scope: current source revision c0f111c310cc562b7122bf606a59c60de758adbb, synthetic kinematics and frozen simulated HP3 observations. No fitting, collection or physics.
- Review: worker measured and checked finite coverage and controls; independent review is pending. No hardware accuracy or implemented fix is established.
- Decision: bounded model_gap evidence supports inspecting pointwise fixed geometry and independent scale information. Production noise floors, priors and nominal spool constraints remain separate from observation symmetry.
- Next: an offline per-point fixed-length residual prototype and an audit of measured versus nominal spool/buildup constraints could change the implementation decision. These are not tested here.

## References

- [Report](report.md): methods, results, controls, limitations and actual source references.
- [Research record](research.md): scope, budgets and corrected trial.
- [Review summary](summary.json): compact measured quantities and checks.
- [Reproduction script](analysis.py), [inputs](inputs.json) and [source/machine manifest](manifest-all.json): self-contained experiment setup and provenance.
- [Pinned GitHub source references and equality check](source-links.json): origin/main at 7fc26ba4 has identical autocal source; producer revision remains c0f111c3.
- [Archive manifest](manifest-archive.json): recommended assets and compressed/raw hashes, including the preserved initial E2 trial.
