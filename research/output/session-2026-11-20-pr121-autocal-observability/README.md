# PR #121 autocal: what the fit cannot observe (2026-11-20)

Bounded checks on the PR #121 tree (extracted to `.git/autocal-pr121.C3v88C`), run with 1 BLAS thread and an 800 MB address-space limit (peak RSS ~85 MB).

- `evidence/check.py` → `evidence/results.json`: spool-scale gauge, repeated or changed load against compliance, one lower anchor off the shared plane.
- `evidence/height_sweep.py` → `evidence/height_sweep.json`: hidden pose error against how far one lower anchor sits off the shared plane.

Findings:
1. Scaling radius and buildup together by s scales every length by exactly s, so the residual is 0 while the anchors are off by 38 mm (HP4 at 2%). Only the buildup curvature, about 0.6 mm rms, separates scale from geometry.
2. A 10% compliance error, refit, moves top z by about 0.8 mm and leaves 0.005 mm rms. Raising the load by 25% raises the separating signal only from 1.49 to 1.86 mm per unit of compliance.
3. The shared-low-z parameterization (the Hangprinter default) gives about 0.37 mm of max pose error per mm of anchor height offset. The training rms is 0.0036 mm per mm, which stays under 0.05 mm encoder noise until the offset passes about 14 mm.
4. All benchmark truths are two idealized geometries: lower anchors exactly coplanar at z=-280, top anchor at (0,0,1900), r=30, buildup 0, radii not fitted. That makes findings 1–3 invisible to the PR's scoring.
