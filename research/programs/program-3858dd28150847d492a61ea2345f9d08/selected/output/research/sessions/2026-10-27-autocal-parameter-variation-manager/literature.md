# Encoder-only calibration ideas and source limits

Literature search performed 2026-10-27 through the connected web search tool. These notes guide experiment choice and do not count as new numerical evidence.

Yuan, Han; You, Xianghui; Zhang, Yongqing; Zhang, Wenjing; Xu, Wenfu, 2019. A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation. DOI https://doi.org/10.3390/app9112182. Publisher https://www.mdpi.com/2076-3417/9/11/2182. Indexed publisher text and the bibliographic record describe calibration using redundant encoder cable lengths and least squares in a simulated 3-DOF four-cable robot. This supports redundancy closure as a candidate. It does not establish performance with HP3 force-dependent errors. Direct publisher retrieval returned HTTP 429, so no full-text derivation was inspected.

Ida, Edoardo; Merlet, Jean-Pierre; Carricato, Marco, 2019. Automatic self-calibration of suspended under-actuated cable-driven parallel robot using incremental measurements. DOI https://doi.org/10.1007/978-3-030-20751-9_28. Institutional abstract https://cris.unibo.it/handle/11585/691786. The abstract describes a geometricostatic least-squares method using incremental cable lengths AND swivel pulley angles. It supports combining equilibrium with geometry as a research route, but its sensor assumptions exceed encoder-only HP3. Only the institutional abstract was inspected.

First-principles candidates before detailed history comparison:

- Difference redundant squared cable lengths to remove a latent point norm and exploit low-rank geometric consistency. Unknown reference lengths and curvature determine whether the anchor gauge is identifiable.
- Reverse force and movement direction through repeated regions to measure hysteresis. Direction-dependent encoder discrepancies can indicate friction or incomplete settling rather than geometry.
- Compare several preload levels at repeated constrained geometries to separate compliance from anchor errors. Nominal motor torque is not guaranteed to equal cable tension.
- Fit simple geometry first, then add nuisance parameters only after a residual test identifies the missing effect. Joint fitting can hide a weak global scale direction.
- Choose additional sweeps where credible solvers disagree in encoder predictions. Solver disagreement without distinguishable new predictions does not justify movement.
- Test every point-thinning or stopping rule on varied synthetic worlds, then on native physics. A gain on one nominal layout is insufficient.

The first two sources are motivation, not authority for any selected algorithm. Archived studies already report failures of sparse sphere closure, blanket thinning and held-out prediction selectors. Those negative results argue for narrowly scoped tests rather than repeating the same default changes.
