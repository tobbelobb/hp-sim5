# Potential issues

## Circle assumption vs real physics
Cable sag, flex, gravity, and torque-mode dynamics mean the effector path is approximately but not exactly the pure circle defined by fixed lengths.

Our Fix/comment: This will be fixed later when the more complete flex compensation, buildup compensation, forward kinematics logic will be included into the autocalibration code.
That's why it's important for us now to store raw data in the form of "encoder readings", not some kind of "guessed line lengths" or anything like that,
since our method of interpreting the raw data will evolve as we include more and more of the advanced physics modelling later.
For now however, we just want a minimal viable implementatino with a simple physics model (no sag, no flex, no spool buildup).

## Elliptic cost function design
Directly comparing (A,B,C,D,E,F) is unsafe due to scale ambiguity and geometric non-uniformity.
A parameterization like (x_0, y_0, a, b, θ) or a geometric distance measure is safer.

Our Fix/comment: In Phase 1, for each sweep: Fit ellipse, then convert it to center, semi-axes, and orientation: (x_0, y_0, a, b, θ). Enforce a consistent ordering (e.g.
a >= b, and orientation in some fixed range). In Phase 2: From anchors, predict the theoretical ellipse, convert to the same (x_0, y_0, a, b, θ) parametrization, and use a weighted sum of squared differences in those parameters.
That avoids the raw scale ambiguity of (A,B,C,D,E,F) and makes the cost more interpretable.

## Information loss vs. raw sweeps
Ellipse fitting is a strong form of data compression. It’s mostly fine, but you do lose the detailed distribution of points along the ellipse (e.g. which parts of the arc you sampled most, how noise varies along the path).
In rare edge cases, that extra info might help disambiguate ambiguous anchor configurations or diagnose non-elliptic behavior.

Our Fix/comment: Improve observability. Make visible

 - which parts of the arc was sampled most,
 - how noise varies along the path
