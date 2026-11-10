# Encoder CDPR autocalibration: covariance-aware data quality

Objective: reproduce score/physical-accuracy disagreements, assess covariance-based ideas from the supplied CDPR paper, and measure accuracy and runtime effects before changing production selection.

Constraints: `.venv/bin/python`; small source changes; ground truth only for offline evaluation; no hardware claims. Existing sessions already tested and rejected raw held-out residual selection, history-age removal, basic D-optimal movement, bounded updates, and several solver changes. No simulated movement was needed for this offline ranking study.

Budgets: one 11-fixture recorded regression matrix, sequential; one offline candidate-covariance audit; zero movement jobs. All eleven current-source replays completed. The initial requested invocation failed before fitting because the regression runner executes `autocal.py` as a script; a second attempt was stopped after confirming the same failure.

Experiment IDs and outcomes:
- E01 — current-source `.venv/bin/python autocal/tools/regress_calibration_logs.py --jobs 1`, after fixing module invocation: 11/11 exact versus stored references; 20 adjacent score/ground-truth disagreements across 9 fixtures; selected-history anchor-sum regret 0–163.828 mm. [Console output is in the conversation; per-fixture logs are under the ignored `autocal/data/.regress_parallel_runs/` directory.]
- E02 — paper/code/prior-session review: the paper's PDOP/ODOP are pose-error covariance from a joint pose/shared-bias estimator; not equivalent to hp-sim5's per-sweep ellipse-feature anchor information.
- E03 — offline heuristic sweep weighting (per-sweep point count divided by recorded sigma squared), selecting minimum maximum anchor standard deviation across frozen histories: rejected. It selected the physically best available history in only 2/11 fixtures and badly worsened several (flexible-lines anchor RMSE 7.492→32.145 mm; ten-points bigger deltas 5.930→8.000 mm). This heuristic is not the paper's estimator.
- E04 — D-opt planning cost audit on 33 Slideprinter / 90 HP3 candidate sweeps per saved fit: 0.039–0.819 seconds per fit, including candidate evaluation. This does not explain tens-of-seconds full-auto runs; repeated optimization/recovery dominates.

Retained source change: `autocal/tools/regress_calibration_logs.py` now launches `-m autocal.autocal`, fixing package shadowing (`'autocal' is not a package`) and enabling the requested reproduction command. The experimental Cholesky rank change was reverted because no observed data justified it. No calibration score, optimizer or stopping behavior changed.

Accepted steering: read autocal docs/code first; reproduce and measure before implementation; inspect the supplied paper; retain automatic stopping. Next decision: a valid paper-derived accuracy change needs a joint point-pose/shared-bias model or similarly justified physical covariance. Do not substitute arbitrary weights on the existing ellipse Jacobian.
