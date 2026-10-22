# HP4: restore single-step performance

User steering: four physics steps are too expensive. Restore default one; retain substeps only as optional. Verify simulated autocal without relaxing stability acceptance. User stopped reference-005 after reaching iteration 4 because it was extremely slow.

Constraint: small source change, no performance claims without matched evidence. No hardware. Preserve reference-005 artifacts. Budget: one isolated production native full-autocal trial with event recording, plus focused default/opt-in tests. No parameter sweep or new stability investigation in this turn.

H1: default one step avoids automatic fourfold integration work. Code restores JS/Python defaults, bypasses rigid/cable queries at one step, and makes loaded-state substep regression explicit opt-in.
H2: retained damping fixes allow fresh HP4 full-autocal at one step. E01 running, no result available yet. Uses --headless-sim production native physics, original machine and speedup, fresh output, events recording. This differs from user's browser and external full recorder; no browser throughput/equivalence claim.

Preflight: isolated launcher --preflight passed; command socket probe passed with local-network execution. Trial command and artifacts in sweeps.full_auto.log and sweeps.headless/manifest.json. Following prior session ../2026-10-16-hp4-effector-instability/report.md.

E01 interrupted deliberately: headless --speedup 25 advances 25 times the requested wait duration and reports a divided collector clock. This does not match the current browser simulation-clock semantics. No autocal success result; no headless timing code changed in this task.

E02 running: restarted local recorder at 9877, reloaded existing stopped simulator tab, and launched the user's exact --sim/--speedup 25/--extended-reference-ws command with only the output dataset relocated to browser/sweeps.json. Fresh browser selection/collection, default one step; source changes are only defaults, avoiding unnecessary component queries, explicit opt-in regression and documentation.

Focused verification: JS scheduler and setup 11 passed; Python scheduler 2 passed; Python composition default assertion 1 passed; explicit four-step loaded-state regression 1 passed. No added acceptance tolerance or motion workaround. Original reference-005 data preserved.

E02 failed: original browser command timed out during size tuning at 120000 simulation ms after force tuning succeeded. No relaxation. Analysis of final RRD shows approximately 2 mm/s continuing horizontal travel near deadline and small rotation; later frozen post-error samples excluded. Default one restored, but original-command completion remains unresolved. Stopped owned test services and paused browser. No further experiments in this turn.
