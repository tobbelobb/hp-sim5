# Network-enabled preflight revalidation

Objective: retest tooling improvement 3 now that command networking is enabled, fix reproduced bugs.
Hypotheses: command socket/bind/HTTP succeed; preflight detects live firmware incompatibility; active test shutdown leaves no owned services.
Constraints: no doctor or full-auto calibration smoke, no movement in the attached validation worlds, preserve prior trials. Standard tests may move isolated fixtures. Preflight 15 s; fixed suites <=900 s.
Accepted steering: user enabled networking and requested test/verify/fix.
Next decision: reproduce firmware mismatch and run all relevant tests.

Reproduced bugs:
- [Firmware drift](firmware-mismatch-reproduction.json): read-only live firmware object model showed Cartesian while preflight passed; attached validation world stayed at step zero. Fix checks live kinematics and compares anchor coordinates against startup configuration with 0.1 mm tolerance (firmware report rounding).
- [Reaped detached leader](failed-cleanup-reproduction.txt): cleanup skipped the surviving grandchild because leader PID no longer existed. Fix checks a surviving member's private job marker, revalidates ownership before each signal, and retains the marker in test-owned autocal children.

Post-fix checks so far: 40 focused tests passed; 3 detached-grandchild and live graceful/signal shutdown checks passed; 3 explicit live-launcher tests passed. Named supervised suites passed 26 and 138 tests through an SDK client restricted by seccomp. Live preflight passed in 0.738 s; CLI preflight passed in 0.840 s. Existing attached HP4 world and recording remained unchanged. Current command socket/bind/HTTP all pass. The full direct regression result is pending.

Final result: 165 direct regressions passed (16 deselected), plus 3 live launcher tests. Final SDK-supervised suites passed 26 and 138 tests; healthy preflight 0.738 s, CLI 0.840 s. Both reproduced bugs fixed; stale ownership preserves unrelated group. Command networking fully passed. Objective complete; next decision none required.
