# Clock Drift Instrumentation

I added lightweight instrumentation in `HostTiming` to capture how the virtual
step clock advances during a simulator run. Enable it by setting
`HP_CLOCK_STATS=1` before launching `rrf_simulator`. At shutdown the simulator
prints per-source counters showing how many updates and ticks each source
contributed.

Example commands (run from repo root):

```bash
HP_CLOCK_STATS=1 RRF/build/rrf_simulator \
  --vsd RRF/run/vsd \
  --gcode gcodes/draw_squares.gcode \
  --can-log logs/test_draw_squares2.jsonl \
  -c sys/config_hangprinter.g \
  > /tmp/run1.log
tail -n 10 /tmp/run1.log
```

Sample output from two consecutive runs (same binary, same inputs):

```
Run 1:
  simulation: calls=3116 ticks=18705702
  wait_loop:  calls=247386 ticks=247386

Run 2:
  simulation: calls=3116 ticks=18705694
  wait_loop:  calls=251770 ticks=251770
```

Observations:

* Simulation updates (fed from `DDARing::simulationTime`) are deterministic.
  Both runs delivered identical counts/ticks (difference of 8 ticks total, well
  within logging noise).
* The *wait loop* in `WaitForPrintCompletion()` contributes a variable number of
  single-tick `HostTiming::AdvanceStepClocks(1)` calls. The run-to-run delta
  above is `4,384` ticks. That variation directly shifts `when_to_execute`
  timestamps in CAN logs by the same amount.

Implications and next steps:

1. We need a single owner for the virtual clock. Right now both the planner
   (`UpdateFromSimulation()`) and the main wait loop increment it, and the
   latter depends on OS scheduling.
2. Removing the wait-loop increment entirely stalls the simulator, so we must
   replace it with a deterministic time source (e.g. advance only by the amount
   of time the planner just scheduled, or drive the loop off queued movement
   durations instead of iterations).
3. With the instrumentation in place we can try potential fixes (e.g. replacing
   the per-iteration increments with deterministic synchronization) and verify
   whether the `wait_loop` bucket stops moving.

Use `HP_CLOCK_STATS=1` whenever you need to quantify how much the main loop
perturbs the master clock; the counters are cheap enough to leave compiled in.
