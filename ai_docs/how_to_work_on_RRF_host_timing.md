# Working on RRF Host Timing

This note summarizes how RepRapFirmware (RRF) keeps time on real hardware, how the host build emulates that behaviour, and the heuristics I used to track down the recent determinism regressions (extra pauses, early exits, and the Hangprinter logo stall at `scheduled-completed = 59`). I treat it as a living overview—extend it when you uncover new subtleties.

---

## 1. Timing on the MCU

1. Every Duet board runs a *free‑running* step timer. In RRF the timer frequency is called `StepClockRate` (`RepRapFirmware.h`) and is 750 kHz on Duet 3 (48 MHz / 64). The timer never stops; it is read through `StepTimer::GetMovementTimerTicks()` (eventually `HostTiming::StepClocks64()` in the host build).

2. The movement planner (`DDA`, `DDARing`, `Move`) schedules each move at an absolute tick boundary:
   - `DDA::Prepare()` freezes the move, sets `afterPrepare.moveStartTime`, and records the duration `clocksNeeded`.
   - If the planner thinks the previous move already finished, it injects the *AbsoluteMinimumPreparedTime* (25 ms) hiccup to avoid modifying a move that might be “in flight”. This is the thing we must avoid on host: the real timer never jumps to the end of the previous move instantaneously.

3. Completion is detected with `DDA::HasExpired()`. It compares `StepTimer::GetMovementTimerTicks()` with `afterPrepare.moveStartTime + clocksNeeded`. As long as the timer lags behind the move’s end time, the DDA stays committed and the scheduler knows something is running.

4. `DDARing::Spin()` is called from the Move task. It retires moves whose end time already passed (`HasExpired()`), commits new ones, and reports lookahead stats, laser work, etc. On hardware this function is driven by interrupts and never blocks waiting for wall-clock time.

5. `HostCanCapture::LogMotion()` logs every CAN move (or local move) with the absolute `when_to_execute` (tick count) and the individual phase clocks (accel, steady, decel). That log is what we compare for determinism.

---

## 2. How the host build emulates time

The host build replaces the MCU interfaces with shims in `RRF/host/src/HostTiming.cpp` and `RRF/host/include/Movement/StepTimer.h`.

### What the shims expose

| Function | Host implementation (after the latest fixes) |
|----------|----------------------------------------------|
| `HostTiming::StepClocks64()` | Returns `g_virtualClockTicks`, an atomic counter we advance manually. |
| `HostTiming::AdvanceStepClocks(delta)` | Adds `delta` to the virtual counter and records stats (Simulation vs WaitLoop vs Delay). |
| `HostTiming::ReportSimulationClocks(delta)` | Keeps track of the last simulation tick for diagnostics but **does not** advance the virtual clock anymore. All real advancement happens when `DDARing` retires a move. |
| `StepTimer::GetMovementTimerTicks()` | Reads `HostTiming::StepClocks64()` and is what the firmware uses as “now”. |

### How time is advanced

1. **Planner completion path**: inside `DDARing::Spin()` we now advance the virtual clock exactly once per committed move. When `HasExpired()` returns true we call `AdvanceStepClocks(clocksNeeded)` and record it as `ClockStatKind::Simulation`. This mimics the “time spent executing the move” on real hardware.

2. **Logging / captures**: CAN capture logging uses the same absolute start times and durations, so once the clock is advanced deterministically the log is deterministic as well. We no longer try to “fast forward” in the wait loop because the planner itself is authoritative about elapsed ticks.

3. **Wait loop fallback**: the host `main.cpp` still has a small wait-loop advance to bootstrap the very first captures (`AdvanceStepClocks(75000)` when no captures exist). After the first move we rely on `DDARing` exclusively.

4. **Host-specific heuristics**: the only new heuristic we added is “if the ring has already completed at least one move and currently only has provisional moves, force `shouldStartMove = true`”. This keeps the host simulation from stalling if wall-clock milliseconds stop increasing (e.g., when the process is paused under heavy CPU load).

---

## 3. Debugging process for the stalled Hangprinter logo

1. **Detection**: Running `./RRF/build/rrf_simulator … Hangprinter_logo6.gcode` under `head -n 130` showed the console stuck at `scheduled-completed=59`, `captureIdle=25`, and `GetVirtualStepClocks()` frozen. The CAN log (`logs/Hangprinter_logo6_2.jsonl`) also stopped growing, proving that no new moves were being scheduled even though the planner still had queued work (`scheduled` kept growing).

2. **Hypothesis**: the planner was waiting for `millis()` to advance before committing the next move (because the next move needs to start at least `gracePeriod` milliseconds after the previous addition). If the host’s `millis()` value stalls (e.g., the OS pauses the process), the condition `millis() - whenLastMoveAdded >= gracePeriod` never becomes true, so `shouldStartMove` stays false and `DDARing::Spin()` never commits the next move. This is unhittable on hardware—real time never rewinds and the Move task is woken by interrupts—but totally plausible on the host because we are emulating time in software.

3. **Evidence**:
   - The Hangprinter run kept printing `[wait] … scheduled=NN completed=NN-2 scheduled-completed=2` forever—`NoLiveMovement()` was false, so `millis()` was the only gate left.
   - `GetVirtualStepClocks()` froze as soon as the wait loop repeated, meaning `AdvanceStepClocks()` was no longer called from anywhere (planner stalled), reinforcing the idea that the next move was never committed.

4. **Fix**:
   - Instead of fiddling with `millis()`, I altered the flow inside `DDARing::Spin()`: once at least one move has completed (`completedMoves != 0`), we treat “ring is provisional only but not idle” as a signal to force `shouldStartMove = true`. This bypasses the milliseconds gate and lets the planner continue scheduling new moves, which in turn keeps advancing the virtual clock.
   - This change has no effect on real hardware, because `completedMoves` increments only after a move retired (which cannot happen unless the hardware timer was increasing). On the host the condition restores forward progress whenever wall-clock time stalls.

5. **Validation**:
   - `timeout 240 ./run_draw_squares_determinism_test.sh 200` → 200 ✅ runs.
   - Full Hangprinter logo run completes in 195 s and emits ~216k CAN captures without stalling.
   - The determinism logs (`test_draw_squares3.jsonl`) match the original golden file byte-for-byte except for the auto-generated time stamp on the first line.

---

## 4. Tips for future work

1. **Watch `scheduled-completed`**: In host traces printed from `main.cpp`, a rising `scheduled` with a stuck `completed` is a strong signal that `DDARing::Spin()` is not committing new moves. If `captureCount` is still climbing, the CAN logging path is alive; focus on the planner.

2. **Check `Move.GetSimulationTime()` vs `GetVirtualStepClocks()`**: If simulation time keeps increasing but the virtual clock stalls, the planner is accumulating floating-point simulation time (from DDA / dwell code) but not advancing the step timer, which will eventually trip safety heuristics (e.g., the 25 ms hiccup). Fix the clock mismatch first.

3. **Keep virtual time slightly behind**: Never let `GetMovementTimerTicks()` jump *past* the end of the current move. The practical trick we use is the `minNow` clamp in `DDA::Prepare()`: at most `AbsoluteMinimumPreparedTime` behind the previous end time. If you add new host timing sources, respect this invariant.

4. **Minimal host-only changes**: Resist the temptation to change movement math or planner behaviour under `#if RRF_HOST_BUILD`. Instead feed the host timing shim with the same inputs the MCU uses. That keeps the host build deterministic while avoiding surprises when you port the logic back to hardware.

5. **Stress tests**: Whenever you touch timing, run both short deterministic tests (`run_draw_squares_determinism_test.sh`) and large files with heavy queues (`Hangprinter_logo6.gcode`). The latter tends to expose scheduling stalls (`millis()` issues, wait-loop starvation, etc.).

Feel free to append further observations—especially if you discover other subtle interactions between the host timing shim and planner guarantees. The more we document, the easier it will be to keep the simulator deterministic. 
