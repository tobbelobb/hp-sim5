I parsed your `.cpuprofile` (≈50.00 s span) and the pattern is very clear. Short version: your **quality monitor’s drain loop** is the #1 hotspot by a mile, **idle time is huge**, and the **ECS lookups** are still a meaningful tax.

Here’s the punch list, ranked by impact:

# 1) `_drainPendingExtrusions` (quality-monitor.js:407) is the furnace

* **Self time:** **10.81 s** (21.6% of total runtime)
* **Inclusive:** **11.16 s**
* Call chain:

  ```
  klipperCommanderWorker.onmessage → runFinalCheck → _drainPendingExtrusions
  ```
* Children under it are tiny by comparison:

  * `_recordExtrusionActive` total **0.35 s** (with pieces like `_accumulateCornerSamples` **0.09 s**, `_updateCoverageForExtrusion` **0.07 s**)
  * `_projectToPath → evaluateSegment` **0.17 s**
  * Everything else is ~milliseconds
* **What this means:** The **looping/queue-draining mechanics themselves** dominate. It’s classic when a queue is drained with `Array.shift()` or similar copy-heavy patterns.

**Fix now (very likely win):**

* If you do `while (arr.length) arr.shift()` (or shift in a loop), replace with an index-based drain:

  ```js
  // Before (bad on big arrays):
  while (pending.length) {
    const ev = pending.shift();
    handle(ev);
  }

  // After (no copying):
  for (let i = 0; i < pending.length; i++) {
    handle(pending[i]);
  }
  pending.length = 0; // clear quickly
  ```
* If you need incremental batches:

  ```js
  let head = 0;
  const BATCH = 4096;
  while (head < pending.length) {
    const end = Math.min(head + BATCH, pending.length);
    for (let i = head; i < end; i++) handle(pending[i]);
    head = end;
    // yield only if necessary to keep UI responsive:
    // await new Promise(requestAnimationFrame);
  }
  pending.length = 0;
  ```
* Consider storing events in **typed arrays** (SoA) and processing with index loops to minimize property reads and function call overhead.

# 2) You’re yielding a lot (big “idle” slice)

* **Idle self time:** **22.21 s** (**44.4%** of total).
* Scheduler/runner frames are big inclusively:

  * `iterate` (runner.js:230): **9.29 s** inclusive
  * `runAsapBatch` (runner.js:174): **8.98 s** inclusive
* **Interpretation:** Your “ASAP” slices are small and you yield frequently. That’s good for interactivity, but it doubles wall time.

**Fix:**

* For “Finish ASAP”, increase per-slice budget from ~8–12 ms to **24–30 ms** (still under 1 frame at 30–40 Hz).

  ```js
  async function runAsapBatch(step, done, SLICE_MS = 24) {
    while (!done()) {
      const t0 = performance.now();
      do { step(); } while (!done() && (performance.now() - t0) < SLICE_MS);
      // Only yield if the browser has pending input:
      if (navigator.scheduling?.isInputPending?.()) {
        await new Promise(requestAnimationFrame);
      }
    }
  }
  ```
* **Render at most once per rAF**, not per micro-step.

# 3) ECS lookups are still costly in aggregate

* **Self time:**

  * `getComponent` (ecs.js:30) **1.58 s**
  * `hasComponent` (ecs.js:35) **0.65 s**
  * `query` (ecs.js:59) **0.27 s** (0.92 s inclusive)
* **ecs.update** (line 101) frames together account for **8.90 s** inclusive.
* **Fix:**

  * Precompute **views** once per system tick (array of entity ids satisfying the component set).
  * Use **struct-of-arrays (typed arrays)** for component storage; index directly by entity id.
  * Lift `getComponent/hasComponent` out of inner loops entirely.

# 4) Worker message path is the trigger, not the sink

* `klipperCommanderWorker.onmessage` **12.28 s** inclusive (but only **1.09 s self**).
* Most of that time is the quality monitor (`runFinalCheck → _drainPendingExtrusions`).
* **Fix:**

  * Don’t run full `runFinalCheck` on every worker tick. **Throttle** it (e.g., at most once per X ms or at end of a slice).
  * If feasible, **move the QC drain** into a **Worker** and return summary metrics; keep UI updates on main thread.

# 5) Rendering is not your bottleneck

* Canvas calls visible in self time: `fill` **0.87 s**, `arc` **0.28 s** total.
* `renderSystem.update` inclusive **1.42 s**.
* So you’re **JS/logic bound**, not paint bound.

---

## Top functions (by inclusive time, excluding meta)

1. **klipperCommanderWorker.onmessage** — **12.28 s** (24.6%)
2. **runFinalCheck (quality-monitor.js:338)** — **11.19 s** (22.4%)
3. **_drainPendingExtrusions (quality-monitor.js:407)** — **11.16 s** (22.3%)
4. **iterate (runner.js:230)** — **9.29 s** (18.6%)
5. **runAsapBatch (runner.js:174)** — **8.98 s** (18.0%)
6. **ecs.update (ecs.js:101)** — **8.90 s** (17.8%)
7. **cable_joints_core.update** — **2.78 s** (5.6%)

## Top functions (by self time, excluding meta)

1. **_drainPendingExtrusions** — **10.81 s** (21.6%)
2. **getComponent** — **1.58 s** (3.2%)
3. **klipperCommanderWorker.onmessage** — **1.09 s** (2.2%)
4. **fill (Canvas)** — **0.87 s** (1.7%)
5. **_processCommand (slideprinter_common.js:223)** — **0.71 s** (1.4%)
6. **hasComponent** — **0.65 s** (1.3%)
7. **_updateAttachmentPoints** — **0.61 s** (1.2%)

---

## Concrete next steps (most impact first)

1. **Rewrite `_drainPendingExtrusions`** to avoid any `shift()`/copying — drain by index; clear with `pending.length = 0`. If you already don’t use `shift()`, profile again after:

   * Hoist loop-invariant reads, avoid tiny helper calls inside the inner loop.
   * Consider SoA/typed arrays for the event fields you touch.

2. **Throttle or relocate `runFinalCheck`:** run at end of batch (or every N ms), not per message; or push its computation into a **Worker** and send back a compact result.

3. **Increase slice budget** for “ASAP” mode to 24–30 ms and **render only once per rAF**.

4. **ECS fast path:** precomputed views + typed arrays; remove per-entity `getComponent/hasComponent` calls from inner loops.

5. **Messaging batching:** if you push many small messages, batch them and use **Transferables** (TypedArrays) to reduce JSON/GC churn. Your `_processCommand` and `onmessage` self time suggest some win here.

If you want, paste `_drainPendingExtrusions` and the queue structure here — I’ll give you a drop-in, allocation-free rewrite that keeps your current return values but kills the O(n²) behavior.
