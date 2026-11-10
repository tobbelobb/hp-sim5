# Autocal recording-mode cost on a frozen production trace

Status: in progress. Date: 2026-10-15.

Objective: give research agents measured time/storage costs for none, events,
sampled and full, with fair workload comparison before any performance claim.
User steering: repeated autocal runs and differing engines need not agree; verify
equivalence explicitly. Ballpark results suffice.

Hypothesis H1: removing physics recording materially reduces collection replay
wall time and disk usage while preserving physics and command/encoder evidence.
H2: stride-100 sampled recording offers an intermediate cost.

Method: replay a bounded prefix of the saved accepted HP3 production full-autocal
job 2ec0b06774624962a18bdbd2818eb134 (2026-10-10). Freeze the original scene and
step-indexed bridge/encoder trace. Use the current native Rust backend, unlayered
HP3, fixed 2 ms timestep and production external-command controller/recorder.
Never rerun the adaptive collector or optimizer for timing comparisons. Validate
full state every 1000 steps and every encoder read exactly across all cases;
validate identical normalized recorded event streams by RRD readback. Reject
comparisons if any equality gate fails. Do not assert parity with the older
source engine: report source encoder drift separately.

Budgets: isolated replay only; no attached/live printer motion or calibration
acceptance. Pilot: 20,000 steps, one run/mode. Main intended budget: 200,000 steps,
four runs/mode, rotated mode order; 3.2 million measured steps, 80,000 warmup
steps, at most 300 seconds per replay plus 60 seconds recorder drain. No other
benchmarks/tests run concurrently during measurement. Retain failed trials.

Timing: report replay+drain+RRD finalization separately from complete trial wall
including recorder startup, Node import/world construction/JIT warmup. RRD
readback is outside timers. Report RRD bytes and the shared command journal,
not a fictional total full-autocal storage estimate. The frozen workload omits
firmware planning, adaptive collection, Python fitting and collector/Python event
logs/stage artifacts. Use it to choose recording modes, not predict calibration
outcomes or full-job wall times.

Tool context: the sandbox socket probe was blocked; the approved outside-sandbox
probe passed socket creation, loopback bind and HTTP. Recorder socket benchmarks
run outside the command sandbox. No attached supervisor exists; no long live
collection is being performed.

Experiments: pilot running. Main matrix not yet tested. Next decision: inspect
pilot equivalence, sample/event counts, runtime and size; proceed with main matrix
only after the pilot passes.

Pilot result: all four 20,000-step cases passed exact cross-mode equality.
Only 32 commands occurred in this early prefix, so expanded main matrix to
200,000 steps to include the command-dense movement phase. Pilot observations
differ from the original older engine by up to 352.64 degrees; this is not
original-engine parity evidence. Cross-mode replay equality is the comparison
criterion. Main matrix is running sequentially.

Boundary audit: all 1060 nonempty encoder request/response pairs in the main
prefix occur at the same saved physics step (all 85 pilot pairs do too). The
potential request-before-drain issue therefore does not alter these measured
observations. Retain the source-engine drift as a disclosed limitation rather
than interpreting it as recording-mode error. Exact cross-mode checks continue
to pass. Archived the measurement harness under harness-v1/.

Final results: main matrix completed, all 16 equivalence gates passed. Four
additional final-harness validation trials matched main trajectory, schedule and
normalized event hashes exactly. Main medians: none 4.8206 s / 0 MB RRD; events
5.0483 s / 3.4434 MB; sampled 5.4889 s / 10.9648 MB; full 15.2297 s / 54.2337 MB.
All cases share a 6.5826 MB journal; recorded modes preserve 34,017 headless events.
H1 and H2 supported for this replay. Original-engine parity rejected (943.55 deg
maximum drift); total full-autocal speedups and alternate backends not tested.
The main matrix remains valid because its 1060 request/response boundaries are
identical; the earlier concern did not supersede the measurements. Main harness
was archived before refinement; final harness versions are in revalidation/harness.
Updated checked-in research cost guidance, README and AGENTS.md so launched agents
receive the numbers while selecting experiment modes and budgets. No new movement
or solver acceptance was requested. Final decision: evidence is sufficient for
ballpark recording-mode guidance; no further expensive trials needed.

Verification: six existing profile_headless harness tests passed outside the
command sandbox. Initial sandboxed Node test launch failed before individual
tests ran; rerunning outside the sandbox passed. Python compile, Node syntax
and git diff whitespace checks passed. Research catalog regenerated.
