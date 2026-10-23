# 013: Continue a managed program safely across turns and interruptions

Outcome: a finite allocation runs through dispatch, submission, review and
replanning across multiple distinct investigations, with durable recovery and
meaningful stop controls. Combine the implemented worker/manager/review paths;
do not create a loop that restarts the same unfinished prompt repeatedly.

Implement controller operations for run-once/run-within-allocation, status,
pause, resume, cancel-attempt and stop-program. The manager chooses the next
assignment; ordinary code validates budgets, leases, transitions and permitted
actions. Preserve reservations for review/cleanup and limit review backlog.
Use a single active worker initially; a program can remain open with no currently
funded work and a scientific question still unresolved.

Done when:

- Finite task/wall allocations, concurrency and reservations are checked before
  dispatch. Actual tool/supervisor usage is distinguished from estimates and
  worker reports. Only supported backend workload limits are called enforced;
  unavailable metering remains explicit. Late results are retained without
  silently overriding a stopped allocation or double-spending its reservation.
- Pause prevents new dispatch without implying running experiments stopped.
  Stop/cancel reaches owned launchers/supervisors and drains/cancels their jobs,
  preserves partial artifacts and reports the physical/recording stop boundary.
  Wall deadline and cleanup allowance are distinct; failures release slots safely.
- Restart reconciles leases, process birth identity, session/connection records
  and actual supervisor state. It detects orphans/stale history without PID reuse
  signals, duplicate attempts or treating a final snapshot as a live world.
- Existing live workers can be inspected/reconnected where supported. Otherwise
  report interrupted/unknown and resume from a new bounded handoff; never claim
  conversation resume restores physics or silently adopt an unowned process.
- Fake-child fault tests cover crash, stale lease, cancelled start, retry limit,
  stop during review, reused PID and controller restart. One integration transcript
  shows several distinct assignments and a fresh manager continuing the program.

Preserve current launcher/supervisor ownership instead of inventing a second
cancellation implementation. Final live production-tool evidence comes in 016.
