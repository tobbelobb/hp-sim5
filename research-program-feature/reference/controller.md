# Execution controller

`program.controller.ProgramController` coordinates allocations, dispatch, review,
pause, cancellation and recovery. `program.pool` tracks capacity across programs
sharing one store. Defaults allow one worker and one separate review slot.

The current controller requires finite wall-time and task allocations, plus
review/startup/cleanup reservations. Some numerical limits are recorded contracts,
not hard aggregate meters. The role cleanup does not remove these API requirements.

The host polls children, enforces deadlines and records cleanup. Historical
process identities never permit adoption or cancellation of an unrelated process.
Unknown ownership blocks reuse until an explicit handoff records what happened.
Each store retains its selected client transport across host recovery.
