
I actually just realized we might not need the CAN sink at all.

I looked in Move.cpp lines, 666, 690, and here on line 760:

```
					if (simulationMode < SimulationMode::partial)			// in simulation mode partial, we don't process incoming moves beyond this point
```

That's inside the `Move::MoveLoop()` function.

There's something called `SUPPORT_ASYNC_MOVES` and `SimulationMode` (found in src/GCodes/SimulationMode.h). The SimulationMode enum class looks like this:
```
enum class SimulationMode : uint8_t
{	off = 0,				// not simulating
	debug,					// simulating step generation
	normal,					// not generating steps, just timing
	partial,				// generating DDAs but doing nothing with them
	highest = partial
};
```

That looks like something we could use and integrate into our own host build. We should set SimulationMode::partial somewhere, as it will allow us to capture the motion we care about (steps + timing).

SimulationMode::partial builds the same DDA objects the planner would normally dispatch.
A fully-built DDA (post planning) contains exactly what our simulator needs:

 - Per-drive step deltas for the segment (signed counts).
 - Segment timing (duration; and typically enough to reconstruct accel/cruise/decel).
 - Start/End speeds (useful for validating continuity).

Geometry/kinematics have already been applied, so the per-axis/per-drive work is settled.

Hook a DDA export right after a DDA is finalized (i.e., after speeds/jerk/segment timing are solved) and before normal dispatch. In partial, that branch already “does nothing” with the DDA—so just emit:

For each segment:

 - seg_id (monotonic)
 - t_start_ticks (running sum) and t_duration_ticks
 - For each drive: { drive_id, step_delta }
 - v_start, v_end, a_max, sub-phase clocks

Write one JSONL per DDA, e.g.:

{"type":"dda","seg":1024,"t0":123456789,"dt":24000,
 "drives":[{"d":0,"steps":+480},{"d":1,"steps":+480}],
 "v0":120.3,"v1":118.7}

Look up each drive_id in the drive->(board,driver) map (from the config gcode) and add {board, driver} fields to each entry

We need to take the config gcode as an input argument to our binary, see an example in ai_docs/config.g.
Pay special attention to mapping from motor to number, for example for a configured CAN driver:
`M584 X40.0 Y41.0 Z42.0 U43.0 P4 ; map ABCD-axes to CAN addresses, and set four visible axes. Please excuse that ABCD motors are called XYZU here.`
