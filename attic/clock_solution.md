We're working on a x86_64 version of ReprapFirmware.
Read the details in RRF/README.md

The fake timing and clocks are a relatively new feature and we've not managed to make it run predictably and deterministically.

Right now we depend on so called "fast forwarding" on lines 552-555 in RRF/host/src/main.cpp to even get the program to finish small gcode samples.

Changing the `yield()` for `sleep_for(std::chrono::milliseconds(1))` made the program much slower but it doesn't seem to have changed much else. The large sample gcodes still hang after finishing the whole gcode file, and runs cpu usage through the roof at that point.

I'm starting to feel that we have the wrong approach. We want complete determinism, with zero probability of difference between runs. We don't want to tweak the advance value or depend on it for even finishing. Something is fundamentally wrong in how we advance our clock in the simulation program.

It should be so easy, we should
 1. Read a gcode line, then segment it.
 2. Calculate the next motor position (at the end of the segment)
 3. Calculate how many ticks that will take.
 4. Construct the rest of the CAN packet data and send/log the CAN packet.
 5. Advance the clock with the same number of ticks as is contained in that packet.
 6. Try to get a new segment. If none, get a new gcode line and segment it. If no gcode lines, we're done.

Or, get as close as possible to this simplicity in this Spin() based ReprapFirmware program.

We can implement something simple and deterministic like this in practice.

We are hooking into the DDA::Prepare function. It is the one who ultimately does the `CanMotion::FinishMovement`, which in turn calls our `CanInterface::SendMotion`, which logs the motions to disk. I'm attaching their definitions here:
```
// Dispatch this DDA to the move segment queue for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare(DDARing& ring,
#if SUPPORT_S_CURVE
					MovementProfile& plannedProfile,
#endif
					uint32_t prepareAdvanceTime, SimulationMode simMode) noexcept
{
	PrepParams params;
#if SUPPORT_S_CURVE
	if (flags.useScurve)
	{
		AllocateMoveFromPlan(plannedProfile, params);
	}
	else
#endif
	{
#if SUPPORT_S_CURVE
		debugPrintf("2nd order move\n");
#endif
		params.SetFromDDA(*this);
	}
	params.useInputShaping = flags.xyMoving
							&& !(   flags.isolatedMove
								 || flags.isLeadscrewAdjustmentMove
#if SUPPORT_SCANNING_PROBES
								 || flags.scanningProbeMove
#endif
								) ;
#if SUPPORT_LASER
	if (topSpeed < requestedSpeed && reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		// Scale back the laser power according to the actual speed
		laserPwmOrIoBits.laserPwm = (Pwm_t)((laserPwmOrIoBits.laserPwm * topSpeed)/requestedSpeed);
	}
#endif

	// Decide when this move should start.
	// Avoid setting the move start time in the past or with very little time before it starts, because this can lead to us trying to modify a segment that is already executing
	const uint32_t now = StepTimer::GetMovementTimerTicks();
	if (prev->state == committed)
	{
		const uint32_t prevEndTime = prev->afterPrepare.moveStartTime + prev->clocksNeeded;
		if ((int32_t)(prevEndTime - now) >= (int32_t)MoveTiming::AbsoluteMinimumPreparedTime)
		{
			afterPrepare.moveStartTime = prevEndTime;		// start this move directly after the previous one
		}
		else if (startSpeed == 0.0)
		{
			afterPrepare.moveStartTime = now + prepareAdvanceTime;
		}
		else
		{
			afterPrepare.moveStartTime = now + MoveTiming::AbsoluteMinimumPreparedTime;
			reprap.GetMove().AddPrepareHiccup();		// move was supposed to follow the previous one directly, so record a hiccup
		}
	}
	else
	{
		afterPrepare.moveStartTime = now + prepareAdvanceTime;
	}

	if (simMode < SimulationMode::normal)
	{
#if SUPPORT_CAN_EXPANSION
		CanMotion::StartMovement();
#endif
		// Handle all drivers
		Move& move = reprap.GetMove();
		if (flags.isLeadscrewAdjustmentMove)
		{
			move.EnableDrivers(Z_AXIS, false);			// ensure all Z motors are enabled
		}

		float extrusionFraction = 0.0;
		AxesBitmap additionalAxisMotorsToEnable, axisMotorsEnabled;
		afterPrepare.drivesMoving.Clear();
		MovementFlags segFlags;
		segFlags.Clear();
		segFlags.checkEndstops = flags.checkEndstops;
		segFlags.noShaping = !params.useInputShaping;
		segFlags.nonPrintingMove = !flags.isPrintingMove;
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			if (flags.isLeadscrewAdjustmentMove)
			{
				// We don't set any bits in drivesMoving because setting the Z bit would be misleading, and setting individual driver bits isn't useful because it doesn't take account of CAN-connected drivers.
				// For a leadscrew adjustment move, the first N elements of the direction vector are the adjustments to the N Z motors
				const AxisDriversConfig& config = move.GetAxisDriversConfig(Z_AXIS);
				if (drive < config.numDrivers)
				{
					const int32_t delta = lrintf(directionVector[drive] * totalDistance * move.DriveStepsPerMm(Z_AXIS));
					const DriverId driver = config.driverNumbers[drive];
					if (delta != 0)
					{
#if SUPPORT_CAN_EXPANSION
						if (driver.IsRemote())
						{
							CanMotion::AddAxisMovement(params, driver, delta);
						}
						else		// we don't generate segments for leadscrew adjustment moves to remote drivers
#endif
						{
							move.AddLinearSegments(driver.localDriver + MaxAxesPlusExtruders, afterPrepare.moveStartTime, params, (motioncalc_t)delta, segFlags);
						}
					}
				}
			}
			else
#if SUPPORT_ASYNC_MOVES
			if (ownedDrives.IsBitSet(drive))
#endif
			{
				if (drive < reprap.GetGCodes().GetTotalAxes())
				{
					// It's a linear axis
					int32_t delta = endPoint[drive] - prev->endPoint[drive];
					if (delta != 0)
					{
						move.EnableDrivers(drive, false);
						if (flags.continuousRotationShortcut && reprap.GetMove().GetKinematics().IsContinuousRotationAxis(drive))
						{
							// This is a continuous rotation axis, so we may have adjusted the move to cross the 180 degrees position
							const int32_t stepsPerRotation = lrintf(360.0 * move.DriveStepsPerMm(drive));
							if (delta > stepsPerRotation/2)
							{
								delta -= stepsPerRotation;
							}
							else if (delta < -stepsPerRotation/2)
							{
								delta += stepsPerRotation;
							}
						}

						delta = move.ApplyBacklashCompensation(drive, delta);

						// We generate segments even for nonlocal drivers so that the final position is correct and to track the position in near real time
						move.AddLinearSegments(drive, afterPrepare.moveStartTime, params, (motioncalc_t)delta, segFlags);
						afterPrepare.drivesMoving.SetBit(drive);

#if SUPPORT_CAN_EXPANSION
						const AxisDriversConfig& config = move.GetAxisDriversConfig(drive);
						for (size_t i = 0; i < config.numDrivers; ++i)
						{
							const DriverId driver = config.driverNumbers[i];
							if (driver.IsRemote())
							{
								CanMotion::AddAxisMovement(params, driver, delta);
							}
						}
#endif
						axisMotorsEnabled.SetBit(drive);
						additionalAxisMotorsToEnable |= reprap.GetMove().GetKinematics().GetControllingDrives(drive, flags.checkEndstops);
					}
				}
				else
				{
					// It's an extruder drive
					if (directionVector[drive] != 0.0)
					{
						const size_t extruder = LogicalDriveToExtruder(drive);

						// Check for cold extrusion/retraction. Do this now because we can't read temperatures from within the step ISR, also this works for CAN-connected extruders.
						// Don't check if it is a special move (indicated by flags.checkEndstops) because the 'tool' variable isn't valid for those moves
						if (simMode != SimulationMode::off || flags.checkEndstops || Tool::ExtruderMovementAllowed(tool, directionVector[drive] > 0.0, extruder))
						{
							move.EnableDrivers(drive, false);

							if (flags.isPrintingMove && directionVector[drive] > 0.0)
							{
								extrusionFraction += directionVector[drive];					// accumulate the total extrusion fraction
							}

#if SUPPORT_NONLINEAR_EXTRUSION
							// Add the nonlinear extrusion correction to totalExtrusion.
							// If we are given a stupidly short move to execute then clocksNeeded can be zero, which leads to NaNs in this code; so we need to guard against that.
							if (flags.isPrintingMove && clocksNeeded != 0)
							{
								const NonlinearExtrusion& nl = move.GetExtrusionCoefficients(extruder);
								float& dv = directionVector[drive];
								const float averageExtrusionSpeed = (totalDistance * dv * StepClockRate)/(float)clocksNeeded;		// need speed in mm/sec for nonlinear extrusion calculation
								const float factor = 1.0 + min<float>((nl.A + (nl.B * averageExtrusionSpeed)) * averageExtrusionSpeed, nl.limit);
								dv *= factor;
							}
#endif

							const motioncalc_t delta = totalDistance * directionVector[drive] * move.DriveStepsPerMm(drive);

							// We generate segments even for nonlocal extruders in order to track extruder position
							move.AddLinearSegments(drive, afterPrepare.moveStartTime, params, delta, segFlags.AddIsExtruder());

#if SUPPORT_CAN_EXPANSION
							const DriverId driver = move.GetExtruderDriver(extruder);
							if (driver.IsRemote())
							{
								// The MovementLinearShaped message requires the extrusion amount in steps to be passed as a float. The remote board adds the PA and handles fractional steps.
								CanMotion::AddExtruderMovement(params, driver, delta, flags.usePressureAdvance);
							}
#endif
							afterPrepare.drivesMoving.SetBit(drive);
						}
					}
				}
			}
		}

		// On CoreXY and similar architectures, we also need to enable the motors controlling any connected axes
		additionalAxisMotorsToEnable &= ~axisMotorsEnabled;
		while (additionalAxisMotorsToEnable.IsNonEmpty())
		{
			const size_t drive = additionalAxisMotorsToEnable.LowestSetBit();
			additionalAxisMotorsToEnable.ClearBit(drive);
			move.EnableDrivers(drive, false);
		}

		afterPrepare.averageExtrusionSpeed = (extrusionFraction * totalDistance * (float)StepClockRate)/(float)clocksNeeded;

		state = committed;																// must do this before we call CheckEndstops
#if SUPPORT_SCANNING_PROBES
		if (flags.scanningProbeMove)
		{
			move.PrepareScanningProbeDataCollection(*this, params);
		}
		else
#endif
		if (flags.checkEndstops)
		{
			// Before we send movement commands to remote drives, if any endstop switches we are monitoring are already set, make sure we don't start the motors concerned.
			// This is especially important when using CAN-connected motors or endstops, because we rely on receiving "endstop changed" messages.
			// Moves that check endstops are always run as isolated moves, so there can be no move in progress and the endstops must already be primed.
			BasePriorityBooster booster(NvicPriorityStep);								// shut out the step interrupt
			(void)move.CheckEndstops(false);											// this may modify pending CAN moves
		}

#if SUPPORT_CAN_EXPANSION
		const uint32_t canClocksNeeded = CanMotion::FinishMovement(*this, afterPrepare.moveStartTime, simMode != SimulationMode::off);
		if (canClocksNeeded > clocksNeeded)
		{
			// Due to rounding error in the calculations, we quite often calculate the CAN move as being longer than our previously-calculated value, normally by just one clock.
			// Extend our move time in this case so that the expansion boards don't need to catch up.
			clocksNeeded = canClocksNeeded;
		}
#endif

		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves))		// show the prepared DDA if debug enabled
		{
			DebugPrint("pr");
		}

#if DDA_MOVE_DEBUG
		MoveParameters& m = savedMoves[savedMovePointer];
		m.accelDistance = accelDistance;
		m.decelDistance = decelDistance;
		m.steadyDistance = totalDistance - accelDistance - decelDistance;
		m.requestedSpeed = requestedSpeed;
		m.startSpeed = startSpeed;
		m.topSpeed = topSpeed;
		m.endSpeed = endSpeed;
		m.targetNextSpeed = targetNextSpeed;
		m.endstopChecks = endStopsToCheck;
		m.flags = flags;
		savedMovePointer = (savedMovePointer + 1) % NumSavedMoves;
#endif
	}
	else
	{
		state = committed;
	}
}

uint32_t CanMotion::FinishMovement(const DDA& dda, uint32_t moveStartTime, bool simulating) noexcept
{
	uint32_t clocks = 0;
	if (simulating)
	{
		FreeMovementBuffers();											// it turned out that there was nothing to move
	}
	else
	{
		CanMessageBuffer *buf = movementBufferList;
		if (buf != nullptr)
		{
			MutexLocker lock((dda.IsCheckingEndstops()) ? &stopListMutex : nullptr);
			do
			{
				CanMessageBuffer * const nextBuffer = buf->next;		// must get this before sending the buffer, because sending the buffer releases it
				CanMessageMovementLinearShaped& msg = buf->msg.moveLinearShaped;
				if (msg.HasMotion())
				{
					msg.whenToExecute = moveStartTime;
					uint8_t& seq = nextSeq[buf->id.Dst()];
					msg.seq = seq;
					seq = (seq + 1) & 0x7F;
					buf->dataLength = msg.GetActualDataLength();
					if (dda.IsCheckingEndstops())
					{
						// Set up the stop list
						DriversStopList * const sl = new DriversStopList(stopList, buf->id.Dst());
						const size_t nd = msg.numDrivers;
						sl->numDrivers = (uint8_t)nd;
						for (size_t i = 0; i < nd; ++i)
						{
							sl->stopStates[i] = (msg.perDrive[i].steps != 0) ? DriverStopState::active : DriverStopState::inactive;
						}
						stopList = sl;
					}
					CanInterface::SendMotion(buf);								// queues the buffer for sending and frees it when done
					clocks = currentMoveClocks;
				}
				else
				{
					CanMessageBuffer::Free(buf);
				}
				buf = nextBuffer;
			} while (buf != nullptr);

			movementBufferList = nullptr;
		}
	}
	return clocks;
}

void SendMotion(CanMessageBuffer* buf) noexcept
{
    if (buf != nullptr)
    {
        HostCanCapture::LogMotion(*buf);
        CanMessageBuffer::Free(buf);
    }
}
```

Somewhere in this code, there's a number which is the optimal number of ticks to advance so we can do that deterministically, not through some kind of 1ms sleep, or busy loop or anything like that.

During `DDA::Prepare()` the planner computes, for each move, both the absolute start tick (`afterPrepare.moveStartTime`) and the number of step‑timer ticks the move will take (`clocksNeeded`).  The segments are prepared with the full look‑ahead, input‑shaping and kinematics logic – by the time `Prepare()` calls `CanMotion::FinishMovement()` the motion packets contain all drive step deltas and the planned `whenToExecute` for that segment, and `clocksNeeded` holds the duration in `StepTimer` ticks.  On CAN‑connected axes the prepared step buffer and its start time are handed off to the expansion board at that moment.

`FinishMovement()` returns `canClocksNeeded`, the number of ticks needed by the CAN move after any rounding. The caller keeps its own `clocksNeeded`.  Immediately afterwards `Prepare()` does:

```c++
uint32_t canClocksNeeded = CanMotion::FinishMovement(*this, afterPrepare.moveStartTime,
                                                     simMode != SimulationMode::off);
if (canClocksNeeded > clocksNeeded) {
    clocksNeeded = canClocksNeeded;
}
```

Thus, for every prepared DDA you have:

* `afterPrepare.moveStartTime` – when the move should start;
* `clocksNeeded` – how many `StepTimer` ticks it should run.

If you maintain a `masterClock` in the host simulator, the deterministic way to advance it is simply:

```c++
masterClock = afterPrepare.moveStartTime;  // schedule the next CAN message
... send/log the motion buffer ...
masterClock += clocksNeeded;               // advance to the end of the move
```

Since each `DDA::Prepare()` call computes `afterPrepare.moveStartTime` from the end time of the previous move (or from `now + prepareAdvanceTime` when starting a new print), the moves will be laid out back‑to‑back with the minimal gaps required by the firmware’s look‑ahead logic.  By using `clocksNeeded` rather than a hard‑coded “advance” value you avoid hanging at the end of files: when there are no more DDAs, the simulator simply stops advancing because no new `Prepare()` calls occur.

In other words, there isn’t a magic constant hiding somewhere – the optimal tick advance is the per‑DDA `clocksNeeded` value that the planner already computes.  If you export `afterPrepare.moveStartTime` and `clocksNeeded` (or, equivalently, the start time and duration of each segment) in your logging hook, you can drive the virtual clock exactly as the firmware would, with zero randomness between runs.

To make the simulator advance time deterministically instead of relying on heuristics, the changes need to be made where the host currently “fast‑forwards” the master clock and where CAN packets update the virtual clock.  The two key areas are in `WaitForPrintCompletion` (which drives the main loop) and in `HostCanCapture::LogMotion` (which records each movement).

1. **Remove the “fast‑forward” heuristic in the print‑completion loop.**
   In `RRF/host/src/main.cpp` the `WaitForPrintCompletion()` loop tracks `fastForwardAttempts` and, when no new CAN captures are observed, it bumps the clock forward by up to `3600` step‑timer ticks on each iteration.  This is the code that yields non‑deterministic results because it advances time arbitrarily:

   ```cpp
   // … inside WaitForPrintCompletion
   ++fastForwardAttempts;
   const uint64_t latestFinish = HostCanCapture::GetLatestFinishMasterClock();
   if (latestFinish != 0) {
       const uint64_t advance = static_cast<uint64_t>(std::min<unsigned int>(fastForwardAttempts, 3600U)) *
                                HostTiming::StepClockFrequencyHz;
       HostTiming::EnsureMasterClockAtLeast(latestFinish + advance);  // <-- pushes the clock forward
   }
   ```

   Replace this block with deterministic advancement.  A simple fix is to stop adding the `advance` term altogether.  Whenever no new captures arrive, call `HostTiming::EnsureMasterClockAtLeast(latestFinish)` to bring the virtual clock exactly to the finish of the last scheduled segment.  Alternatively, remove the entire fast‑forward section and rely on the clock advancement done in `HostCanCapture::LogMotion` as described below.

2. **Advance the virtual clock by each segment’s exact duration.**
   In `RRF/host/can/CanCapture.cpp`, `HostCanCapture::LogMotion()` computes the segment start time (`whenStep`) and duration (`accelStep + steadyStep + decelStep`).  It then converts them into master‑clock units and currently calls `HostTiming::EnsureMasterClockAtLeast(finishMaster)`, which jumps the clock to the absolute end of the segment.  To achieve repeatability:

   * Compute the segment duration in master‑clock ticks:

     ```cpp
     const uint64_t durationMaster = durationStep * MasterClocksPerStepTick;
     ```

   * Replace `HostTiming::EnsureMasterClockAtLeast(finishMaster)` with a call to increment the master clock by this exact amount:

     ```cpp
     HostTiming::AdvanceStepClocks(durationMaster);
     ```

     This advances `g_virtualClockTicks` by exactly the segment’s duration rather than forcing it to match an absolute timestamp.  Keep `UpdateLatestFinish(finishMaster)` so that `WaitForPrintCompletion()` can still detect when the last movement finishes.

3. **Remove the overshoot guard in `AdvanceStepClocks` (optional).**
   `HostTiming::AdvanceStepClocks()` currently limits how far it will move the virtual clock: it clips the target to `latestFinish + StepClockFrequencyHz` (i.e. 1 s beyond the latest CAN finish).  When you remove the fast‑forwarding logic and advance the clock by known durations, this guard will unnecessarily prevent the clock from progressing if segments are long.  Simplify the function by deleting the `if (target > maxAllowed) target = maxAllowed;` test so that it always adds the requested tick count.

With these changes, each call to `HostCanCapture::LogMotion()` will move the clock forward by the exact number of ticks contained in the motion message, and `WaitForPrintCompletion()` will no longer have to guess how far to fast‑forward.  The simulator’s master clock will match the firmware’s planned start and end times, producing deterministic runs without depending on a tunable “advance” value.
