The CAN movements are driven by data packets of CanMessageMovementLinearShaped structs. The struct is defined like this (in RRF/CANlib/src/CanMessageFormats.h):
```
struct __attribute__((packed)) CanMessageMovementLinearShaped
{
	static constexpr CanMessageType messageType = CanMessageType::movementLinearShaped;

	uint32_t whenToExecute;							// the master clock time at which this move should start
	uint32_t accelerationClocks;					// how many clocks the acceleration phase should last
	uint32_t steadyClocks;							// how many clocks the steady speed phase should last
	uint32_t decelClocks;							// how many clocks the deceleration phase should last

	uint32_t extruderDrives : 8,					// which drivers are for extruders
			 numDrivers : 4,						// how many drivers we included (maximum is 8)
			 seq : 4,								// sequence number
			 zero1 : 8,								// was used to hold the input shaping plan for this move
			 usePressureAdvance : 1,				// true to apply PA to the extruders and accumulate partial steps
			 useLateInputShaping : 1,
			 zero2 : 6;								// unused

	static constexpr uint8_t SeqMask = 0x0f;

	float acceleration;								// the base acceleration during the acceleration segment, when the total distance is normalised to 1.0
	float deceleration;								// the base deceleration during the deceleration segment, when the total distance is normalised to 1.0

	union PerDriveValues
	{
		int32_t steps;								// net steps moved by this drive (for non-extruders)
		float extrusion;							// how many steps of extrusion to do (for extruders) including fractional parts

		void Init() noexcept
		{
			steps = 0;
		}
	};

	PerDriveValues perDrive[MaxLinearDriversPerCanSlave];

	void ClearReservedFields() noexcept
	{
		extruderDrives = 0;
		usePressureAdvance = 0;
		useLateInputShaping = 0;
		zero1 = zero2 = 0;
	}

	void DebugPrint() const noexcept;

	size_t GetActualDataLength() const noexcept
	{
		return (sizeof(*this) - sizeof(perDrive)) + (numDrivers * sizeof(perDrive[0]));
	}

	// This is called from just one place (in CanMotion::FinishMovement), so inline
	bool HasMotion() const noexcept
	{
		for (size_t drive = 0; drive < numDrivers; ++drive)
		{
			if (perDrive[drive].steps != 0)				// we rely on this being valid even if perDrive[drive] contains [positive] floating point zero
			{
				return true;
			}
		}
		return false;
	}
};
```

The original ReprapFirmware code that reads the CAN packages and transforms them to step/dir signals is in RRF/Duet3Expansion/src/
See the src/CAN and src/Movement directories.
Pay special attention to:
 - src/Movement/Move.cpp, It should contain the equations of motion.
 - src/CAN/CanInterface.cpp, The ProcessReceivedMessage handles some timestamping of the movementLinearShaped, it's defined in CanInterface.cpp.
 - src/Movement/DriveMovement.cpp, The DriveMovement::NewSegment, and potentially other movementLinearShaped related functions.

# Maintaining Smooth Motion in Duet3
The key to understanding how Duet3 maintains smooth motion across segmented moves lies in the whenToExecute field of the CanMessageMovementLinearShaped message and how the expansion board firmware processes overlapping move segments.

## Time Synchronization
All boards on the CAN bus are synchronized to a master clock.
The whenToExecute field specifies the exact time (in 48MHz clock ticks) when a move should begin.
This allows the host to schedule a sequence of moves in the future, and each expansion board will execute them at the correct time.

## Overlapping Moves
The firmware on the expansion boards is designed to handle overlapping move segments. When a new move is received, it is added to a queue for the corresponding driver. The AddSegment function in Move.cpp can split and merge segments, effectively blending the end of one move into the start of the next.
(See functions `CalcInitialSpeed`, `Move::AddSegment` including the comment above it, and `Move::AddLinearSegments` in RRF/Duet3Expansion/src/Movement/Move.cpp. Our host ReprapFirmware will have to do some things slightly differently compared to these original functions, because we don't have a constant `steps_per_mm`, but we have access to the full HangprinterKinematics and the `CartesianToMotorSteps` functions at runtime.)

Overlapping moves is how constant velocity is maintained between segments.
For a smooth, continuous motion, the whenToExecute of a subsequent move should be scheduled before the previous move has fully decelerated. The expansion board's firmware will then correctly blend these moves.

## Generating queue_step Commands
To generate Klipper queue_step commands, you need to calculate the precise timing of each step.
The movementLinearShaped message provides all the necessary information to do this. The motion is divided into three phases: acceleration, steady speed, and deceleration.

Here are the formulas and steps to calculate the step timings, derived from the logic in Move.cpp and DriveMovement.cpp:

### Total Distance Normalization
The acceleration and deceleration values are provided based on a total distance of 1.0.
You'll need to scale these by the total number of steps for the move.

### Top Speed Calculation
First, calculate the "top speed" of the move in steps per clock tick. This is derived from the total distance and the durations of the acceleration, steady, and deceleration phases. The AddMove function in Move.cpp calculates this as follows:

```cpp
const float accelDistanceExTopSpeed = -0.5 * params.acceleration * fsquare((float)params.accelClocks);
const float decelDistanceExTopSpeed = -0.5 * params.deceleration * fsquare((float)params.decelClocks);
const float topSpeed = (1.0 - accelDistanceExTopSpeed - decelDistanceExTopSpeed) / (params.accelClocks + params.steadyClocks + params.decelClocks);
```

### Step Timing Calculation
The time t for any given step n can be calculated using the following kinematic equations, which are implemented in the CalcNextStepTimeFull function in DriveMovement.cpp:

#### For the acceleration phase
 - The initial velocity, u, is topSpeed - (acceleration * accelClocks).
 - The time for step n is t = (-u + sqrt(u^2 + 2 * a * n)) / a.

#### For the steady phase
 - The time for step n is simply t = n / topSpeed.

#### For the deceleration phase:
 - The initial velocity, u, is topSpeed.
 - The time for step n (relative to the start of the decel phase) is t = (-u + sqrt(u^2 + 2 * a * n)) / a, where a is the deceleration value (negative).

### Converting to queue_step
Once you have the time for each step, you can calculate the interval between consecutive steps. The Klipper queue_step command takes an interval in clock ticks. So, for each step, the interval would be time_of_step_n - time_of_step_n-1. Since queue_step can schedule multiple steps with the same interval, you can group steps where the interval is constant or changes linearly.
 - For the steady speed phase, the interval will be constant.
 - For the acceleration and deceleration phases, the interval will change with each step. You can either issue a queue_step command for each step or use the add parameter to account for the linear change in interval over a small number of steps.

In conclusion, the ReprapFirmware log, in the form of movementLinearShaped messages, contains all the necessary information to generate Klipper queue_step commands. The key is to correctly interpret the timing information and schedule the moves to overlap, allowing the firmware's segment blending to create smooth, continuous motion.
