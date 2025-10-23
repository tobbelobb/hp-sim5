// Host-side DDA implementation for Step 9.1

#ifdef RRF_HOST_BUILD

#include <Movement/DDA.h>
#include <Movement/Move.h>
#include <Movement/MovementError.h>
#include <Platform/RepRap.h>
#include <CAN/CanMotion.h>
#include <CAN/CanInterface.h>
#include <Platform/Platform.h>
#include <cmath>
#include <cstring>

DDA::DDA() noexcept
	: checkEndstops(false)
	, isPrintingMove(false)
	, moveStartTime(0)
	, clocksNeeded(0)
	, requestedSpeed(0.0f)
	, hasExtrusion(false)
{
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		endSteps[i] = 0;
		startSteps[i] = 0;
	}
}

// Initialize DDA from RawMove
// Step 9.3.2: Now uses real Kinematics::CartesianToMotorSteps() instead of 1:1 Cartesian transform
bool DDA::Init(const RawMove& move, float startCoords[MaxAxesPlusExtruders]) noexcept
{
	Move& m = reprap.GetMove();

	// Step 9.3.2: Use real Kinematics to transform start position to motor steps
	const MovementError startErr = m.CartesianToMotorSteps(startCoords, startSteps, true);
	if (startErr != MovementError::ok)
	{
		// Could not transform start position
		return false;
	}

	// Step 9.3.2: Use real Kinematics to transform end position to motor steps
	const MovementError endErr = m.CartesianToMotorSteps(move.coords, endSteps, true);
	if (endErr != MovementError::ok)
	{
		// Could not transform end position (unreachable, etc.)
		return false;
	}

	// Handle extruder (simplified: just one extruder for now)
	// Extruders are not transformed by kinematics, they're direct linear drives
	hasExtrusion = move.hasE;
	if (hasExtrusion)
	{
		const size_t extruderDrive = MaxAxes;  // First extruder after axes
		const float stepsPerMm = m.DriveStepsPerMm(extruderDrive);
		startSteps[extruderDrive] = static_cast<int32_t>(startCoords[extruderDrive] * stepsPerMm);
		endSteps[extruderDrive] = static_cast<int32_t>(move.coords[extruderDrive] * stepsPerMm);
	}

	requestedSpeed = move.feedRate;

	// Calculate total distance in Cartesian space (user coordinates)
	float totalDistanceSquared = 0.0f;
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		const float axisDelta = move.coords[axis] - startCoords[axis];
		totalDistanceSquared += axisDelta * axisDelta;
	}
	params.totalDistance = sqrtf(totalDistanceSquared);

	// Zero move check
	if (params.totalDistance < 0.001f && !hasExtrusion)
	{
		return false;  // Nothing to do
	}

	isPrintingMove = hasExtrusion && (params.totalDistance > 0.001f);

	return true;
}

// Prepare DDA: compute trapezoid profile and call CanMotion APIs
bool DDA::Prepare() noexcept
{
	// Step timer frequency (48 MHz for Duet 3)
	constexpr float StepClockFrequency = 48000000.0f;

	Move& m = reprap.GetMove();

	// Step 9.2.2: Use configured acceleration values
	// For now, use the first axis acceleration as representative (will be improved for multi-axis)
	float maxAcceleration = m.GetAcceleration(0);  // Get configured acceleration
	float maxDeceleration = maxAcceleration;       // Same for deceleration

	// Clamp speed to something reasonable if not set
	if (requestedSpeed <= 0.0f)
	{
		requestedSpeed = 100.0f;  // 100 mm/s default
	}

	// Clamp requested speed to configured maximum
	float maxSpeed = m.GetMaxFeedrate(0);  // Get configured max speed
	if (requestedSpeed > maxSpeed)
	{
		requestedSpeed = maxSpeed;
	}

	// Simple trapezoid profile calculation
	// For now, no S-curve support (SUPPORT_S_CURVE is typically 0 in host build)

	const float totalDistance = params.totalDistance;
	if (totalDistance < 0.001f && !hasExtrusion)
	{
		return false;
	}

	// Calculate time needed to accelerate to requested speed
	const float accelTime = requestedSpeed / maxAcceleration;
	const float accelDistance = 0.5f * maxAcceleration * accelTime * accelTime;

	// Calculate time needed to decelerate from requested speed
	const float decelTime = requestedSpeed / maxDeceleration;
	const float decelDistance = 0.5f * maxDeceleration * decelTime * decelTime;

	// Check if we can reach requested speed
	if (accelDistance + decelDistance <= totalDistance)
	{
		// Can reach requested speed with steady phase
		params.topSpeed = requestedSpeed;
		params.acceleration = maxAcceleration;
		params.deceleration = -maxDeceleration;
		params.accelDistance = accelDistance;
		params.decelStartDistance = totalDistance - decelDistance;

		params.accelClocks = static_cast<uint32_t>(accelTime * StepClockFrequency);
		const float steadyDistance = totalDistance - accelDistance - decelDistance;
		const float steadyTime = steadyDistance / requestedSpeed;
		params.steadyClocks = static_cast<uint32_t>(steadyTime * StepClockFrequency);
		params.decelClocks = static_cast<uint32_t>(decelTime * StepClockFrequency);
	}
	else
	{
		// Triangle profile: can't reach requested speed
		// Calculate peak speed we can reach
		const float peakSpeed = sqrtf(maxAcceleration * maxDeceleration * totalDistance / (maxAcceleration + maxDeceleration));
		params.topSpeed = peakSpeed;
		params.acceleration = maxAcceleration;
		params.deceleration = -maxDeceleration;

		const float accelDist = peakSpeed * peakSpeed / (2.0f * maxAcceleration);
		params.accelDistance = accelDist;
		params.decelStartDistance = accelDist;

		const float tAccel = peakSpeed / maxAcceleration;
		const float tDecel = peakSpeed / maxDeceleration;
		params.accelClocks = static_cast<uint32_t>(tAccel * StepClockFrequency);
		params.steadyClocks = 0;
		params.decelClocks = static_cast<uint32_t>(tDecel * StepClockFrequency);
	}

	params.useInputShaping = false;  // No input shaping for now

	// Ensure all PrepParams fields are initialized for safety
	// (The non-S-curve branch above should have initialized everything, but double-check)
	#if !SUPPORT_S_CURVE
	// Already initialized: accelClocks, steadyClocks, decelClocks
	// Already initialized: acceleration, deceleration
	// Already initialized: accelDistance, decelStartDistance
	#endif
	// Already initialized: totalDistance, topSpeed, useInputShaping

	clocksNeeded = params.TotalClocks();

	// Now call CanMotion APIs to emit movement packets
	CanMotion::StartMovement();

	// Step 9.2.2: Use configured driver mappings from M584
	// Add axis movements using the configured driver IDs
	for (size_t axis = 0; axis < reprap.GetGCodes().GetVisibleAxes() && axis < MaxAxes; ++axis)
	{
		const int32_t steps = endSteps[axis] - startSteps[axis];
		if (steps != 0)
		{
			DriverId driver = m.GetAxisDriverId(axis);
			CanMotion::AddAxisMovement(params, driver, steps);
		}
	}

	// Add extruder movement if present
	if (hasExtrusion)
	{
		const size_t extruderDrive = MaxAxes;
		const int32_t steps = endSteps[extruderDrive] - startSteps[extruderDrive];
		if (steps != 0)
		{
			// For now, use a default CAN driver for extruder (will be config-driven later)
			// Assume board 0, driver 0 as placeholder (real config would come from M584 E parameter)
			DriverId driver;
			driver.SetLocal(0);  // Local driver 0 for extruder
			const float extrusion = static_cast<float>(steps) / m.DriveStepsPerMm(extruderDrive);
			CanMotion::AddExtruderMovement(params, driver, extrusion, false);  // No pressure advance for now
		}
	}

	// This will be called with moveStartTime from the move execution loop
	// For now we return success; actual packet emission happens in the execution loop
	return true;
}

#endif // RRF_HOST_BUILD
