// Host-side DDA implementation for Step 9.1

#ifdef RRF_HOST_BUILD

#include <Movement/DDA.h>
#include <Movement/Move.h>
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
// Converts coordinates to steps and calculates basic move parameters
bool DDA::Init(const RawMove& move, float startCoords[MaxAxesPlusExtruders]) noexcept
{
	Move& m = reprap.GetMove();

	// Convert start and end positions to steps
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		const float stepsPerMm = m.DriveStepsPerMm(axis);
		startSteps[axis] = static_cast<int32_t>(startCoords[axis] * stepsPerMm);
		endSteps[axis] = static_cast<int32_t>(move.coords[axis] * stepsPerMm);
	}

	// Handle extruder (simplified: just one extruder for now)
	hasExtrusion = move.hasE;
	if (hasExtrusion)
	{
		const size_t extruderDrive = MaxAxes;  // First extruder after axes
		const float stepsPerMm = m.DriveStepsPerMm(extruderDrive);
		startSteps[extruderDrive] = static_cast<int32_t>(startCoords[extruderDrive] * stepsPerMm);
		endSteps[extruderDrive] = static_cast<int32_t>(move.coords[extruderDrive] * stepsPerMm);
	}

	requestedSpeed = move.feedRate;

	// Calculate total distance (Cartesian for now)
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

	// Use reasonable defaults for acceleration (will be config-driven in Step 9.2.2)
	constexpr float maxAcceleration = 1000.0f;  // mm/s^2
	constexpr float maxDeceleration = 1000.0f;  // mm/s^2

	// Clamp speed to something reasonable if not set
	if (requestedSpeed <= 0.0f)
	{
		requestedSpeed = 100.0f;  // 100 mm/s default
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

	// For now, use hardcoded CAN driver mappings (will be config-driven in Step 9.2.2)
	// Assume board 121, drivers 0-3 for axes A-D (Hangprinter convention)
	constexpr uint8_t canBoardAddress = 121;

	Move& m = reprap.GetMove();

	// Add axis movements (for now, assume first 4 axes map to CAN drivers 0-3)
	for (size_t axis = 0; axis < 4 && axis < MaxAxes; ++axis)
	{
		const int32_t steps = endSteps[axis] - startSteps[axis];
		if (steps != 0)
		{
			DriverId driver(canBoardAddress, static_cast<uint8_t>(axis));
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
			DriverId driver(canBoardAddress, 4);  // Extruder on driver 4
			const float extrusion = static_cast<float>(steps) / m.DriveStepsPerMm(extruderDrive);
			CanMotion::AddExtruderMovement(params, driver, extrusion, false);  // No pressure advance for now
		}
	}

	// This will be called with moveStartTime from the move execution loop
	// For now we return success; actual packet emission happens in the execution loop
	return true;
}

#endif // RRF_HOST_BUILD
