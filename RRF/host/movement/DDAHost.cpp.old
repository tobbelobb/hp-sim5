#include <Movement/DDA.h>

#ifdef RRF_HOST_BUILD

#include <Movement/DDAHost.h>
#include <Movement/Move.h>
#include <Movement/StepTimerHost.h>
#include <Platform/RepRap.h>
#include <CAN/CanMotion.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

using host::DDAHost;

namespace
{
	constexpr double kTiny = 1e-9;

	inline bool axis_in_mask(const RawMove& mv, size_t axis) noexcept
	{
		if ((mv.flags & RMF_RawMotorMove) == 0)
		{
			return true;
		}
		return (mv.independentMask & (1u << axis)) != 0;
	}
}

DDA::DDA() noexcept
	: checkEndstops(false)
	, isPrintingMove(false)
	, moveStartTime(0)
	, clocksNeeded(0)
	, requestedSpeed(0.0f)
	, hasExtrusion(false)
	, plannedEntrySpeed(0.0f)
	, plannedTopSpeed(0.0f)
	, plannedExitSpeed(0.0f)
	, plannedAcceleration(0.0f)
{
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		endSteps[i] = 0;
		startSteps[i] = 0;
		startMachineCoords[i] = 0.0f;
	}
}

bool DDA::Init(const RawMove& move, float startCoords[MaxAxesPlusExtruders]) noexcept
{
	Move& m = reprap.GetMove();

	rawMove = move;
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		startMachineCoords[i] = startCoords[i];
	}

	const bool isRaw = (move.flags & RMF_RawMotorMove) != 0;

	if (!isRaw)
	{
		const MovementError startErr = m.CartesianToMotorSteps(startCoords, startSteps, true);
		if (startErr != MovementError::ok)
		{
			return false;
		}

		const MovementError endErr = m.CartesianToMotorSteps(move.coords, endSteps, true);
		if (endErr != MovementError::ok)
		{
			return false;
		}
	}
	else
	{
		for (size_t axis = 0; axis < MaxAxes; ++axis)
		{
			const float stepsPerMm = m.DriveStepsPerMm(axis);
			const float startMm = startCoords[axis];
			const float endMm = move.coords[axis];
			startSteps[axis] = static_cast<int32_t>(std::lround(startMm * stepsPerMm));
			endSteps[axis] = static_cast<int32_t>(std::lround(endMm * stepsPerMm));
		}
	}

	hasExtrusion = move.hasE;
	if (hasExtrusion)
	{
		const size_t extruderDrive = MaxAxes;
		const float stepsPerMm = m.DriveStepsPerMm(extruderDrive);
		startSteps[extruderDrive] = static_cast<int32_t>(std::lround(startCoords[extruderDrive] * stepsPerMm));
		endSteps[extruderDrive] = static_cast<int32_t>(std::lround(move.coords[extruderDrive] * stepsPerMm));
	}
	else
	{
		const size_t extruderDrive = MaxAxes;
		const float stepsPerMm = m.DriveStepsPerMm(extruderDrive);
		startSteps[extruderDrive] = static_cast<int32_t>(std::lround(startCoords[extruderDrive] * stepsPerMm));
		endSteps[extruderDrive] = startSteps[extruderDrive];
	}

	requestedSpeed = (move.feedRate > 0.0f) ? move.feedRate : 1.0f;

	float totalDistanceSq = 0.0f;
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		if (!axis_in_mask(move, axis))
		{
			continue;
		}

		const float axisDelta = move.coords[axis] - startCoords[axis];
		totalDistanceSq += axisDelta * axisDelta;
	}

	params.totalDistance = sqrtf(totalDistanceSq);
	if (params.totalDistance < 1e-6f && hasExtrusion)
	{
		const float extruderDelta = move.coords[MaxAxes] - startCoords[MaxAxes];
		params.totalDistance = fabsf(extruderDelta);
	}

	if (params.totalDistance < 1e-6f && !hasExtrusion)
	{
		return false;
	}

	isPrintingMove = hasExtrusion && (params.totalDistance > 1e-3f);

	return true;
}

namespace host
{

static double compute_direction(const RawMove& mv,
								const float startMachineCoords[MaxAxesPlusExtruders],
								double unitDir[MaxAxes],
								double axisDelta[MaxAxes],
								size_t numAxes) noexcept
{
	double sumSq = 0.0;
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		double delta = static_cast<double>(mv.coords[axis] - startMachineCoords[axis]);
		if (!axis_in_mask(mv, axis))
		{
			delta = 0.0;
		}
		axisDelta[axis] = delta;
		sumSq += delta * delta;
	}
	const double length = (sumSq > kTiny) ? std::sqrt(sumSq) : 0.0;

	if (length > kTiny)
	{
		const double inv = 1.0 / length;
		for (size_t axis = 0; axis < numAxes; ++axis)
		{
			unitDir[axis] = axisDelta[axis] * inv;
		}
	}
	else
	{
		for (size_t axis = 0; axis < numAxes; ++axis)
		{
		unitDir[axis] = 0.0;
		}
	}
	return length;
}

bool DDAHost::Prepare(const RawMove& mv,
					  const float startMachineCoords[MaxAxesPlusExtruders],
					  const int32_t startSteps[MaxAxesPlusExtruders],
					  const int32_t endSteps[MaxAxesPlusExtruders],
					  double entrySpeed,
					  double topSpeed,
					  double exitSpeed,
					  double accelLimit,
					  PrepOut& out) noexcept
{
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();

	double unitDir[MaxAxes] = {0.0};
	double axisDelta[MaxAxes] = {0.0};
	double length = compute_direction(mv, startMachineCoords, unitDir, axisDelta, numAxes);

	const double extruderDelta = static_cast<double>(mv.coords[MaxAxes] - startMachineCoords[MaxAxes]);
	const bool hasAxes = length > kTiny;
	const bool hasExtrusionOnly = (!hasAxes && std::abs(extruderDelta) > kTiny);
	if (!hasAxes && !hasExtrusionOnly)
	{
		return false;
	}

	const double travelDistance = hasAxes ? length : std::abs(extruderDelta);

	const double accel = std::max(accelLimit, 1.0);
	const double vEntry = std::max(0.0, entrySpeed);
	const double vExit = std::max(0.0, exitSpeed);

	double vPeak = std::max({topSpeed, vEntry, vExit});

	// Calculate acceleration/deceleration distances assuming trapezoid
	double accelDistance = 0.0;
	if (vPeak > vEntry)
	{
		accelDistance = (vPeak * vPeak - vEntry * vEntry) / (2.0 * accel);
	}
	double decelDistance = 0.0;
	if (vPeak > vExit)
	{
		decelDistance = (vPeak * vPeak - vExit * vExit) / (2.0 * accel);
	}
	double steadyDistance = travelDistance - accelDistance - decelDistance;

	if (steadyDistance < -1e-6)
	{
		// Triangular profile: recompute peak velocity
		const double term = std::max(0.0, accel * travelDistance + 0.5 * (vEntry * vEntry + vExit * vExit));
		vPeak = std::sqrt(term);
		if (vPeak < vEntry)
		{
			vPeak = vEntry;
		}
		if (vPeak < vExit)
		{
			vPeak = vExit;
		}

		if (vPeak > vEntry)
		{
			accelDistance = (vPeak * vPeak - vEntry * vEntry) / (2.0 * accel);
		}
		else
		{
			accelDistance = 0.0;
		}

		if (vPeak > vExit)
		{
			decelDistance = (vPeak * vPeak - vExit * vExit) / (2.0 * accel);
		}
		else
		{
			decelDistance = 0.0;
		}
		steadyDistance = 0.0;
	}

	steadyDistance = std::max(0.0, steadyDistance);

	const double tAcc = (vPeak > vEntry) ? (vPeak - vEntry) / accel : 0.0;
	const double tDec = (vPeak > vExit) ? (vPeak - vExit) / accel : 0.0;
	const double tSteady = (steadyDistance > kTiny && vPeak > kTiny) ? steadyDistance / vPeak : 0.0;

	double totalTime = tAcc + tSteady + tDec;
	if (totalTime <= kTiny)
	{
		totalTime = std::max(travelDistance / std::max(vPeak, 1e-6), 0.0);
		if (totalTime <= kTiny)
		{
			return false;
		}
	}

	const double ticksPerSec = static_cast<double>(host::StepTimerHost::TICKS_PER_SEC);
	const uint32_t accelClocks = static_cast<uint32_t>(std::llround(tAcc * ticksPerSec));
	const uint32_t steadyClocks = static_cast<uint32_t>(std::llround(tSteady * ticksPerSec));
	const uint32_t decelClocks = static_cast<uint32_t>(std::llround(tDec * ticksPerSec));

	out.startClock = 0;
	out.accelClocks = accelClocks;
	out.steadyClocks = steadyClocks;
	out.decelClocks = decelClocks;
	out.vEntry = vEntry;
	out.vTop = vPeak;
	out.vExit = vExit;
	out.acceleration = accel;
	out.deceleration = -accel;
	out.accelDistance = accelDistance;
	out.decelDistanceStart = std::max(0.0, travelDistance - decelDistance);
	out.totalDistance = travelDistance;

	out.numDrives = 0;
	for (int drive = 0; drive < kMaxDrives; ++drive)
	{
		const int64_t deltaSteps = static_cast<int64_t>(endSteps[drive]) - static_cast<int64_t>(startSteps[drive]);
		if (deltaSteps == 0)
		{
			continue;
		}

		if (out.numDrives >= kMaxDrives)
		{
			break;
		}

		DrivePlan& dp = out.drives[out.numDrives++];
		dp.id = static_cast<uint8_t>(drive);
		dp.steps = deltaSteps;
		dp.stepsPerSecTop = (totalTime > kTiny) ? std::abs(static_cast<double>(deltaSteps)) / totalTime : 0.0;
	}

	return true;
}

} // namespace host

bool DDA::Prepare() noexcept
{
	CanMotion::StartMovement();

	host::DDAHost::PrepOut prepOut{};
	if (!host::DDAHost::Prepare(rawMove,
								startMachineCoords,
								startSteps,
								endSteps,
								static_cast<double>(plannedEntrySpeed),
								static_cast<double>(plannedTopSpeed),
								static_cast<double>(plannedExitSpeed),
								static_cast<double>(plannedAcceleration),
								prepOut))
	{
		return false;
	}

	params.accelClocks = prepOut.accelClocks;
	params.steadyClocks = prepOut.steadyClocks;
	params.decelClocks = prepOut.decelClocks;
	params.acceleration = static_cast<float>(prepOut.acceleration);
	params.deceleration = static_cast<float>(prepOut.deceleration);
	params.accelDistance = static_cast<float>(prepOut.accelDistance);
	params.decelStartDistance = static_cast<float>(prepOut.decelDistanceStart);
	params.totalDistance = static_cast<float>(prepOut.totalDistance);
	params.topSpeed = static_cast<float>(prepOut.vTop);
	params.useInputShaping = false;

	clocksNeeded = params.TotalClocks();

	Move& move = reprap.GetMove();
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numAxes && axis < MaxAxes; ++axis)
	{
		const int32_t steps = endSteps[axis] - startSteps[axis];
		if (steps != 0)
		{
			DriverId driver = move.GetAxisDriverId(axis);
			CanMotion::AddAxisMovement(params, driver, steps);
		}
	}

	if (hasExtrusion)
	{
		const size_t extruderDrive = MaxAxes;
		const int32_t steps = endSteps[extruderDrive] - startSteps[extruderDrive];
		if (steps != 0)
		{
			DriverId driver;
			driver.SetLocal(0);
			const float extrusion = static_cast<float>(steps) / move.DriveStepsPerMm(extruderDrive);
			CanMotion::AddExtruderMovement(params, driver, extrusion, false);
		}
	}

	return true;
}

#endif // RRF_HOST_BUILD
