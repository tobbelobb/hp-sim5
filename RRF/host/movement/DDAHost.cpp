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
					  PrepOut& out) noexcept
{
	Move& move = reprap.GetMove();
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();

	double unitDir[MaxAxes] = {0.0};
	double axisDelta[MaxAxes] = {0.0};
	double length = compute_direction(mv, startMachineCoords, unitDir, axisDelta, numAxes);

	const double extruderDelta = static_cast<double>(mv.coords[MaxAxes] - startMachineCoords[MaxAxes]);
	if (length < kTiny && std::abs(extruderDelta) > kTiny)
	{
		length = std::abs(extruderDelta);
	}

	if (length < kTiny)
	{
		return false;
	}

	double requested = static_cast<double>(mv.feedRate);
	if (requested <= 0.0)
	{
		requested = static_cast<double>(move.GetMaxFeedrate(0));
	}

	double accelLimit = std::numeric_limits<double>::max();
	double speedLimit = std::numeric_limits<double>::max();

	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		const double delta = std::abs(axisDelta[axis]);
		if (delta < kTiny)
		{
			continue;
		}

		const double axisAccel = static_cast<double>(move.GetAcceleration(axis));
		const double axisSpeed = static_cast<double>(move.GetMaxFeedrate(axis));
		const double ui = std::abs(unitDir[axis]);

		const double accelCap = (ui > kTiny) ? axisAccel / ui : axisAccel;
		const double speedCap = (ui > kTiny) ? axisSpeed / ui : axisSpeed;

		accelLimit = std::min(accelLimit, accelCap);
		speedLimit = std::min(speedLimit, speedCap);
	}

	if (accelLimit == std::numeric_limits<double>::max())
	{
		accelLimit = static_cast<double>(move.GetAcceleration(0));
	}
	accelLimit = std::max(accelLimit, 1.0);

	double targetSpeed = std::min(requested, speedLimit);
	if (!std::isfinite(targetSpeed) || targetSpeed <= 0.0)
	{
		targetSpeed = requested;
	}
	targetSpeed = std::max(targetSpeed, 1e-3);

	const double t_to_target = targetSpeed / accelLimit;
	double accelDistance = 0.5 * accelLimit * t_to_target * t_to_target;

	bool triangle = (2.0 * accelDistance) >= length;

	double t_acc, t_dec, t_steady, v_top;
	if (triangle)
	{
		t_acc = std::sqrt(length / accelLimit);
		t_dec = t_acc;
		t_steady = 0.0;
		v_top = accelLimit * t_acc;
		accelDistance = 0.5 * accelLimit * t_acc * t_acc;
	}
	else
	{
		t_acc = t_to_target;
		t_dec = t_to_target;
		t_steady = (length - 2.0 * accelDistance) / targetSpeed;
		v_top = targetSpeed;
	}

	const double decelDistance = 0.5 * accelLimit * t_dec * t_dec;

	const double totalTime = t_acc + t_steady + t_dec;
	if (totalTime <= kTiny)
	{
		return false;
	}

	const double ticksPerSec = static_cast<double>(host::StepTimerHost::TICKS_PER_SEC);
	const uint32_t accelClocks = static_cast<uint32_t>(std::llround(t_acc * ticksPerSec));
	const uint32_t steadyClocks = static_cast<uint32_t>(std::llround(t_steady * ticksPerSec));
	const uint32_t decelClocks = static_cast<uint32_t>(std::llround(t_dec * ticksPerSec));
	const uint32_t totalClocks = accelClocks + steadyClocks + decelClocks;

	const uint64_t startClock = host::StepTimerHost::align_to_next(host::StepTimerHost::now(), 50);
	host::StepTimerHost::align_to_next(startClock + totalClocks, 0);

	out.startClock = startClock;
	out.accelClocks = accelClocks;
	out.steadyClocks = steadyClocks;
	out.decelClocks = decelClocks;
	out.vEntry = 0.0;
	out.vTop = v_top;
	out.vExit = 0.0;
	out.acceleration = accelLimit;
	out.deceleration = -accelLimit;
	out.accelDistance = accelDistance;
	out.decelDistanceStart = length - decelDistance;
	out.totalDistance = length;

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
		dp.stepsPerSecTop = std::abs(static_cast<double>(deltaSteps)) / totalTime;
	}

	return true;
}

} // namespace host

bool DDA::Prepare() noexcept
{
	CanMotion::StartMovement();

	host::DDAHost::PrepOut prepOut{};
	if (!host::DDAHost::Prepare(rawMove, startMachineCoords, startSteps, endSteps, prepOut))
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
