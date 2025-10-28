// Host-side DDARing implementation for Step 9.1

#ifdef RRF_HOST_BUILD

#include <Movement/DDARing.h>
#include <cstring>

DDARing::DDARing() noexcept
	: idle(true)
	, scheduledMoves(0)
	, completedMoves(0)
	, simulationTime(0.0f)
	, topSpeedMmPerSec(0.0f)
	, requestedSpeedMmPerSec(0.0f)
	, accelerationMmPerSecSquared(0.0f)
	, decelerationMmPerSecSquared(0.0f)
	, currentMoveDistance(0.0f)
	, currentMoveDuration(0.0f)
	, totalExtrusionRate(0.0f)
{
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		machinePosition[i] = 0.0f;
	}
}

void DDARing::GetCurrentMachinePosition(float m[MaxAxes]) const noexcept
{
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		m[i] = machinePosition[i];
	}
}

void DDARing::UpdateStartCoordinates(const float *coords) noexcept
{
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		machinePosition[i] = coords[i];
	}
}

#endif // RRF_HOST_BUILD
