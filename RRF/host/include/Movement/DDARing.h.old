#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <Movement/DDA.h>
#include <cstdint>

// Minimal DDARing for host build
// The DDARing manages a circular buffer of DDAs (deferred move commands).
// For Step 9.1, we only need a placeholder that can be queried.
class DDARing
{
public:
	DDARing() noexcept;

	// Query methods
	bool IsIdle() const noexcept { return idle; }
	void GetCurrentMachinePosition(float m[MaxAxes]) const noexcept;

	// Movement tracking
	uint32_t GetScheduledMoves() const noexcept { return scheduledMoves; }
	uint32_t GetCompletedMoves() const noexcept { return completedMoves; }
	void ResetMoveCounters() noexcept { scheduledMoves = 0; completedMoves = 0; }

	// Simulation support
	float GetSimulationTime() const noexcept { return simulationTime; }

	// Reporting
	float GetTopSpeedMmPerSec() const noexcept { return topSpeedMmPerSec; }
	float GetRequestedSpeedMmPerSec() const noexcept { return requestedSpeedMmPerSec; }
	float GetAccelerationMmPerSecSquared() const noexcept { return accelerationMmPerSecSquared; }
	float GetDecelerationMmPerSecSquared() const noexcept { return decelerationMmPerSecSquared; }
	float GetCurrentMoveDistance() const noexcept { return currentMoveDistance; }
	float GetCurrentMoveDuration() const noexcept { return currentMoveDuration; }
	float GetTotalExtrusionRate() const noexcept { return totalExtrusionRate; }

	void UpdateStartCoordinates(const float *coords) noexcept;

private:
	bool idle;
	uint32_t scheduledMoves;
	uint32_t completedMoves;
	float simulationTime;
	float machinePosition[MaxAxes];

	// Reporting fields
	float topSpeedMmPerSec;
	float requestedSpeedMmPerSec;
	float accelerationMmPerSecSquared;
	float decelerationMmPerSecSquared;
	float currentMoveDistance;
	float currentMoveDuration;
	float totalExtrusionRate;
};

#endif // RRF_HOST_BUILD
