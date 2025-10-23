#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <cstdint>
#include <cstddef>

// Forward declarations
class DDARing;

// Minimal PrepParams struct for host build
// This matches the structure in the real DDA.h but with simplified fields
struct PrepParams
{
#if SUPPORT_S_CURVE
	uint32_t phaseClocks[7];						// the number of step clocks for each phase
	float initialAcceleration, peakAcceleration;	// the accelerations, always positive
	float initialDeceleration, peakDeceleration;	// the decelerations, always negative
	float distances[7];								// the distances of each phase
	float jerk;										// the magnitude of the rate of change of acceleration or deceleration, always positive
	uint32_t SteadyClocks() const noexcept { return phaseClocks[3]; }
	uint32_t TotalAccelClocks() const noexcept { return phaseClocks[0] + phaseClocks[1] + phaseClocks[2]; }
	uint32_t TotalDecelClocks() const noexcept { return phaseClocks[4] + phaseClocks[5] + phaseClocks[6]; }
#else
	uint32_t accelClocks, steadyClocks, decelClocks;
	float acceleration;								// the acceleration to use, always positive
	float deceleration;								// the deceleration to use, always negative
	float accelDistance;
	float decelStartDistance;
	uint32_t SteadyClocks() const noexcept { return steadyClocks; }
	uint32_t TotalAccelClocks() const noexcept { return accelClocks; }
	uint32_t TotalDecelClocks() const noexcept { return decelClocks; }
#endif
	float totalDistance;
	float topSpeed;									// the top speed reached
	bool useInputShaping;

	// Get the total clocks needed
	uint32_t TotalClocks() const noexcept { return TotalAccelClocks() + SteadyClocks() + TotalDecelClocks(); }
};

// Minimal DDA class for host build
// This provides just enough interface for CanMotion to call IsCheckingEndstops()
// and other minimal queries during FinishMovement().
class DDA
{
public:
	DDA() noexcept;

	// Query methods used by CanMotion
	bool IsCheckingEndstops() const noexcept { return checkEndstops; }
	bool IsPrintingMove() const noexcept { return isPrintingMove; }

	uint32_t GetMoveStartTime() const noexcept { return moveStartTime; }
	uint32_t GetClocksNeeded() const noexcept { return clocksNeeded; }

	// For future use when we integrate real movement planning
	void SetCheckEndstops(bool check) noexcept { checkEndstops = check; }
	void SetIsPrintingMove(bool printing) noexcept { isPrintingMove = printing; }
	void SetMoveStartTime(uint32_t time) noexcept { moveStartTime = time; }
	void SetClocksNeeded(uint32_t clocks) noexcept { clocksNeeded = clocks; }

private:
	bool checkEndstops;
	bool isPrintingMove;
	uint32_t moveStartTime;
	uint32_t clocksNeeded;
};

#endif // RRF_HOST_BUILD
