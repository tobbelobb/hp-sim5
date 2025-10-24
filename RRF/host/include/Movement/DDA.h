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

enum RawMoveFlags : uint32_t
{
	RMF_None = 0u,
	RMF_RawMotorMove = 1u << 0,
	RMF_Rapid = 1u << 1,
	RMF_IgnoreLimits = 1u << 2
};

// Simple RawMove structure for G1/G0 commands
struct RawMove
{
	float coords[MaxAxesPlusExtruders];		// Target coordinates for each axis/extruder
	float feedRate;							// Requested feedrate in mm/sec
	uint32_t flags;						// RawMoveFlags bitfield
	uint32_t independentMask;			// Bitmask of axes to treat as independent (raw motor moves)
	bool hasE;								// Does this move include extrusion?

	RawMove() noexcept
		: feedRate(0.0f)
		, flags(RMF_None)
		, independentMask(0)
		, hasE(false)
	{
		for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
		{
			coords[i] = 0.0f;
		}
	}
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

	// Step 9.2.1: Movement planning methods
	bool Init(const RawMove& move, float startCoords[MaxAxesPlusExtruders]) noexcept;
	bool Prepare() noexcept;

	PrepParams& GetPrepParams() noexcept { return params; }

	// Step 9.3: Additional methods needed by real Kinematics
	void LimitSpeedAndAcceleration(float /*maxSpeed*/, float /*maxAccel*/) noexcept {}  // Stub for now

private:
	bool checkEndstops;
	bool isPrintingMove;
	uint32_t moveStartTime;
	uint32_t clocksNeeded;

	// Step 9.2.1: Movement data
	PrepParams params;
	int32_t endSteps[MaxAxesPlusExtruders];	// Target position in steps
	int32_t startSteps[MaxAxesPlusExtruders];	// Starting position in steps
	float requestedSpeed;						// Requested speed in mm/sec
	bool hasExtrusion;							// Does this move include extruder movement?
	float startMachineCoords[MaxAxesPlusExtruders]; // machine-space start position (mm)
	RawMove rawMove;								// cached move definition for Prepare phase
};

#endif // RRF_HOST_BUILD
