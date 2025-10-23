#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <cstdint>
#include <cstddef>

// Forward declarations
class DDA;
class Kinematics;
struct PrepParams;
class DDARing;
class GCodeBuffer;
class StringRef;

// Minimal host-side Move facade for Step 9.1
// This provides just enough interface for CanMotion.cpp to compile and generate CAN packets
// without dragging in the full Movement subsystem.
//
// Future work (Step 9.2): Replace this with a more complete Move implementation that
// actually plans moves using RRF's DDA/DriveMovement classes.

class Move
{
public:
	Move() noexcept;
	~Move() noexcept;

	void Init() noexcept;
	void Exit() noexcept;

	// Minimal kinematics support - just return a stub for now
	Kinematics& GetKinematics() const noexcept;

	// Steps per mm configuration (needed for motor step calculations)
	float DriveStepsPerMm(size_t axisOrExtruder) const noexcept;
	float SetDriveStepsPerMm(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping) noexcept;

	// Current position tracking (needed for relative moves)
	void GetCurrentMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber) const noexcept;
	void SetMotorPosition(size_t drive, int32_t pos, bool clearBacklash) noexcept;

	// Driver configuration stubs (CanMotion may query these)
	size_t GetNumActualDirectDrivers() const noexcept { return NumDirectDrivers; }

	// Main DDA ring accessor
	DDARing& GetMainDDARing() noexcept;

	// Diagnostics
	void Diagnostics(unsigned int part, const StringRef& reply) noexcept;

private:
	Kinematics* kinematics;
	DDARing* mainRing;
	float driveStepsPerMm[MaxAxesPlusExtruders];
	int32_t motorPositions[MaxAxesPlusExtruders];
	float machinePosition[MaxAxes];
};

#endif // RRF_HOST_BUILD
