#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <Movement/MovementError.h>
#include <Movement/Kinematics/Kinematics.h>
#include <cstdint>
#include <cstddef>

// Forward declarations
class DDA;
struct PrepParams;
class DDARing;
class GCodeBuffer;
class StringRef;

// Step 9.3: AxisDriversConfig stub for Kinematics
struct AxisDriversConfig
{
	DriverId driverNumbers[MaxDriversPerAxis];
	size_t numDrivers;

	AxisDriversConfig() noexcept : numDrivers(1)
	{
		for (size_t i = 0; i < MaxDriversPerAxis; ++i)
		{
			driverNumbers[i].SetLocal(0);
		}
	}
};

// Minimal host-side Move facade for Step 9.1
// This provides just enough interface for CanMotion.cpp to compile and generate CAN packets
// without dragging in the full Movement subsystem.
//
// Step 9.2.2: Extended with configuration storage for M92, M201, M203, M566, M584, M569

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

	// Step 9.2.2: Configuration accessors
	float GetAcceleration(size_t drive) const noexcept;
	void SetAcceleration(size_t drive, float value) noexcept;

	float GetMaxFeedrate(size_t drive) const noexcept;
	void SetMaxFeedrate(size_t drive, float value) noexcept;

	float GetJerk(size_t drive) const noexcept;
	void SetJerk(size_t drive, float value) noexcept;

	DriverId GetAxisDriverId(size_t axis) const noexcept;
	void SetAxisDriverId(size_t axis, const DriverId& driver) noexcept;

	bool GetDriverDirection(const DriverId& driver) const noexcept;
	void SetDriverDirection(const DriverId& driver, bool forward) noexcept;

	// Step 9.3: Additional accessors needed by real Kinematics
	float AxisMinimum(size_t axis) const noexcept;
	float AxisMaximum(size_t axis) const noexcept;
	float MaxFeedrate(size_t drive) const noexcept { return GetMaxFeedrate(drive); }  // Alias
	float NormalAcceleration(size_t drive) const noexcept { return GetAcceleration(drive); }  // Alias
	const AxisDriversConfig& GetAxisDriversConfig(size_t axis) const noexcept;
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) const noexcept;

	// Step 9.3.2: Coordinate transform via real Kinematics
	MovementError CartesianToMotorSteps(const float machinePos[], int32_t motorPos[], bool isCoordinated) const noexcept;
	bool IsAxisRotational(size_t axis) const noexcept;
	void SetKinematics(KinematicsType type) noexcept;

private:
	Kinematics* kinematics;
	DDARing* mainRing;
	float driveStepsPerMm[MaxAxesPlusExtruders];
	int32_t motorPositions[MaxAxesPlusExtruders];
	float machinePosition[MaxAxes];

	// Step 9.2.2: Configuration storage
	float accelerations[MaxAxesPlusExtruders];		// Max acceleration per axis/extruder (mm/s²)
	float maxFeedrates[MaxAxesPlusExtruders];		// Max speed per axis/extruder (mm/s)
	float jerks[MaxAxesPlusExtruders];				// Jerk/instant speed change (mm/min)
	DriverId axisDrivers[MaxAxes];					// Axis to driver mapping (for M584)
	DriverId extruderDrivers[MaxExtrudersPerTool];	// Extruder to driver mapping
	bool driverForward[256];						// Driver direction (indexed by board*16 + driver)

	// Step 9.3: Axis limits for kinematics
	float axisMinima[MaxAxes];						// Minimum position for each axis (mm)
	float axisMaxima[MaxAxes];						// Maximum position for each axis (mm)
	AxisDriversConfig axisDriversConfigs[MaxAxes];	// Driver configuration for each axis
};

#endif // RRF_HOST_BUILD
