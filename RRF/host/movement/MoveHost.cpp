// Host-side Move implementation for Step 9.1
// Provides minimal Move facade to support CanMotion packet generation

#ifdef RRF_HOST_BUILD

#include <Movement/Move.h>
#include <Movement/DDARing.h>
#include <Movement/Kinematics/CartesianKinematicsHost.h>
#include <Movement/Kinematics/Kinematics.h>
#include <Movement/Kinematics/HangprinterKinematics.h>
#include <General/StringRef.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>
#include <cstring>

Move::Move() noexcept
	: kinematics(nullptr)  // Will be set by SetKinematics()
	, mainRing(new DDARing())
{
	// Initialize steps per mm to reasonable defaults (1mm = 80 steps is common)
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		driveStepsPerMm[i] = 80.0f;
		motorPositions[i] = 0;
		accelerations[i] = 1000.0f;		// Default 1000 mm/s²
		maxFeedrates[i] = 100.0f;		// Default 100 mm/s
		jerks[i] = 240.0f;				// Default 240 mm/min (4 mm/s instant change)
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		machinePosition[i] = 0.0f;
		axisDrivers[i].SetLocal(0);		// Default to local driver 0 (will be overridden by M584)
	}

	for (size_t i = 0; i < MaxExtrudersPerTool; ++i)
	{
		extruderDrivers[i].SetLocal(0);	// Default to local driver 0
	}

	// Initialize all driver directions to forward
	for (size_t i = 0; i < 256; ++i)
	{
		driverForward[i] = true;
	}

	// Step 9.3: Initialize axis limits to reasonable defaults
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		axisMinima[i] = 0.0f;		// Default minimum 0mm
		axisMaxima[i] = 200.0f;		// Default maximum 200mm (will be set by kinematics or M208)
	}

	// Default to Cartesian kinematics on host startup
	SetKinematics(KinematicsType::cartesian);
}

Move::~Move() noexcept
{
	delete mainRing;
	delete kinematics;
}

void Move::Init() noexcept
{
	// Nothing to initialize yet in Step 9.1
}

void Move::Exit() noexcept
{
	// Nothing to clean up yet in Step 9.1
}

Kinematics& Move::GetKinematics() const noexcept
{
	return *kinematics;
}

float Move::DriveStepsPerMm(size_t axisOrExtruder) const noexcept
{
	return (axisOrExtruder < MaxAxesPlusExtruders) ? driveStepsPerMm[axisOrExtruder] : 80.0f;
}

float Move::SetDriveStepsPerMm(size_t axisOrExtruder, float value, uint32_t /*requestedMicrostepping*/) noexcept
{
	if (axisOrExtruder < MaxAxesPlusExtruders)
	{
		driveStepsPerMm[axisOrExtruder] = value;
		return value;
	}
	return 80.0f;
}

void Move::GetCurrentMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber) const noexcept
{
	if (msNumber == 0)
	{
		mainRing->GetCurrentMachinePosition(m);
	}
	else
	{
		// For other movement systems, just return zeros
		for (size_t i = 0; i < MaxAxes; ++i)
		{
			m[i] = 0.0f;
		}
	}
}

void Move::SetMotorPosition(size_t drive, int32_t pos, bool /*clearBacklash*/) noexcept
{
	if (drive < MaxAxesPlusExtruders)
	{
		motorPositions[drive] = pos;
	}
}

DDARing& Move::GetMainDDARing() noexcept
{
	return *mainRing;
}

void Move::Diagnostics(unsigned int /*part*/, const StringRef& reply) noexcept
{
	reply.copy("Move diagnostics not yet implemented in host build\n");
}

// Step 9.2.2: Configuration accessors
float Move::GetAcceleration(size_t drive) const noexcept
{
	return (drive < MaxAxesPlusExtruders) ? accelerations[drive] : 1000.0f;
}

void Move::SetAcceleration(size_t drive, float value) noexcept
{
	if (drive < MaxAxesPlusExtruders)
	{
		accelerations[drive] = value;
	}
}

float Move::GetMaxFeedrate(size_t drive) const noexcept
{
	return (drive < MaxAxesPlusExtruders) ? maxFeedrates[drive] : 100.0f;
}

void Move::SetMaxFeedrate(size_t drive, float value) noexcept
{
	if (drive < MaxAxesPlusExtruders)
	{
		maxFeedrates[drive] = value;
	}
}

float Move::GetJerk(size_t drive) const noexcept
{
	return (drive < MaxAxesPlusExtruders) ? jerks[drive] : 240.0f;
}

void Move::SetJerk(size_t drive, float value) noexcept
{
	if (drive < MaxAxesPlusExtruders)
	{
		jerks[drive] = value;
	}
}

float Move::GetMaxInstantDv(size_t drive) const noexcept
{
	constexpr float fallback = 240.0f / 60.0f;	// default 4 mm/s instant dv
	if (drive < MaxAxesPlusExtruders)
	{
		const float jerkMmPerMin = jerks[drive];
		if (jerkMmPerMin > 0.0f)
		{
			return jerkMmPerMin / 60.0f;
		}
	}
	return fallback;
}

DriverId Move::GetAxisDriverId(size_t axis) const noexcept
{
	if (axis < MaxAxes)
	{
		return axisDrivers[axis];
	}
	DriverId nullDriver;
	nullDriver.SetLocal(0);
	return nullDriver;
}

void Move::SetAxisDriverId(size_t axis, const DriverId& driver) noexcept
{
	if (axis < MaxAxes)
	{
		axisDrivers[axis] = driver;
		// Step 9.3: Also update the AxisDriversConfig
		axisDriversConfigs[axis].driverNumbers[0] = driver;
		axisDriversConfigs[axis].numDrivers = 1;
	}
}

bool Move::GetDriverDirection(const DriverId& driver) const noexcept
{
	// Hash DriverId to index: (board * 16 + local)
	const uint8_t board = driver.boardAddress;
	const uint8_t local = driver.localDriver;
	const size_t index = (static_cast<size_t>(board) * 16) + local;
	return (index < 256) ? driverForward[index] : true;
}

void Move::SetDriverDirection(const DriverId& driver, bool forward) noexcept
{
	const uint8_t board = driver.boardAddress;
	const uint8_t local = driver.localDriver;
	const size_t index = (static_cast<size_t>(board) * 16) + local;
	if (index < 256)
	{
		driverForward[index] = forward;
	}
}

// Step 9.3: Axis limit accessors for real Kinematics
float Move::AxisMinimum(size_t axis) const noexcept
{
	return (axis < MaxAxes) ? axisMinima[axis] : 0.0f;
}

float Move::AxisMaximum(size_t axis) const noexcept
{
	return (axis < MaxAxes) ? axisMaxima[axis] : 200.0f;
}

const AxisDriversConfig& Move::GetAxisDriversConfig(size_t axis) const noexcept
{
	static AxisDriversConfig defaultConfig;
	return (axis < MaxAxes) ? axisDriversConfigs[axis] : defaultConfig;
}

unsigned int Move::GetMicrostepping(size_t /*drive*/, bool& interpolation) const noexcept
{
	interpolation = false;
	return 16;  // Default 16x microstepping for CAN-only builds
}

// Step 9.3.2: Call real Kinematics::CartesianToMotorSteps()
MovementError Move::CartesianToMotorSteps(const float machinePos[], int32_t motorPos[], bool isCoordinated) const noexcept
{
	// Get the configured number of axes
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();

	// Call the real kinematics transform
	return kinematics->CartesianToMotorSteps(machinePos, driveStepsPerMm, numVisibleAxes, numTotalAxes, motorPos, isCoordinated);
}

bool Move::IsAxisRotational(size_t /*axis*/) const noexcept
{
	// For now, treat all axes as linear (non-rotational)
	// TODO: Add rotational axis support if needed (e.g., for A, B, C axes)
	return false;
}

// Step 9.3.2: Change kinematics type (used by M669)
void Move::SetKinematics(KinematicsType type) noexcept
{
	// Delete old kinematics if it exists
	if (kinematics != nullptr)
	{
		delete kinematics;
		kinematics = nullptr;
	}

	// Create new kinematics based on type
	if (type == KinematicsType::hangprinter)
	{
		kinematics = new HangprinterKinematics();
	}
	else
	{
		kinematics = new CartesianKinematicsHost();
	}
}

void Move::ConfigureSegmentation(float segmentsPerSecond, float minSegmentLength) noexcept
{
	if (kinematics == nullptr)
	{
		return;
	}

	kinematics->ConfigureSegmentationParameters(segmentsPerSecond, minSegmentLength);
}

#endif // RRF_HOST_BUILD
