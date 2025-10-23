// Host-side Move implementation for Step 9.1
// Provides minimal Move facade to support CanMotion packet generation

#ifdef RRF_HOST_BUILD

#include <Movement/Move.h>
#include <Movement/DDARing.h>
#include <Movement/Kinematics/Kinematics.h>
#include <General/StringRef.h>
#include <cstring>

Move::Move() noexcept
	: kinematics(new Kinematics())
	, mainRing(new DDARing())
{
	// Initialize steps per mm to reasonable defaults (1mm = 80 steps is common)
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		driveStepsPerMm[i] = 80.0f;
		motorPositions[i] = 0;
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		machinePosition[i] = 0.0f;
	}
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

#endif // RRF_HOST_BUILD
