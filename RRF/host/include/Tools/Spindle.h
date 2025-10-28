#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>

// This enum definition is still correct
NamedEnum(SpindleState, uint8_t, unconfigured, stopped, forward, reverse);

// A more complete fake Spindle class to satisfy Tool.cpp
class Spindle INHERIT_OBJECT_MODEL
{
public:
	// Constructor
	Spindle() noexcept {}

	// --- Methods called by Tool.cpp ---

	// The original method we stubbed
	SpindleState GetState() const noexcept { return SpindleState::unconfigured; }

	// New empty stubs to fix the compile errors.
	// We don't need these to do anything in our simulation.
	void SetState(SpindleState newState) noexcept { }
	void SetConfiguredRpm(uint32_t rpm, bool updateCurrentRpm) noexcept { }

	// For IsValidRpm, returning 'true' is the safest option. It prevents
	// the calling code from thinking the RPM is invalid and throwing an error.
	bool IsValidRpm(uint32_t rpm) const noexcept { return true; }

protected:
	DECLARE_OBJECT_MODEL
};

#endif
