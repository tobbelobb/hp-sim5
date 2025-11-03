#pragma once

#ifdef RRF_HOST_BUILD

#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>
#include <RepRapFirmware.h>

NamedEnum(SpindleState, uint8_t, unconfigured, stopped, forward, reverse);

class Spindle INHERIT_OBJECT_MODEL
{
private:
    uint32_t currentRpm = 0;  // Just made-up values
    uint32_t configuredRpm = 100;
    uint32_t minRpm = 0;
    uint32_t maxRpm = 200;

public:
    // Constructor
    Spindle() noexcept
    {
    }

    // --- Methods called by Tool.cpp ---

    // The original method we stubbed
    SpindleState GetState() const noexcept
    {
        return SpindleState::unconfigured;
    }

    // New empty stubs to fix the compile errors.
    // We don't need these to do anything in our simulation.
    void SetState(SpindleState newState) noexcept
    {
    }
    void SetConfiguredRpm(uint32_t rpm, bool updateCurrentRpm) noexcept
    {
    }

    // For IsValidRpm, returning 'true' is the safest option. It prevents
    // the calling code from thinking the RPM is invalid and throwing an error.
    bool IsValidRpm(uint32_t rpm) const noexcept
    {
        return true;
    }

    // --- Methods called by Gcodes.cpp ---
    uint32_t GetRpm() const noexcept
    {
        return configuredRpm;
    }

protected:
    DECLARE_OBJECT_MODEL
};

#endif
