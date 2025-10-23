#pragma once

// Host stub for Endstops/EndstopDefs.h
// This prevents redefinition errors from the real RRF/ReprapFirmware/src/Endstops/EndstopDefs.h

#ifndef ENDSTOPS_ENDSTOPS_DEFS_H_
#define ENDSTOPS_ENDSTOPS_DEFS_H_

#include <RepRapFirmware.h>

enum class EndstopHitAction : uint8_t
{
	none = 0,
	stopAll,
	stopAxis,
	stopDriver
};

struct EndstopHitDetails
{
	EndstopHitAction action{EndstopHitAction::none};
	bool isZProbe{false};
	size_t axis{0};
	DriverId driver{};

	EndstopHitAction GetAction() const noexcept { return action; }
	float GetZProbeHeightError() const noexcept { return 0.0f; }
	bool HasDriver() const noexcept { return false; }
};

#endif // ENDSTOPS_ENDSTOPS_DEFS_H_
