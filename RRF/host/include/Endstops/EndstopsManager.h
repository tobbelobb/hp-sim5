#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/Bitmap.h>

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

class EndstopsManager
{
public:
	void Init() noexcept {}
	void ClearEndstops() noexcept {}

	void EnableAxisEndstops(AxesBitmap, const float*, bool, bool&) noexcept {}
	bool EnableZProbe(size_t, bool = false) noexcept { return false; }
	void EnableExtruderEndstops(ExtrudersBitmap, const float*, bool&) noexcept {}

	EndstopHitDetails CheckEndstops() noexcept { return EndstopHitDetails{}; }
	bool AnyEndstopsActive() const noexcept { return false; }

	void DisableRemoteStallEndstops() noexcept {}
};

#endif
