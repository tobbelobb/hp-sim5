#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/Bitmap.h>
#include <Endstops/EndstopDefs.h>

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

	// Step 9.3: Additional stub for Kinematics
	bool HomingZWithProbe() const noexcept { return false; }
};

#endif
