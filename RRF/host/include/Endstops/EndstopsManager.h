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

	bool HomingZWithProbe() const noexcept { return false; }

	void GetM119report(const StringRef& reply) noexcept {	reply.copy("No Endstops or zprobes on Host"); };
	// G31: Set or Report Current Probe status
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) { return GCodeResult::ok; };

	void SetZProbeDefaults() noexcept { };
	ReadLockedPointer<ZProbe> GetZProbe(size_t index) const noexcept { return ReadLockedPointer<ZProbe>(nullptr, nullptr); } ;
};

#endif
