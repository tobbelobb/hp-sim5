#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/Bitmap.h>
#include <Endstops/EndstopDefs.h>
#include <RTOSIface/RTOSIface.h>

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

	void GetM119report(const StringRef& reply) noexcept {	reply.copy("No Endstops or zprobes on Host"); }
	// M558: Create or modify probe
	GCodeResult HandleM558(GCodeBuffer& gb, const StringRef &reply) { return GCodeResult::ok; }
	// G31: Set or Report Current Probe status
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) { return GCodeResult::ok; }
	// Configure the endstops in response to M574
	GCodeResult HandleM574(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & outbuf)  { return GCodeResult::ok; }

	bool Stopped(size_t axis) const noexcept { return true; }

	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply)  { return GCodeResult::ok; }

	void SetZProbeDefaults() noexcept { }
	ReadLockedPointer<ZProbe> GetZProbe(size_t index) const noexcept { return ReadLockedPointer<ZProbe>(nullptr, nullptr); }
};

#endif
