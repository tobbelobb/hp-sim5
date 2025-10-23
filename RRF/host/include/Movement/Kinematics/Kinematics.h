#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <cstdint>

class StringRef;

// Minimal Kinematics stub for host build
// For Step 9.1, we just need a placeholder that can be queried for basic info.
// Real kinematics (Cartesian, Hangprinter, etc.) will be integrated in Step 9.2.
class Kinematics
{
public:
	Kinematics() noexcept;
	virtual ~Kinematics() noexcept;

	// Basic queries
	virtual const char* GetName() const noexcept { return "none"; }
	virtual bool IsCartesian() const noexcept { return false; }

	// For future integration
	virtual void Diagnostics(unsigned int part, const StringRef& reply) noexcept;
};

#endif // RRF_HOST_BUILD
