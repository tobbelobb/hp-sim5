#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdint>

#include <General/StringRef.h>
#include <GCodeResult.h>
#include <GCodes/GCodeChannel.h>
#include <GCodes/GCodeMachineState.h>

class GCodeBuffer;

enum class MachineType : uint8_t
{
	fff,
	cnc,
	laser
};

// Minimal host stub for the GCodes manager.  It exposes just enough surface
// for the current G-code buffer stack to compile and issue replies.
class GCodes
{
public:
	GCodes() noexcept = default;

	MachineType GetMachineType() const noexcept { return machineType; }
	const char* GetAxisLetters() const noexcept { return axisLetters; }

	void AbortPrint(GCodeBuffer&) noexcept {}

	void HandleReply(GCodeBuffer&, GCodeResult, const char*) noexcept {}

private:
	MachineType machineType{MachineType::fff};
	const char* axisLetters{"XYZABC"};
};

#endif
