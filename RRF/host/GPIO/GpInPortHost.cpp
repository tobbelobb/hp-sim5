#include <GPIO/GpInPort.h>
#include <cstdint>

namespace {
	inline void UNUSED_(const void*) {}
}

// Object model note:
// In the host build, the object-model macros typically collapse to no-ops.
// Add any IMPLEMENT_OBJECT_MODEL(...) here if your environment requires it.

bool GpInputPort::GetState() const noexcept
{
	// Host stub: just return the remembered logical level.
	return currentState;
}

bool GpInputPort::IsUnused() const noexcept
{
	// Host stub: treat "unused" as "currently low/not asserted".
	// Adjust to your needs if callers expect a different notion of "unused".
	return !currentState;
}

GCodeResult GpInputPort::Configure(uint32_t gpinNumber,
                                   GCodeBuffer& gb,
                                   const StringRef& reply) THROWS(GCodeException)
{
	UNUSED_(&gpinNumber);
	UNUSED_(&gb);
	UNUSED_(&reply);

	// Host stub: accept any configuration without touching hardware.
	// Leave currentState as-is (default false until someone sets it).
	return GCodeResult::ok;
}

// ----------------------------------------------------------------------------
// Object model plumbing (host build)

constexpr ObjectModelTableEntry GpInputPort::objectModelTable[] = { };
constexpr uint8_t GpInputPort::objectModelTableDescriptor[] = { 0 };

DEFINE_GET_OBJECT_MODEL_TABLE(GpInputPort)
