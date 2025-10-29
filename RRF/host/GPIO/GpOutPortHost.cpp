#include <GPIO/GpOutPort.h>
#include <cstdint>

namespace {
	inline void UNUSED_(const void*) {}
}

// Object model note:
// In the host build, the object-model macros usually collapse to no-ops.
// If stubs define something like IMPLEMENT_OBJECT_MODEL(...),
// you can drop it in here guarded by #ifdef as needed.

bool GpOutputPort::IsUnused() const noexcept
{
	// Treat "unused" as "not actively driven" on host.
	// This is arbitrary but harmless; adjust if your code expects otherwise.
	return lastPwm == 0.0f;
}

#if SUPPORT_CAN_EXPANSION
bool GpOutputPort::IsLocal() const noexcept
{
	// On host, consider everything "local" by default.
	// If you prefer, compare to CanInterface::GetCanAddress().
	return true;
}
#endif

GCodeResult GpOutputPort::WriteAnalog(uint32_t gpioPortNumber,
                                      bool /*isServo*/,
                                      float pwm,
                                      const GCodeBuffer& gb,
                                      const StringRef& reply) noexcept
{
	UNUSED_(&gpioPortNumber);
	UNUSED_(&gb);
	UNUSED_(&reply);

	// Host stub: just remember last value.
	lastPwm = pwm;
	return GCodeResult::ok;
}

void GpOutputPort::WriteDigital(bool value) noexcept
{
	// Represent digital on/off as 1.0 / 0.0 in our simple host stub.
	lastPwm = value ? 1.0f : 0.0f;
}

GCodeResult GpOutputPort::Configure(uint32_t gpioNumber,
                                    bool /*isServo*/,
                                    GCodeBuffer& gb,
                                    const StringRef& reply) THROWS(GCodeException)
{
	UNUSED_(&gpioNumber);
	UNUSED_(&gb);
	UNUSED_(&reply);

	// Host stub: accept any configuration without touching hardware.
	return GCodeResult::ok;
}

void GpOutputPort::WriteAnalog(float pwm) noexcept
{
	lastPwm = pwm;
}

#if SUPPORT_REMOTE_COMMANDS
GCodeResult GpOutputPort::AssignFromRemote(uint32_t gpioPortNumber,
                                           const CanMessageGenericParser& parser,
                                           const StringRef& reply) noexcept
{
	UNUSED_(&gpioPortNumber);
	UNUSED_(&parser);
	UNUSED_(&reply);

	// Host stub: pretend remote assignment succeeded.
	return GCodeResult::ok;
}
#endif

#ifdef PCCB
void GpOutputPort::Assign(const char *pinName) noexcept
{
	UNUSED_(pinName);
	// Host stub: no-op.
}
#endif
