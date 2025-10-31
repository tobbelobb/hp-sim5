#include <Platform/Platform.h>
#include <Platform/Event.h>
#include <General/StringRef.h>
#include <cstdarg>

MessageType Event::GetTextDescription(const StringRef& str) noexcept
{
	str.copy("No pending events");
	return GenericMessage;
}

bool Event::AddEvent(EventType, uint16_t, CanAddress, uint8_t, const char*, ...) noexcept
{
	return false;
}

bool Event::AddEventV(EventType, uint16_t, CanAddress, uint8_t, const char*, va_list) noexcept
{
	return false;
}

#if SUPPORT_CAN_EXPANSION
void Event::Add(const CanMessageEvent&, CanAddress, size_t) noexcept {}
#endif

bool Event::StartProcessing() noexcept
{
	return false;
}

void Event::GetMacroFileName(const StringRef& fname) noexcept
{
	fname.copy("event.g");
}

void Event::GetParameters(VariableSet&) noexcept {}

PrintPausedReason Event::GetDefaultPauseReason() noexcept
{
	return PrintPausedReason::dontPause;
}

void Event::FinishedProcessing() noexcept {}

void Event::Diagnostics(const StringRef& reply, Platform&) noexcept
{
	reply.copy("Event system unavailable on host");
}
