#pragma once

#include "CoreIO.h"
#include <cstdlib>

using AdcInput = uint8_t;
using AnalogChannelNumber = uint8_t;

using AnalogInCallbackFunction = void (*)(CallbackParameter, uint32_t) noexcept;

namespace AnalogIn
{
	inline constexpr unsigned int AdcBits = 12;

	inline void Init(NvicPriority = 0) noexcept {}
	inline void Exit() noexcept {}
	inline bool EnableChannel(AdcInput, AnalogInCallbackFunction, CallbackParameter, uint32_t) noexcept { return false; }
	inline bool SetCallback(AdcInput, AnalogInCallbackFunction, CallbackParameter, uint32_t) noexcept { return false; }
	inline bool IsChannelEnabled(AdcInput) noexcept { return false; }
	inline void DisableChannel(AdcInput) noexcept {}
	inline uint16_t ReadChannel(AdcInput) noexcept { return 0; }
	inline void GetDebugInfo(uint32_t&, uint32_t&, uint32_t&, uint32_t&) noexcept {}
	inline bool EnableTemperatureSensor(unsigned int, AnalogInCallbackFunction, CallbackParameter, uint32_t, unsigned int) noexcept { return false; }
	inline void EnableTemperatureSensor(AnalogInCallbackFunction, CallbackParameter, uint32_t) noexcept {}
	[[noreturn]] inline void TaskLoop(void*) noexcept { std::abort(); }
	using AdcTaskHookFunction = void() noexcept;
	inline AdcTaskHookFunction* SetTaskHook(AdcTaskHookFunction* fn) noexcept { return fn; }
}

namespace LegacyAnalogIn
{
	inline void AnalogInInit() noexcept {}
	inline void AnalogInEnableChannel(AnalogChannelNumber, bool) noexcept {}
	inline constexpr unsigned int AdcBits = AnalogIn::AdcBits;
	inline uint16_t AnalogInReadChannel(AnalogChannelNumber) noexcept { return 0; }
	using AnalogCallback_t = void (*)() noexcept;
	inline AnalogCallback_t AnalogInSetCallback(AnalogCallback_t fn) noexcept { return fn; }
	inline void AnalogInStartConversion(uint32_t = 0xFFFFFFFFu) noexcept {}
	inline void AnalogInFinaliseConversion() noexcept {}
	inline bool AnalogInCheckReady(uint32_t = 0xFFFFFFFFu) noexcept { return true; }
	inline AnalogChannelNumber PinToAdcChannel(uint32_t) noexcept { return 0; }
	inline AnalogChannelNumber GetTemperatureAdcChannel() noexcept { return 0; }
}

inline uint16_t AnalogInReadChannel(AdcInput input) noexcept
{
	return AnalogIn::ReadChannel(input);
}

inline void AnalogInEnableChannel(AdcInput input, bool enable) noexcept
{
	if (enable)
	{
		AnalogIn::IsChannelEnabled(input);
	}
	else
	{
		AnalogIn::DisableChannel(input);
	}
}
