#pragma once

#include "CoreIO.h"

enum class InterruptMode : uint8_t
{
	none = 0,
	low,
	high,
	change,
	falling,
	rising,
	debounce = 0x80
};

inline void InitialiseExints() noexcept {}
inline bool AttachPinInterrupt(Pin, StandardCallbackFunction, InterruptMode, CallbackParameter, bool = true) noexcept { return false; }
inline void DetachPinInterrupt(Pin) noexcept {}
inline void EnablePinInterrupt(Pin) noexcept {}
inline void DisablePinInterrupt(Pin) noexcept {}
inline ExintNumber AttachEvent(Pin, InterruptMode, bool) noexcept { return 0; }
inline void DetachEvent(Pin) noexcept {}
inline bool ReadDebouncedPin(Pin) noexcept { return false; }

