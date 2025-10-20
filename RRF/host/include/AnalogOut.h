#pragma once

#include "CoreIO.h"

namespace AnalogOut
{
	inline void Init() noexcept {}
	inline void Write(Pin, float, PwmFrequency = 500) noexcept {}
	inline void Beep(Pin, Pin, PwmFrequency) noexcept {}
}

