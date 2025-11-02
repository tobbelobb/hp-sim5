#pragma once

#include <cstdint>

class Platform;

namespace HostTiming
{
	uint32_t Millis() noexcept;
	uint64_t Millis64() noexcept;
	uint32_t Micros() noexcept;
	uint64_t Micros64() noexcept;
	void Reset(uint64_t value = 0) noexcept;
	void AdvanceMicros(uint64_t value) noexcept;
	void DelayMilliseconds(uint32_t value) noexcept;
	void DelayMicroseconds(uint32_t value) noexcept;
	void RegisterPlatform(Platform& platform) noexcept;
	void UnregisterPlatform() noexcept;
}
