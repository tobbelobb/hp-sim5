#pragma once

#include <cstdint>

class Platform;

namespace HostTiming
{
	inline constexpr uint32_t StepClockFrequencyHz = 48'000'000;

	uint64_t StepClocks64() noexcept;
	uint32_t StepClocks() noexcept;
	uint32_t Millis() noexcept;
	uint64_t Millis64() noexcept;
	uint32_t Micros() noexcept;
	uint64_t Micros64() noexcept;
	void Reset(uint64_t stepClocks = 0) noexcept;
	void AdvanceStepClocks(uint64_t value) noexcept;
	void AdvanceMicros(uint64_t value) noexcept;
	void DelayMilliseconds(uint32_t value) noexcept;
	void DelayMicroseconds(uint32_t value) noexcept;
	void RegisterPlatform(Platform& platform) noexcept;
	void UnregisterPlatform() noexcept;
}
