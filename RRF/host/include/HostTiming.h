#pragma once

#include <cstdint>

class Platform;

namespace HostTiming
{
uint64_t StepClocks64() noexcept;
uint32_t StepClocks() noexcept;
uint32_t Millis() noexcept;
uint64_t Millis64() noexcept;
void Reset(uint64_t stepClocks = 0) noexcept;
void AdvanceStepClocks(uint64_t value) noexcept;
void EnsureMasterClockAtLeast(uint64_t masterClocks) noexcept;
void DelayMilliseconds(uint32_t value) noexcept;
void RegisterPlatform(Platform& platform) noexcept;
void UnregisterPlatform() noexcept;
}  // namespace HostTiming
