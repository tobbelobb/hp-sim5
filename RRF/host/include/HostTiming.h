#pragma once

#include <cstdint>

class Platform;

namespace HostTiming
{
enum class ClockStatKind : uint8_t
{
    Simulation = 0,
    WaitLoop,
    Delay,
    Other,
    Count
};

class ClockTagScope
{
public:
    explicit ClockTagScope(ClockStatKind kind) noexcept;
    ~ClockTagScope();

private:
    ClockStatKind previous;
};

uint64_t StepClocks64() noexcept;
uint32_t StepClocks() noexcept;
uint32_t Millis() noexcept;
uint64_t Millis64() noexcept;
void Reset(uint64_t stepClocks = 0) noexcept;
void AdvanceStepClocks(uint64_t value) noexcept;
void ReportSimulationClocks(uint64_t deltaStepClocks) noexcept;
void DelayMilliseconds(uint32_t value) noexcept;
void RegisterPlatform(Platform& platform) noexcept;
void UnregisterPlatform() noexcept;
void ResetClockStats() noexcept;
void DumpClockStats() noexcept;
}  // namespace HostTiming
