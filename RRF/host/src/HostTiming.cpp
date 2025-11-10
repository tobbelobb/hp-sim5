#include <HostTiming.h>

#include <RepRapFirmware.h>
#include <Platform/Platform.h>
#include <can/CanCapture.h>

#include <atomic>
#include <cmath>
#include <iostream>

namespace HostTiming
{
namespace
{
std::atomic<Platform*> g_platform{nullptr};
std::atomic<uint64_t> g_virtualClockTicks{0};
std::atomic<uint64_t> g_lastSimulationTicks{0};

constexpr uint64_t StepClocksPerMicrosecond = StepClockRate / 1'000'000ULL;
constexpr uint64_t StepClocksPerMillisecond = StepClockRate / 1'000ULL;

inline Platform* TryGetPlatform() noexcept
{
    return g_platform.load(std::memory_order_acquire);
}

uint64_t CalculateSimulationTicks(Platform& platform) noexcept
{
    const double totalSeconds = platform.GetSimulationTimeSeconds();
    if (!std::isfinite(totalSeconds) || totalSeconds <= 0.0)
    {
        return 0;
    }
    const double ticks = totalSeconds * static_cast<double>(StepClockRate);
    if (!std::isfinite(ticks) || ticks <= 0.0)
    {
        return 0;
    }
    return static_cast<uint64_t>(ticks);
}

void UpdateFromSimulation() noexcept
{
    Platform* platform = TryGetPlatform();
    if (platform == nullptr)
    {
        return;
    }

    const uint64_t simTicks = CalculateSimulationTicks(*platform);
    uint64_t expected = g_lastSimulationTicks.load(std::memory_order_relaxed);

    /* If simTicks is ahead of our last known simulation time, try to atomically update g_lastSimulationTicks to simTicks.
     * If we win that race, also advance g_virtualClockTicks by exactly how much we moved forward.
     * If another thread gets there first (and moves it far enough), we stop.
     * Only one thread accounts for each bit of time progress.
     */
    if (simTicks <= expected)
    {
        return;
    }

    while (expected < simTicks)
    {
        if (g_lastSimulationTicks.compare_exchange_weak(
                expected, simTicks, std::memory_order_relaxed, std::memory_order_relaxed))
        {
            g_virtualClockTicks.fetch_add(simTicks - expected, std::memory_order_relaxed);
            break;
        }

        if (expected >= simTicks)
        {
            break;
        }
    }
}
uint64_t GetVirtualStepClocks() noexcept
{
    UpdateFromSimulation();
    return g_virtualClockTicks.load(std::memory_order_relaxed);
}
}  // namespace

uint64_t StepClocks64() noexcept
{
    return GetVirtualStepClocks();
}

uint32_t StepClocks() noexcept
{
    return static_cast<uint32_t>(StepClocks64());
}

uint32_t Millis() noexcept
{
    return static_cast<uint32_t>(Millis64());
}

uint64_t Millis64() noexcept
{
    return StepClocks64() / StepClocksPerMillisecond;
}

void Reset(uint64_t stepClocks) noexcept
{
    g_lastSimulationTicks.store(stepClocks, std::memory_order_relaxed);
    g_virtualClockTicks.store(stepClocks, std::memory_order_relaxed);
}

void AdvanceStepClocks(uint64_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    g_virtualClockTicks.fetch_add(value, std::memory_order_relaxed);
}

void EnsureMasterClockAtLeast(uint64_t masterClocks) noexcept
{
    uint64_t current = g_virtualClockTicks.load(std::memory_order_relaxed);
    while (current < masterClocks && !g_virtualClockTicks.compare_exchange_weak(
                                         current, masterClocks, std::memory_order_relaxed,
                                         std::memory_order_relaxed))
    {
    }

    uint64_t lastSim = g_lastSimulationTicks.load(std::memory_order_relaxed);
    while (lastSim < masterClocks && !g_lastSimulationTicks.compare_exchange_weak(
                                         lastSim, masterClocks, std::memory_order_relaxed,
                                         std::memory_order_relaxed))
    {
    }
}

void DelayMilliseconds(uint32_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMillisecond);
}

void RegisterPlatform(Platform& platform) noexcept
{
    Reset();
    g_platform.store(&platform, std::memory_order_release);
}

void UnregisterPlatform() noexcept
{
    g_platform.store(nullptr, std::memory_order_release);
}
}  // namespace HostTiming
