#include <HostTiming.h>

#include <RepRapFirmware.h>
#include <Platform/Platform.h>
#include <can/CanCapture.h>

#include <array>
#include <atomic>
#include <cstdlib>
#include <cmath>
#include <iostream>

namespace HostTiming
{
namespace
{
std::atomic<Platform*> g_platform{nullptr};
std::atomic<uint64_t> g_virtualClockTicks{0};
std::atomic<uint64_t> g_lastSimulationTicks{0};
thread_local ClockStatKind g_currentClockStat = ClockStatKind::Other;

struct ClockStats
{
    std::atomic<uint64_t> calls{0};
    std::atomic<uint64_t> ticks{0};
};

constexpr size_t kClockStatCount = static_cast<size_t>(ClockStatKind::Count);
std::array<ClockStats, kClockStatCount> g_clockStats{};

const char* ClockStatName(ClockStatKind kind) noexcept
{
    switch (kind)
    {
        case ClockStatKind::Simulation:
            return "simulation";
        case ClockStatKind::WaitLoop:
            return "wait_loop";
        case ClockStatKind::Delay:
            return "delay";
        case ClockStatKind::Other:
        default:
            return "other";
    }
}

void RecordClockAdvance(ClockStatKind kind, uint64_t amount) noexcept
{
    if (amount == 0)
    {
        return;
    }

    const size_t index = static_cast<size_t>(kind);
    if (index >= g_clockStats.size())
    {
        return;
    }

    g_clockStats[index].calls.fetch_add(1, std::memory_order_relaxed);
    g_clockStats[index].ticks.fetch_add(amount, std::memory_order_relaxed);
}

bool ShouldReportClockStats() noexcept
{
    static std::atomic<int8_t> cached{-1};
    int8_t value = cached.load(std::memory_order_acquire);
    if (value == -1)
    {
        const char* env = std::getenv("HP_CLOCK_STATS");
        value = (env != nullptr && env[0] != '\0' && env[0] != '0') ? 1 : 0;
        cached.store(value, std::memory_order_release);
    }
    return value == 1;
}

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
            const uint64_t delta = simTicks - expected;
            g_virtualClockTicks.fetch_add(delta, std::memory_order_relaxed);
            RecordClockAdvance(ClockStatKind::Simulation, delta);
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

ClockTagScope::ClockTagScope(ClockStatKind kind) noexcept : previous(g_currentClockStat)
{
    g_currentClockStat = kind;
}

ClockTagScope::~ClockTagScope()
{
    g_currentClockStat = previous;
}

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
    ResetClockStats();
}

void AdvanceStepClocks(uint64_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    g_virtualClockTicks.fetch_add(value, std::memory_order_relaxed);
    RecordClockAdvance(g_currentClockStat, value);
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
    ClockTagScope clockTag(ClockStatKind::Delay);
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

void ResetClockStats() noexcept
{
    for (auto& stats : g_clockStats)
    {
        stats.calls.store(0, std::memory_order_relaxed);
        stats.ticks.store(0, std::memory_order_relaxed);
    }
}

void DumpClockStats() noexcept
{
    if (!ShouldReportClockStats())
    {
        return;
    }

    std::cout << "Clock advance stats:\n";
    for (size_t i = 0; i < kClockStatCount; ++i)
    {
        const auto calls = g_clockStats[i].calls.load(std::memory_order_relaxed);
        const auto ticks = g_clockStats[i].ticks.load(std::memory_order_relaxed);
        if (calls == 0)
        {
            continue;
        }
        std::cout << "  " << ClockStatName(static_cast<ClockStatKind>(i)) << ": "
                  << "calls=" << calls << " ticks=" << ticks << '\n';
    }
}
}  // namespace HostTiming
