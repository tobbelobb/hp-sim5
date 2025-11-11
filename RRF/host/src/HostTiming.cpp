#include <HostTiming.h>

#include <RepRapFirmware.h>
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

uint64_t GetVirtualStepClocks() noexcept
{
    // We want to call `RecordClockAdvance` instantly when events appear
    // in RRFs planner, pretending the event finished instantly.
    // This way we can "set and forget".
    // We don't need to trace every event's execution path.
    //
    // However, on real hw the step timer is a free‑running counter, and
    // the RRF code is made to run "in flight".
    // Being "too late" or "already finished" triggers unwanted special behavior.
    // See eg RRF/ReprapFirmware/src/Movement/DDA.cpp:1156‑1172.
    // If DDA::Prepare() sees that the previous move is already finished,
    // it will inject a 25 ms pause called `AbsoluteMinimumPreparedTime`.
    // This manifests as "stuttering" in a host build.
    //
    // To get the best of both worlds, we pretend that we're always behind by a few ms.
    //constexpr uint64_t ClockDelayWhenReporting = StepClockRate / 100ULL;
    //return g_virtualClockTicks.load(std::memory_order_relaxed) - ClockDelayWhenReporting;
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

void ReportSimulationClocks(uint64_t deltaStepClocks) noexcept
{
    if (deltaStepClocks == 0)
    {
        return;
    }

    g_lastSimulationTicks.fetch_add(deltaStepClocks, std::memory_order_relaxed);
}

void RegisterPlatform(Platform& platform) noexcept
{
    (void)platform;
    Reset();
}

void UnregisterPlatform() noexcept
{
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
