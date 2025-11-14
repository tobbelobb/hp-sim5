#include <HostTiming.h>
#include <Movement/MoveTiming.h>

#include <RepRapFirmware.h>
#include <can/CanCapture.h>

#include <array>
#include <atomic>
#include <cstdlib>
#include <cmath>
#include <iostream>

#include <thread>

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

void RecordClockBackOff(ClockStatKind kind, uint64_t amount) noexcept
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
    g_clockStats[index].ticks.fetch_sub(amount, std::memory_order_relaxed);
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
    // We want to call `AdvanceStepClocks` fully and instantly when events appear
    // and `RecordClockAdvance` when they expire.
    // in RRFs planner, basically pretending the event finished instantly.
    //
    // However, on real hw the step timer is a free‑running counter, and
    // the RRF code is made to run "in flight".
    // Being "too late" or "already finished" might trigger unwanted special behavior.
    // This might manifests as stuttering in a host build, or a hang/early finish by the
    // host build binary.
    //
    // To get the best of both worlds, we pretend that we're always behind by a few ms.
    //constexpr uint64_t delay = MoveTiming::UsualMinimumPreparedTime;
    //uint64_t current_ticks = g_virtualClockTicks.load(std::memory_order_relaxed);
    //return (current_ticks >= delay) ? (current_ticks - delay) : 0;
    return g_virtualClockTicks.load(std::memory_order_relaxed);
}
}  // namespace

void ReportSimulationClocks(uint64_t deltaStepClocks) noexcept
{
    //static uint32_t prev_simdiff = 37500;
    if (deltaStepClocks == 0)
    {
        return;
    }

    g_lastSimulationTicks.fetch_add(deltaStepClocks, std::memory_order_relaxed);
    //auto const v = g_virtualClockTicks.load(std::memory_order_relaxed);
    //auto const s = g_lastSimulationTicks.load(std::memory_order_relaxed);
    //auto const simdiff = v - s;
    //if (simdiff != 37510 )
    //if (simdiff - prev_simdiff != 0)
    //std::cout << "off simdiff: " << simdiff - prev_simdiff << '\n';
    //prev_simdiff = simdiff;

    g_virtualClockTicks.store(g_lastSimulationTicks.load(std::memory_order_relaxed), std::memory_order_relaxed);
}

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
    g_virtualClockTicks.store(stepClocks, std::memory_order_relaxed);
    ResetClockStats();
}

void AdvanceStepClocks(uint64_t value) noexcept
{
    //std::cout << "Advanced " << value << '\n';
    if (value == 0)
    {
        return;
    }
    g_virtualClockTicks.fetch_add(value, std::memory_order_relaxed);
    RecordClockAdvance(g_currentClockStat, value);
    //std::this_thread::sleep_for(std::chrono::nanoseconds(1));
}

void BackOffStepClocks(uint64_t value) noexcept
{
    std::cout << "Backed off " << value << '\n';
    if (value == 0)
    {
        return;
    }
    g_virtualClockTicks.fetch_sub(value, std::memory_order_relaxed);
    RecordClockBackOff(g_currentClockStat, value);
}

void DelayMilliseconds(uint32_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    ClockTagScope clockTag(ClockStatKind::Delay);
    //AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMillisecond);
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
