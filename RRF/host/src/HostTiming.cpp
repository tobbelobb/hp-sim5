#include <HostTiming.h>

#include <Platform/Platform.h>
#include <can/CanCapture.h>

#include <atomic>
#include <cmath>

namespace HostTiming
{
namespace
{
std::atomic<Platform*> g_platform{nullptr};
std::atomic<uint64_t> g_virtualClockTicks{0};
std::atomic<uint64_t> g_lastSimulationTicks{0};

static_assert(StepClockFrequencyHz % 1'000'000U == 0,
              "Step clock frequency must be integer multiple of 1MHz");
static_assert(StepClockFrequencyHz % 1'000U == 0,
              "Step clock frequency must be integer multiple of 1kHz");
constexpr uint64_t StepClocksPerMicrosecond = StepClockFrequencyHz / 1'000'000ULL;
constexpr uint64_t StepClocksPerMillisecond = StepClockFrequencyHz / 1'000ULL;
static_assert(StepClocksPerMicrosecond > 0, "Step clock frequency must be >= 1MHz");
static_assert(StepClocksPerMillisecond > 0, "Step clock frequency must be >= 1kHz");

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
    const double ticks = totalSeconds * static_cast<double>(StepClockFrequencyHz);
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
    return Micros64() / 1000ULL;
}

uint32_t Micros() noexcept
{
    return static_cast<uint32_t>(Micros64());
}

uint64_t Micros64() noexcept
{
    return StepClocks64() / StepClocksPerMicrosecond;
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
    uint64_t current = g_virtualClockTicks.load(std::memory_order_relaxed);
    do
    {
        uint64_t target = current + value;
        if (HostCanCapture::GetCaptureCount() != 0)
        {
            const uint64_t latestFinish = HostCanCapture::GetLatestFinishMasterClock();
            if (latestFinish != 0)
            {
                const uint64_t maxAllowed =
                    latestFinish +
                    StepClockFrequencyHz;  // allow up to ~1s beyond latest finish
                if (target > maxAllowed)
                {
                    target = maxAllowed;
                }
            }
        }
        if (g_virtualClockTicks.compare_exchange_weak(
                current, target, std::memory_order_relaxed, std::memory_order_relaxed))
        {
            break;
        }
    } while (true);
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

void AdvanceMicros(uint64_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(value * StepClocksPerMicrosecond);
}

void DelayMilliseconds(uint32_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMillisecond);
}

void DelayMicroseconds(uint32_t value) noexcept
{
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMicrosecond);
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
