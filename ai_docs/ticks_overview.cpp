// File: RRF/ReprapFirmware/src/RepRapFirmware.h
constexpr uint32_t StepClockRate = 48000000/64;								// 750kHz
constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
constexpr float StepClocksToSeconds = 1.0/(float)StepClockRate;
static inline constexpr uint32_t MillisToStepClocks(uint32_t numMillis) noexcept
{
	if constexpr (StepClockRate % 1000 == 0)
	{
		return numMillis * (StepClockRate/1000);			// this works for Duet 3, step clock rate is 750kHz
	}
	if constexpr (StepClockRate % 500 == 0)
	{
		return (numMillis * (StepClockRate/500))/2;			// this works for Duet 2, step clock rate is 937500Hz
	}
	return (numMillis * (uint64_t)StepClockRate)/1000;		// catch-all in case of using other step clock rates
}

// File: RRF/host/src/HostTiming.cpp
std::atomic<uint64_t> g_virtualClockTicks{0};
std::atomic<uint64_t> g_lastSimulationTicks{0};

//File RRF/host/platform/PlatformHost.cpp
double Platform::GetSimulationTimeSeconds() const noexcept
{
    return static_cast<double>(move->GetSimulationTime()) +  // Get the accumulated simulation time (Print time since we started simulating)
           static_cast<double>(gCodes->GetSimulationTime()); // The GCodes simulationTime underlying state only increments during dwell operations.
}
/* The move->GetSimulationTime()'s internal simulationTime state is in the DDARing object:
// File: RRF/ReprapFirmware/src/Movement/Move.h
class Move final INHERIT_OBJECT_MODEL
{
  ...
	float GetSimulationTime() const noexcept { return rings[0].GetSimulationTime(); }		// Get the accumulated simulation time
}
// File: RRF/ReprapFirmware/src/Movement/DDARing.cpp
// Try to process moves in the ring. Called by the Move task.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or MoveTiming::StandardMoveWakeupInterval if there are no unprepared moves left.
uint32_t DDARing::Spin(uint32_t prepareAdvanceTime, SimulationMode simulationMode, bool signalMoveCompletion, bool shouldStartMove) noexcept
{
	DDA *cdda = getPointer;											// capture volatile variable

#if RRF_HOST_BUILD
  // On host we want to keep track of simulation time even when we're not running in simulation mode
	if (cdda->IsCommitted() && simulationMode == SimulationMode::off)
	{
		simulationTime += (float)cdda->GetClocksNeeded() * (1.0/StepClockRate);
	}
#endif
*/

// File: RRF/host/src/HostTiming.cpp
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
uint64_t GetVirtualStepClocks() noexcept {
    UpdateFromSimulation();
    return g_virtualClockTicks.load(std::memory_order_relaxed);
}
// StepClocks64, StepClock, Millis, Millis64, just wrap GetVirtualStepClocks
uint64_t StepClocks64() noexcept { return GetVirtualStepClocks(); }
uint32_t StepClocks() noexcept { return static_cast<uint32_t>(StepClocks64()); }
uint64_t Millis64() noexcept { return StepClocks64() / StepClocksPerMillisecond; }
uint32_t Millis() noexcept { return static_cast<uint32_t>(Millis64()); }
// There are also some StepClocks64() wrappers in RRF/host/platform/PlatformHost.cpp.
// Only Platform::millis() is used, and it's only used for logging
uint32_t Platform::millis() const noexcept { return HostTiming::Millis(); }
uint64_t Platform::GetStepClockCount() const noexcept { return HostTiming::StepClocks64(); }
// So we continue to look into RRF/host/src/HostTiming.cpp:
void AdvanceStepClocks(uint64_t value) noexcept {
    if (value == 0)
    {
        return;
    }
    g_virtualClockTicks.fetch_add(value, std::memory_order_relaxed);
}

void EnsureMasterClockAtLeast(uint64_t masterClocks) noexcept
{
    // Conceptually:
    // g_virtualClockTicks = max(g_virtualClockTicks, masterClocks);
    // g_lastSimulationTicks = max(g_lastSimulationTicks, masterClocks);
    // ...implemented safely under concurrency.
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
void DelayMilliseconds(uint32_t value) noexcept {
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMillisecond);
}
// In RRF/host/include/Movement/StepTimer.h we have:
// Host-only shim that mimics the StepTimer interface without touching MCU peripherals.
class StepTimer final
{
public:
    using Ticks = uint32_t;
    using TimerCallbackFunction = void (*)(CallbackParameter) noexcept;
    //...
    // Several wrappers of NowTicks()
    static Ticks GetTimerTicks() noexcept SPEED_CRITICAL { return NowTicks(); }
    static Ticks GetTimerTicksWhenInterruptsDisabled() noexcept SPEED_CRITICAL { return NowTicks(); }
    static Ticks GetMovementTimerTicks() noexcept SPEED_CRITICAL { return NowTicks(); }
    static Ticks ConvertLocalToMovementTime(Ticks localTime) noexcept { return localTime; }
    static uint16_t GetTimerTicks16() noexcept { return static_cast<uint16_t>(NowTicks()); }
    static constexpr uint32_t GetTickRate() noexcept { return StepClockRate; }
    static uint32_t ConvertToMasterTime(uint32_t localTime) noexcept { return localTime; }
    static uint32_t GetMasterTime() noexcept { return NowTicks(); }
    static uint32_t TicksToIntegerMicroseconds(uint32_t n) noexcept
    {
        return (n * 1000) / (StepClockRate / 1000);
    }

    static float TicksToFloatMicroseconds(uint32_t n) noexcept
    {
        return static_cast<float>(n) * (1000000.0f / static_cast<float>(StepClockRate));
    }
    static constexpr uint32_t MasterClocksPerStepTick =
        HostTiming::StepClockFrequencyHz / StepClockRate;
    static Ticks NowTicks() noexcept {
        const uint64_t stepClocks = HostTiming::StepClocks64();
        return static_cast<Ticks>(stepClocks / MasterClocksPerStepTick);
    }
}
// Then, in order to make the simulation clock not stall completely, we have this in the spin loop of
// RRF/host/src/main.cpp:
        HostTiming::AdvanceStepClocks(1);
// It's manual and hopelessly dependent on the number of Spin() iterations, which is not deterministic.
// However, it's the closest to deterministic we've managed to get thus far.
// We're looking for a way to either advance manually only a constant, deterministic amount of ticks,
// or not do manual clock advancement at all.


// Here's some timing related code from RRF/host/rtos/freertos_shim.cpp that I don't really understand.
TickType_t xTaskGetTickCount() noexcept
{
    // Use virtual clock for deterministic simulation
    return static_cast<TickType_t>(HostTiming::Millis());
}

TickType_t xTaskGetTickCountFromISR() noexcept
{
    return xTaskGetTickCount();
}

UBaseType_t uxTaskGetNumberOfTasks() noexcept
{
    std::lock_guard<std::mutex> lock(tasksMutex);
    return static_cast<UBaseType_t>(allTasks.size());
}

void vTaskDelay(const TickType_t ticksToDelay) noexcept
{
    if (ticksToDelay == 0)
    {
        std::this_thread::yield();
        return;
    }
    // Use virtual delay for deterministic simulation
    // Convert ticks to milliseconds (1 tick = 1ms)
    uint32_t delayMs = static_cast<uint32_t>(ticksToDelay);
    HostTiming::DelayMilliseconds(delayMs);

    // Still yield to allow other threads to run
    std::this_thread::yield();
}

void vTaskDelayUntil(TickType_t* const lastWakeTime,
                     const TickType_t ticksToWait) noexcept
{
    if (lastWakeTime == nullptr)
    {
        vTaskDelay(ticksToWait);
        return;
    }

    const TickType_t currentTicks = xTaskGetTickCount();
    TickType_t target = *lastWakeTime + ticksToWait;
    if (target <= currentTicks)
    {
        target = currentTicks + ticksToWait;
    }

    const TickType_t waitTicks = target > currentTicks ? (target - currentTicks) : 0;
    vTaskDelay(waitTicks);
    *lastWakeTime = target;
}

// There's also these functions in RRF/host/include/Core.h
// They're just even more wrappers of the same old Millis() which wraps StepClocks64() which wraps GetVirtualStepClocks().
// ... and HostTiming::DelayMilliseconds() which increments g_virtualClockTicks via AdvanceStepClocks()...
inline uint32_t millis() noexcept
{
    return HostTiming::Millis();
}
inline uint64_t millis64() noexcept
{
    return HostTiming::Millis64();
}
inline void delay(uint32_t value) noexcept
{
    HostTiming::DelayMilliseconds(value);
}

// This commend in RRF/ReprapFirmware/src/Movement/Move.cpp also gives some important hints about initial delays...
//
// Let ring 0 process moves
// When there is a gap between moves it can be that we try to prepare the second move while a segment of the first move that has been delayed by input shaping is still executing.
// To avoid this we must ensure that we prepare moves at least half an input shaper period in advance. This avoids the problem because any delayed segment of the first move
// will be half a shaper period long. In order to handle CAN delays etc. we prepare moves [half a shaper period plus MoveTiming::AbsoluteMinimumPreparedTime] in advance,
// with a minimum of MoveTiming::UsualMinimumPreparedTime.

