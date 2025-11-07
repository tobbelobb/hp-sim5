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

// Convert microseconds to step clocks, rounding up to the next step clock
static inline constexpr uint32_t MicrosecondsToStepClocks(float us) noexcept
{
	return (uint32_t)ceilf((float)StepClockRate * 0.000001 * us);
}

std::atomic<uint64_t> g_virtualClockTicks{0};
std::atomic<uint64_t> g_lastSimulationTicks{0};
inline constexpr uint32_t StepClockFrequencyHz = 48'000'000;
constexpr uint64_t StepClocksPerMicrosecond = StepClockFrequencyHz / 1'000'000ULL;
constexpr uint64_t StepClocksPerMillisecond = StepClockFrequencyHz / 1'000ULL;

double Platform::GetSimulationTimeSeconds() const noexcept {
    if (reprap == nullptr || gCodes == nullptr || move == nullptr)
    {
        // ^^ All these three are always nullptr for some reason
        return 0.0;
    }
    return static_cast<double>(move->GetSimulationTime()) +  // Get the accumulated simulation time (Print time since we started simulating)
           static_cast<double>(gCodes->GetSimulationTime()); // The GCodes simulationTime underlying state only increments during dwell operations.
                                                             // Should use GetLastDuration() instead?
                                                             // (Time or simulated time of the last successful print or simulation, in
                                                             // seconds)
}
/* The move->GetSimulationTime()'s internal simulationTime state is in the DDARing object:

class Move final INHERIT_OBJECT_MODEL
{
  ...
	float GetSimulationTime() const noexcept { return rings[0].GetSimulationTime(); }		// Get the accumulated simulation time
}
uint32_t DDARing::Spin(uint32_t prepareAdvanceTime, SimulationMode simulationMode, bool signalMoveCompletion, bool shouldStartMove) noexcept
{
	DDA *cdda = getPointer;											// capture volatile variable

	// If we are simulating, simulate completion of the current move
	if (simulationMode >= SimulationMode::normal)
	{
		// Simulate completion of one move
		if (cdda->IsCommitted())
		{
			simulationTime += (float)cdda->GetClocksNeeded() * (1.0/StepClockRate);
			++completedMoves;
-------------

We have a problem becasue we want acces to the simulationTime state, but we have to run with SimulationMode::off in order to
exercise all the code we want to exercise. Maybe we could calculate an equivalent number ourselves, or inject some tiny code into DDARing.cpp?
*/
uint64_t CalculateSimulationTicks(Platform& platform) noexcept {
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
void UpdateFromSimulation() noexcept {
    Platform* platform = TryGetPlatform();
    if (platform == nullptr)
    {
        return;
    }

    // simTicks is always zero because platform.GetSimulationTimeSeconds() is always zero,
    // which is because reprap, gCodes, move are all nullptr (none of them can be nullptr)
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
    // This returns step clocks incremented solely by our manual calls to AdvanceStepClocks
    return g_virtualClockTicks.load(std::memory_order_relaxed);
}
// The same goes for StepClocks64 and StepClocks, because they just wrap GetVirtualStepClocks
uint64_t StepClocks64() noexcept { return GetVirtualStepClocks(); }
uint32_t StepClocks() noexcept { return static_cast<uint32_t>(StepClocks64()); }
// The same also goes for Millis, Millis64, Micros, Micros64
// Because they all just wrap StepClocks64().
uint32_t Millis() noexcept { return static_cast<uint32_t>(Millis64()); }
uint64_t Millis64() noexcept { return Micros64() / 1000ULL; }
uint32_t Micros() noexcept { return static_cast<uint32_t>(Micros64()); }
uint64_t Micros64() noexcept { return StepClocks64() / StepClocksPerMicrosecond; }
// There are also some StepClocks64() wrappers in PlatformHost.cpp.
// Only Platform::millis() is used, and it's only used for logging
uint32_t Platform::millis() const noexcept { return HostTiming::Millis(); }
uint32_t Platform::micros() const noexcept { return HostTiming::Micros(); }
uint64_t Platform::GetStepClockCount() const noexcept { return HostTiming::StepClocks64(); }
// So we continue to look into HostTiming.cpp:
// Since automatic updates of clock doesn't work in our case, we rely solely on these:
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
void AdvanceMicros(uint64_t value) noexcept {
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(value * StepClocksPerMicrosecond);
}
void DelayMilliseconds(uint32_t value) noexcept {
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMillisecond);
}
void DelayMicroseconds(uint32_t value) noexcept {
    if (value == 0)
    {
        return;
    }
    AdvanceStepClocks(static_cast<uint64_t>(value) * StepClocksPerMicrosecond);
}

// In StepTimer.h we have:
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

// There's also the unused branch of clock bookkeeping called "gLatestFinishMasterClock" in RRF/host/can/CanCapture.cpp
std::atomic<uint64_t> gLatestFinishMasterClock{0};
inline void UpdateLatestFinish(uint64_t finishMasterClock) noexcept
{
    uint64_t current = gLatestFinishMasterClock.load(std::memory_order_relaxed);
    while (current < finishMasterClock &&
           !gLatestFinishMasterClock.compare_exchange_weak(current, finishMasterClock,
                                                           std::memory_order_relaxed,
                                                           std::memory_order_relaxed))
    {
    }
}
uint64_t HostCanCapture::GetLatestFinishMasterClock() noexcept
{
    return gLatestFinishMasterClock.load(std::memory_order_relaxed);
}

----

Based on what we have learned from inspecting the time keeping we want to:
 - Delete the gLatestFinishMasterClock stuff (CHECK)
 - Delete all references to a 48MHz frequency, only keep ClockRate, which is 750kHz. (CHECK)
 - Don't advance the clocks by ourselves. Rely on the built in Platform::GetSimulationTimeSeconds() (CHECK)
   ... which in turn relies on move->GetSimulationTime() (and to some degree gCodes->GetSimulationTime()).
   The most important one is cleary Move::GetSimulationTime() which relies on DDARing::GetSimulationTime().
 - We need to "fix" DDARing::GetSimulationTime() so that the internal state variable `simulationTime` is
   updated although we want SimulationMode::off. (CHECK)
