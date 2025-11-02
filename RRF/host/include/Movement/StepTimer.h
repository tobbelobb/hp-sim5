#pragma once

#ifdef SRC_MOVEMENT_STEPTIMER_H_
#undef SRC_MOVEMENT_STEPTIMER_H_
#endif
#define SRC_MOVEMENT_STEPTIMER_H_

#include <RepRapFirmware.h>

#include <atomic>
#include <HostTiming.h>

struct CanMessageTimeSync;

// Host-only shim that mimics the StepTimer interface without touching MCU peripherals.
class StepTimer final
{
public:
	using Ticks = uint32_t;
	using TimerCallbackFunction = void (*)(CallbackParameter) noexcept;

	StepTimer() noexcept = default;

	void SetCallback(TimerCallbackFunction cb, CallbackParameter param) noexcept
	{
		callback = cb;
		cbParam = param;
	}

	bool ScheduleCallback(Ticks) noexcept SPEED_CRITICAL
	{
		return false;
	}

	bool ScheduleCallbackFromIsr(Ticks) noexcept SPEED_CRITICAL
	{
		return false;
	}

	bool ScheduleMovementCallbackFromIsr(Ticks) noexcept SPEED_CRITICAL
	{
		return false;
	}

	bool ScheduleCallbackFromIsr() noexcept SPEED_CRITICAL
	{
		return false;
	}

	void CancelCallback() noexcept
	{
	}

	void CancelCallbackFromIsr() noexcept SPEED_CRITICAL
	{
	}

	static void Init() noexcept {}
	static void DisableTimerInterrupt() noexcept {}

	static Ticks GetTimerTicks() noexcept SPEED_CRITICAL
	{
		return NowTicks();
	}

	static Ticks GetTimerTicksWhenInterruptsDisabled() noexcept SPEED_CRITICAL
	{
		return NowTicks();
	}

	static Ticks GetMovementTimerTicks() noexcept SPEED_CRITICAL
	{
		return NowTicks();
	}

	static Ticks ConvertLocalToMovementTime(Ticks localTime) noexcept
	{
		return localTime;
	}

	static uint16_t GetTimerTicks16() noexcept
	{
		return static_cast<uint16_t>(NowTicks());
	}

	static constexpr uint32_t GetTickRate() noexcept
	{
		return StepClockRate;
	}

	static void IncreaseMovementDelay(uint32_t increase) noexcept
	{
		movementDelay.fetch_add(increase, std::memory_order_relaxed);
	}

	static Ticks GetMovementDelay() noexcept
	{
		return movementDelay.load(std::memory_order_relaxed);
	}

#if SUPPORT_CAN_EXPANSION
	static void ProcessMovementDelayRequest(uint32_t delayRequested) noexcept
	{
		movementDelay.store(delayRequested, std::memory_order_relaxed);
	}

	static Ticks CheckMovementDelayIncreased() noexcept
	{
		return movementDelay.exchange(0, std::memory_order_relaxed);
	}

	static uint32_t GetOwnMovementDelay() noexcept
	{
		return 0;
	}
#endif

#if SUPPORT_REMOTE_COMMANDS
	static void ProcessTimeSyncMessage(const CanMessageTimeSync&, size_t, uint16_t) noexcept {}
	static uint32_t ConvertToMasterTime(uint32_t localTime) noexcept { return localTime; }
	static uint32_t GetMasterTime() noexcept { return NowTicks(); }
	static bool CheckSynced() noexcept { return true; }
	static bool IsSynced() noexcept { return true; }
	static constexpr uint32_t MinSyncInterval = 2000;
#endif

	static void Interrupt() noexcept {}

	static void Diagnostics(const StringRef& reply) noexcept
	{
		reply.cat("StepTimer host shim active");
	}

	static uint32_t TicksToIntegerMicroseconds(uint32_t n) noexcept
	{
		return (n * 1000)/(StepClockRate/1000);
	}

	static float TicksToFloatMicroseconds(uint32_t n) noexcept
	{
		return static_cast<float>(n) * (1000000.0f/static_cast<float>(StepClockRate));
	}

private:
	static constexpr uint32_t MasterClocksPerStepTick = HostTiming::StepClockFrequencyHz/StepClockRate;
	static_assert(StepClockRate != 0, "Step clock rate must be non-zero");
	static_assert(HostTiming::StepClockFrequencyHz % StepClockRate == 0, "Master clock must be integer multiple of step clock rate");

	static Ticks NowTicks() noexcept
	{
		const uint64_t stepClocks = HostTiming::StepClocks64();
		return static_cast<Ticks>(stepClocks / MasterClocksPerStepTick);
	}

	TimerCallbackFunction callback{nullptr};
	CallbackParameter cbParam{};
static inline std::atomic<uint32_t> movementDelay{0};
};
