#include <HostTiming.h>

#include <Platform/Platform.h>

#include <atomic>
#include <cmath>

namespace HostTiming
{
	namespace
	{
		std::atomic<Platform*> g_platform{nullptr};
		std::atomic<uint64_t> g_virtualClockMicros{0};
		std::atomic<uint64_t> g_lastSimulationMicros{0};

		inline Platform* TryGetPlatform() noexcept
		{
			return g_platform.load(std::memory_order_acquire);
		}

		uint64_t CalculateSimulationMicros(Platform& platform) noexcept
		{
			const double totalSeconds = platform.GetSimulationTimeSeconds();
			if (!std::isfinite(totalSeconds) || totalSeconds <= 0.0)
			{
				return 0;
			}
			const double micros = totalSeconds * 1'000'000.0;
			if (!std::isfinite(micros) || micros <= 0.0)
			{
				return 0;
			}
			return static_cast<uint64_t>(micros);
		}

		void UpdateFromSimulation() noexcept
		{
			Platform* platform = TryGetPlatform();
			if (platform == nullptr)
			{
				return;
			}

			const uint64_t simMicros = CalculateSimulationMicros(*platform);
			uint64_t prevSim = g_lastSimulationMicros.load(std::memory_order_relaxed);

			if (simMicros < prevSim)
			{
				g_lastSimulationMicros.store(simMicros, std::memory_order_relaxed);
				g_virtualClockMicros.store(simMicros, std::memory_order_relaxed);
				return;
			}

			while (simMicros > prevSim && !g_lastSimulationMicros.compare_exchange_weak(
					   prevSim, simMicros, std::memory_order_relaxed, std::memory_order_relaxed))
			{
			}

			if (simMicros > prevSim)
			{
				g_virtualClockMicros.fetch_add(simMicros - prevSim, std::memory_order_relaxed);
			}
		}

		uint64_t GetVirtualMicros() noexcept
		{
			UpdateFromSimulation();
			return g_virtualClockMicros.load(std::memory_order_relaxed);
		}
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
		return GetVirtualMicros();
	}

	void Reset(uint64_t value) noexcept
	{
		g_lastSimulationMicros.store(value, std::memory_order_relaxed);
		g_virtualClockMicros.store(value, std::memory_order_relaxed);
	}

	void AdvanceMicros(uint64_t value) noexcept
	{
		if (value == 0)
		{
			return;
		}
		g_virtualClockMicros.fetch_add(value, std::memory_order_relaxed);
	}

	void DelayMilliseconds(uint32_t value) noexcept
	{
		if (value == 0)
		{
			return;
		}
		AdvanceMicros(static_cast<uint64_t>(value) * 1000ULL);
	}

	void DelayMicroseconds(uint32_t value) noexcept
	{
		if (value == 0)
		{
			return;
		}
		AdvanceMicros(static_cast<uint64_t>(value));
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
}
