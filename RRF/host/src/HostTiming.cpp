#include <HostTiming.h>

#include <Platform/Platform.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace HostTiming
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		std::atomic<Platform*> g_platform{nullptr};
		const Clock::time_point g_start = Clock::now();

		inline Platform* TryGetPlatform() noexcept
		{
			return g_platform.load(std::memory_order_acquire);
		}

		template <typename Duration>
		uint64_t FallbackElapsed() noexcept
		{
			return static_cast<uint64_t>(std::chrono::duration_cast<Duration>(Clock::now() - g_start).count());
		}
	}

	uint32_t Millis() noexcept
	{
		if (auto* platform = TryGetPlatform())
		{
			return platform->millis();
		}
		return static_cast<uint32_t>(FallbackElapsed<std::chrono::milliseconds>());
	}

	uint64_t Millis64() noexcept
	{
		if (auto* platform = TryGetPlatform())
		{
			return platform->micros() / 1000ULL;
		}
		return FallbackElapsed<std::chrono::milliseconds>();
	}

	uint32_t Micros() noexcept
	{
		return static_cast<uint32_t>(Micros64());
	}

	uint64_t Micros64() noexcept
	{
		if (auto* platform = TryGetPlatform())
		{
			return platform->micros();
		}
		return FallbackElapsed<std::chrono::microseconds>();
	}

	void DelayMilliseconds(uint32_t value) noexcept
	{
		if (value == 0)
		{
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(value));
	}

	void DelayMicroseconds(uint32_t value) noexcept
	{
		if (value == 0)
		{
			return;
		}
		std::this_thread::sleep_for(std::chrono::microseconds(value));
	}

	void RegisterPlatform(Platform& platform) noexcept
	{
		g_platform.store(&platform, std::memory_order_release);
	}

	void UnregisterPlatform() noexcept
	{
		g_platform.store(nullptr, std::memory_order_release);
	}
}
