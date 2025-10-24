#include "Movement/StepTimerHost.h"

#ifdef RRF_HOST_BUILD

#include <atomic>

namespace host {

namespace
{
	std::atomic<uint64_t> g_ticks{0};
}

uint64_t StepTimerHost::now() noexcept
{
	return g_ticks.load(std::memory_order_relaxed);
}

uint64_t StepTimerHost::align_to_next(uint64_t after, uint32_t min_gap_ticks) noexcept
{
	uint64_t current = g_ticks.load(std::memory_order_relaxed);
	uint64_t base = (after > current) ? after : current;
	uint64_t target = base + static_cast<uint64_t>(min_gap_ticks);
	g_ticks.store(target, std::memory_order_relaxed);
	return target;
}

} // namespace host

#endif // RRF_HOST_BUILD
