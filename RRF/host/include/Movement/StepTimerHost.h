#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdint>

namespace host {

struct StepTimerHost
{
	static constexpr uint32_t TICKS_PER_SEC = 200'000; // coarse but stable tick rate

	static uint64_t now() noexcept;
	static uint64_t align_to_next(uint64_t after, uint32_t min_gap_ticks) noexcept;
};

} // namespace host

#endif // RRF_HOST_BUILD
