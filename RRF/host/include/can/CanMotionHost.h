#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdint>
#include <string>
#include <vector>

namespace host {

struct StepSegment
{
	uint64_t start_clock = 0;
	uint32_t accel_clocks = 0;
	uint32_t steady_clocks = 0;
	uint32_t decel_clocks = 0;

	double v_entry = 0.0;
	double v_top = 0.0;
	double v_exit = 0.0;

	struct Drive
	{
		uint8_t drive_id = 0;
		int64_t steps_total = 0;
		double steps_per_sec_top = 0.0;
	};

	std::vector<Drive> drives;
};

void set_log_path(const std::string& path);
void clear_log();
void emit_segment(const StepSegment& seg);

} // namespace host

#endif // RRF_HOST_BUILD
