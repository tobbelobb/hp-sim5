#include "can/CanMotionHost.h"

#ifdef RRF_HOST_BUILD

#include <fstream>
#include <mutex>

namespace host {

namespace
{
	std::mutex g_mutex;
	std::string g_log_path;

	void write_json_line(std::ofstream& out, const StepSegment& seg)
	{
		out << '{';
		out << "\"start_clock\":" << seg.start_clock << ',';
		out << "\"accel_clocks\":" << seg.accel_clocks << ',';
		out << "\"steady_clocks\":" << seg.steady_clocks << ',';
		out << "\"decel_clocks\":" << seg.decel_clocks << ',';
		out << "\"v_entry\":" << seg.v_entry << ',';
		out << "\"v_top\":" << seg.v_top << ',';
		out << "\"v_exit\":" << seg.v_exit << ',';
		out << "\"drives\":[";
		for (size_t i = 0; i < seg.drives.size(); ++i)
		{
			const auto& d = seg.drives[i];
			if (i != 0)
			{
				out << ',';
			}
			out << '{'
				<< "\"drive_id\":" << static_cast<unsigned int>(d.drive_id) << ','
				<< "\"steps_total\":" << d.steps_total << ','
				<< "\"steps_per_sec_top\":" << d.steps_per_sec_top
				<< '}';
		}
		out << ']';
		out << "}\n";
	}
}

void set_log_path(const std::string& path)
{
	std::scoped_lock lk(g_mutex);
	g_log_path = path;
}

void clear_log()
{
	std::scoped_lock lk(g_mutex);
	if (!g_log_path.empty())
	{
		std::ofstream trunc(g_log_path, std::ios::trunc);
	}
}

void emit_segment(const StepSegment& seg)
{
	std::scoped_lock lk(g_mutex);
	if (g_log_path.empty())
	{
		return;
	}

	std::ofstream out(g_log_path, std::ios::app);
	if (!out.is_open())
	{
		return;
	}

	write_json_line(out, seg);
}

} // namespace host

#endif // RRF_HOST_BUILD
