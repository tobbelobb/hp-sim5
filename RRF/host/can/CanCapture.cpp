#include <CAN/CanCapture.h>

#include <atomic>
#include <cstdint>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>

#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>
#include <CanId.h>

namespace
{
std::mutex gMutex;
std::ofstream gStream;
std::filesystem::path gOutputPath;
bool gEnabled = false;
std::atomic<uint64_t> gCaptureIndex{0};

}

bool HostCanCapture::Configure(const std::filesystem::path& filePath) noexcept
{
	std::lock_guard<std::mutex> lock(gMutex);

	if (filePath.empty())
	{
		if (gStream.is_open())
		{
			gStream.flush();
			gStream.close();
		}
		gOutputPath.clear();
		gEnabled = false;
		return true;
	}

	std::error_code ec;
	const auto directory = filePath.parent_path();
	if (!directory.empty())
	{
		std::filesystem::create_directories(directory, ec);
		if (ec)
		{
			gEnabled = false;
			return false;
		}
	}

	gStream.close();
	gStream.clear();
	gStream.open(filePath, std::ios::out | std::ios::trunc);
	if (!gStream.is_open())
	{
		gEnabled = false;
		return false;
	}

	gStream << "{\"capture_version\":1,\"generated_at\":\""
	        << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())
	        << "\"}\n";
	gStream.flush();

	gOutputPath = filePath;
	gEnabled = true;
	gCaptureIndex.store(0, std::memory_order_relaxed);
	return true;
}

void HostCanCapture::LogMotion(const CanMessageBuffer& buffer) noexcept
{
	if (!gEnabled)
	{
		return;
	}

	if (buffer.id.MsgType() != CanMessageType::movementLinearShaped)
	{
		return;
	}

	const auto& msg = buffer.msg.moveLinearShaped;

	std::ostringstream line;
	line.setf(std::ios::fixed, std::ios::floatfield);
	line << std::setprecision(6);

	const uint64_t captureIndex = gCaptureIndex.fetch_add(1, std::memory_order_relaxed);

	line << "{\"type\":\"movement_linear_shaped\"";
	line << ",\"capture_index\":" << captureIndex;
	line << ",\"destination\":" << static_cast<unsigned int>(buffer.id.Dst());
	line << ",\"when_to_execute\":" << msg.whenToExecute;
	line << ",\"accel_clocks\":" << msg.accelerationClocks;
	line << ",\"steady_clocks\":" << msg.steadyClocks;
	line << ",\"decel_clocks\":" << msg.decelClocks;
	line << ",\"acceleration\":" << msg.acceleration;
	line << ",\"deceleration\":" << msg.deceleration;
	line << ",\"seq\":" << static_cast<unsigned int>(msg.seq & CanMessageMovementLinearShaped::SeqMask);
	line << ",\"extruder_mask\":" << static_cast<unsigned int>(msg.extruderDrives);
	line << ",\"use_pressure_advance\":" << (msg.usePressureAdvance ? "true" : "false");
	line << ",\"use_late_input_shaping\":" << (msg.useLateInputShaping ? "true" : "false");

	line << ",\"drivers\":[";
	for (uint32_t i = 0; i < msg.numDrivers && i < MaxLinearDriversPerCanSlave; ++i)
	{
		if (i > 0)
		{
			line << ',';
		}
		const bool isExtruder = ((msg.extruderDrives >> i) & 0x1u) != 0;
		line << "{\"index\":" << i;
		if (isExtruder)
		{
			line << ",\"extrusion\":" << msg.perDrive[i].extrusion;
		}
		else
		{
			line << ",\"steps\":" << msg.perDrive[i].steps;
		}
		line << "}";
	}
	line << "]}";

	std::lock_guard<std::mutex> lock(gMutex);
	if (!gEnabled || !gStream.is_open())
	{
		return;
	}

	gStream << line.str() << '\n';
	gStream.flush();
}

void HostCanCapture::Shutdown() noexcept
{
	std::lock_guard<std::mutex> lock(gMutex);
	if (gStream.is_open())
	{
		gStream.flush();
		gStream.close();
	}
	gOutputPath.clear();
	gEnabled = false;
}
