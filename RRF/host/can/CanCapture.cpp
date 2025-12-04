#include "CanCapture.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <iostream>
#include <string>

#include <CanId.h>
#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>
#include <HostTiming.h>
#include <RepRapFirmware.h>

namespace
{
std::mutex gMutex;
std::ofstream gStream;
std::filesystem::path gOutputPath;
bool gEnabled = false;
std::atomic<uint64_t> gCaptureIndex{0};
std::atomic<uint64_t> gLastExtendedWhen{0};
std::atomic<uint64_t> gBaseMasterClock{std::numeric_limits<uint64_t>::max()};
std::vector<std::string> gMemoryBuffer;
std::mutex gMemoryMutex;
std::atomic<bool> gCaptureToMemory{false};
constexpr size_t kMaxMemoryBufferSize = 10000;

static_assert(StepClockRate != 0, "Step clock rate must not be zero");

inline uint64_t ExtendTimestamp(uint32_t raw) noexcept
{
    uint64_t expected = gLastExtendedWhen.load(std::memory_order_relaxed);
    for (;;)
    {
        const uint64_t high = expected & 0xFFFFFFFF00000000ULL;
        const uint32_t low = static_cast<uint32_t>(expected & 0xFFFFFFFFUL);
        const uint64_t candidate =
            (raw < low) ? (high + (1ULL << 32) + raw) : (high + raw);
        if (gLastExtendedWhen.compare_exchange_weak(expected, candidate,
                                                    std::memory_order_relaxed,
                                                    std::memory_order_relaxed))
        {
            return candidate;
        }
    }
}

inline uint64_t NormaliseMasterClock(uint64_t absoluteMasterClock) noexcept
{
    uint64_t base = gBaseMasterClock.load(std::memory_order_relaxed);
    if (base == std::numeric_limits<uint64_t>::max())
    {
        if (gBaseMasterClock.compare_exchange_strong(base, absoluteMasterClock,
                                                     std::memory_order_relaxed,
                                                     std::memory_order_relaxed))
        {
            return 0;
        }
        base = gBaseMasterClock.load(std::memory_order_relaxed);
    }
    if (absoluteMasterClock <= base)
    {
        return 0;
    }
    return absoluteMasterClock - base;
}
}  // namespace

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
    gLastExtendedWhen.store(0, std::memory_order_relaxed);
    gBaseMasterClock.store(std::numeric_limits<uint64_t>::max(),
                           std::memory_order_relaxed);
    return true;
}

void HostCanCapture::LogMotion(const CanMessageBuffer& buffer) noexcept
{
    const bool captureMemory = gCaptureToMemory.load(std::memory_order_relaxed);
    if (!gEnabled && !captureMemory)
    {
        return;
    }

    if (buffer.id.MsgType() != CanMessageType::movementLinearShaped)
    {
        std::cerr << "Got a MsgType that is not CanMessageType::movementLinearShaped: " << static_cast<int>(buffer.id.MsgType()) << '\n';
        return;
    }

    const auto& msg = buffer.msg.moveLinearShaped;

    const uint64_t captureIndex = gCaptureIndex.fetch_add(1, std::memory_order_relaxed);
    const uint64_t accelClocks = static_cast<uint64_t>(msg.accelerationClocks);
    const uint64_t steadyClocks = static_cast<uint64_t>(msg.steadyClocks);
    const uint64_t decelClocks = static_cast<uint64_t>(msg.decelClocks);

    const uint64_t absoluteWhen = ExtendTimestamp(msg.whenToExecute);
    const uint64_t normalisedAbsoluteWhen = NormaliseMasterClock(absoluteWhen);

    std::ostringstream line;
    line.setf(std::ios::fixed, std::ios::floatfield);
    line << std::setprecision(0);

    // Minified...
    line << captureIndex;
    line << "," << static_cast<unsigned int>(buffer.id.Dst());
    line << "," << normalisedAbsoluteWhen;
    line << "," << accelClocks;
    line << "," << steadyClocks;
    line << "," << decelClocks;
    const std::streamsize originalPrecision = line.precision();
    for (uint8_t drive = 0; drive < msg.numDrivers; ++drive)
    {
        line << ",";
        const bool isExtruder = (msg.extruderDrives & (1u << drive)) != 0;
        if (isExtruder)
        {
            line << std::setprecision(6) << msg.perDrive[drive].extrusion;
            line.precision(originalPrecision);
        }
        else
        {
            line << msg.perDrive[drive].steps;
        }
    }
    line << "," << std::scientific << std::setprecision(8) << msg.acceleration;
    line << "," << std::scientific << std::setprecision(8) << msg.deceleration;
    line << std::defaultfloat;
    line.precision(originalPrecision);

    const std::string formatted = line.str();

    if (captureMemory)
    {
        std::lock_guard<std::mutex> memoryLock(gMemoryMutex);
        if (gMemoryBuffer.size() >= kMaxMemoryBufferSize)
        {
            gMemoryBuffer.erase(gMemoryBuffer.begin());
        }
        gMemoryBuffer.emplace_back(formatted);
    }

    if (gEnabled)
    {
        std::lock_guard<std::mutex> lock(gMutex);
        gStream << formatted << '\n';
    }
}

void HostCanCapture::LogTorqueModeChange(uint8_t driverAddress, float torqueNm) noexcept
{
    const bool captureMemory = gCaptureToMemory.load(std::memory_order_relaxed);
    if (!gEnabled && !captureMemory)
    {
        return;
    }

    std::ostringstream oss;
    oss << "T," << static_cast<unsigned int>(driverAddress) << "," << torqueNm;
    const std::string entry = oss.str();

    if (captureMemory)
    {
        std::lock_guard<std::mutex> memoryLock(gMemoryMutex);
        if (gMemoryBuffer.size() >= kMaxMemoryBufferSize)
        {
            gMemoryBuffer.erase(gMemoryBuffer.begin());
        }
        gMemoryBuffer.emplace_back(entry);
    }

    if (gEnabled)
    {
        std::lock_guard<std::mutex> lock(gMutex);
        gStream << entry << '\n';
        gStream.flush();
    }
}

uint64_t HostCanCapture::GetCaptureCount() noexcept
{
    return gCaptureIndex.load(std::memory_order_relaxed);
}

void HostCanCapture::Reset() noexcept
{
    gCaptureIndex.store(0, std::memory_order_relaxed);
    gLastExtendedWhen.store(0, std::memory_order_relaxed);
    gBaseMasterClock.store(std::numeric_limits<uint64_t>::max(),
                           std::memory_order_relaxed);
    gCaptureToMemory.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> memoryLock(gMemoryMutex);
        gMemoryBuffer.clear();
    }
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
    gBaseMasterClock.store(std::numeric_limits<uint64_t>::max(),
                           std::memory_order_relaxed);
}

void HostCanCapture::StartCapture() noexcept
{
    std::lock_guard<std::mutex> memoryLock(gMemoryMutex);
    gMemoryBuffer.clear();
    gCaptureToMemory.store(true, std::memory_order_relaxed);
}

void HostCanCapture::StopCapture() noexcept
{
    gCaptureToMemory.store(false, std::memory_order_relaxed);
}

bool HostCanCapture::IsCapturing() noexcept
{
    return gCaptureToMemory.load(std::memory_order_relaxed);
}

std::string HostCanCapture::FlushCapture()
{
    std::lock_guard<std::mutex> memoryLock(gMemoryMutex);

    std::string aggregated;
    aggregated.reserve(gMemoryBuffer.size() * 32);  // heuristic
    for (const auto& line : gMemoryBuffer)
    {
        aggregated += line;
        aggregated += '\n';
    }

    gMemoryBuffer.clear();
    return aggregated;
}
