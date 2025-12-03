#pragma once

#include <cstdint>
#include <filesystem>

class CanMessageBuffer;

namespace HostCanCapture
{
// Configure the capture sink. When filePath is empty, capture is disabled.
// Returns true on success.
bool Configure(const std::filesystem::path& filePath) noexcept;

// Record a motion CAN message. No effect if Configure() has not enabled capture.
void LogMotion(const CanMessageBuffer& buffer) noexcept;

// Record a torque mode change event (driver address, torque Nm).
void LogTorqueModeChange(uint8_t driverAddress, float torqueNm) noexcept;

// Return the total number of captured movement messages.
uint64_t GetCaptureCount() noexcept;

// Reset capture state (counters and base clock) without closing the file.
// This allows running multiple prints in the same process with deterministic timing.
void Reset() noexcept;

// Flush any pending output and release resources.
void Shutdown() noexcept;
}  // namespace HostCanCapture
