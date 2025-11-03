#pragma once

#include <filesystem>

class CanMessageBuffer;

namespace HostCanCapture
{
// Configure the capture sink. When filePath is empty, capture is disabled.
// Returns true on success.
bool Configure(const std::filesystem::path& filePath) noexcept;

// Record a motion CAN message. No effect if Configure() has not enabled capture.
void LogMotion(const CanMessageBuffer& buffer) noexcept;

// Return the total number of captured movement messages.
uint64_t GetCaptureCount() noexcept;

// Return the latest movement completion time expressed in master clock ticks (48MHz domain).
uint64_t GetLatestFinishMasterClock() noexcept;

// Flush any pending output and release resources.
void Shutdown() noexcept;
}
