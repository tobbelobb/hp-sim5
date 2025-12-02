#pragma once

#include <atomic>

namespace HostIdle
{
void SetServerMode(bool enabled) noexcept;
void SetServerIdle(bool idle) noexcept;
bool IsServerIdle() noexcept;
}

