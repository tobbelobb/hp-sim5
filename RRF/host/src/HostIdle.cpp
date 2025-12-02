#include <HostIdle.h>

namespace HostIdle
{
namespace
{
std::atomic<bool> serverMode{false};
std::atomic<bool> serverIdle{false};
}  // namespace

void SetServerMode(bool enabled) noexcept
{
    serverMode.store(enabled, std::memory_order_relaxed);
    if (!enabled)
    {
        serverIdle.store(false, std::memory_order_relaxed);
    }
}

void SetServerIdle(bool idle) noexcept
{
    const bool mode = serverMode.load(std::memory_order_relaxed);
    serverIdle.store(mode && idle, std::memory_order_relaxed);
}

bool IsServerIdle() noexcept
{
    return serverMode.load(std::memory_order_relaxed) &&
           serverIdle.load(std::memory_order_relaxed);
}
}  // namespace HostIdle

