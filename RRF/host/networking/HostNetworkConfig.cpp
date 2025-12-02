#include "HostNetworkConfig.h"

#ifdef RRF_HOST_BUILD

#include <atomic>
#include <cstddef>

#include <Networking/NetworkDefs.h>

namespace
{
std::atomic<uint16_t> httpPort{8080};
}

namespace HostNetworkConfig
{

void SetHttpPort(uint16_t port) noexcept
{
    const uint16_t chosen = (port == 0) ? static_cast<uint16_t>(DefaultHttpPort) : port;
    httpPort.store(chosen, std::memory_order_relaxed);
}

uint16_t GetHttpPort() noexcept
{
    return httpPort.load(std::memory_order_relaxed);
}

}  // namespace HostNetworkConfig

#endif
