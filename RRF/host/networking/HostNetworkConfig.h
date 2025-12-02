#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdint>

namespace HostNetworkConfig
{
void SetHttpPort(uint16_t port) noexcept;
uint16_t GetHttpPort() noexcept;
}

#endif
