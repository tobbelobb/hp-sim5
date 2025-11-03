#pragma once

#include <RTOSIface/RTOSNotifyIndices.h>
#include <cstdint>

namespace NotifyIndices
{
// Reuse the RTOS notification slots and extend them for core services
constexpr std::uint32_t UartTx = NextAvailableAfterRTOS;
constexpr std::uint32_t CanDevice = UartTx;  // share the same slot under host shim
constexpr std::uint32_t Usb = UartTx;
constexpr std::uint32_t Sdhc = UartTx;
constexpr std::uint32_t AnalogIn = UartTx + 1;
constexpr std::uint32_t NextAvailableAfterCore = NextAvailableAfterRTOS + 2;
}  // namespace NotifyIndices
