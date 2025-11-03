#pragma once

#include "CoreIO.h"
#include "CoreTypes.h"

#include <cstdint>

constexpr unsigned int NumDmaChannelsUsed = 0;
constexpr unsigned int NumDmaChannelsSupported = 0;

enum class DmaCallbackReason : uint8_t
{
    none = 0,
    complete = 1
};

using DmaCallbackFunction = void (*)(CallbackParameter, DmaCallbackReason) noexcept;

enum class DmaTrigSource : uint8_t
{
    hsmci = 0,
    spi0tx,
    spi0rx,
    spi1tx,
    spi1rx,
    qspitx,
    qspirx,
    usart0tx,
    usart0rx,
    usart1tx,
    usart1rx,
    usart2tx,
    usart2rx,
    pwm0tx,
    twihs0tx,
    twihs0rx,
    twihs1tx,
    twihs1rx,
    twihs2tx,
    twihs2rx,
    uart0tx,
    uart0rx,
    uart1tx,
    uart1rx,
    uart2tx,
    uart2rx,
    uart3tx,
    uart3rx,
    uart4tx,
    uart4rx,
    dacctx,
    unused1,
    ssctx,
    sscrx,
    pioarx,
    afec0rx,
    afec1rx,
    aestx,
    aesrx,
    pwm1tx,
    tc0rx,
    tc3rx,
    tc6rx,
    tc9rx,
    i2sc0txl,
    i2sc0rxl,
    i2sc1txl,
    i2sc1rxl,
    i2sc0txr,
    i2sc0rxr,
    i2sc1txr,
    i2sc1rxr,
    numPeripheralIds
};

namespace DmacManager
{
inline void Init() noexcept
{
}
inline void SetBtctrl(DmaChannel, uint16_t) noexcept
{
}
inline void SetBtctrl(DmaChannel, uint32_t) noexcept
{
}
inline void SetSourceAddress(DmaChannel, const volatile void*) noexcept
{
}
inline void SetDestinationAddress(DmaChannel, volatile void*) noexcept
{
}
inline void SetDataLength(DmaChannel, uint32_t) noexcept
{
}
inline void SetTriggerSource(DmaChannel, DmaTrigSource) noexcept
{
}
inline void SetTriggerSourceSercomTx(DmaChannel, uint8_t) noexcept
{
}
inline void SetTriggerSourceSercomRx(DmaChannel, uint8_t) noexcept
{
}
inline void SetArbitrationLevel(DmaChannel, uint8_t) noexcept
{
}
inline uint16_t GetBytesTransferred(DmaChannel) noexcept
{
    return 0;
}
inline void EnableChannel(DmaChannel, DmaPriority) noexcept
{
}
inline bool DisableChannel(DmaChannel) noexcept
{
    return true;
}
inline bool SuspendChannel(DmaChannel) noexcept
{
    return true;
}
inline void ResumeChannel(DmaChannel) noexcept
{
}
inline void SetInterruptCallback(DmaChannel, DmaCallbackFunction,
                                 CallbackParameter) noexcept
{
}
inline void EnableCompletedInterrupt(DmaChannel) noexcept
{
}
inline void DisableCompletedInterrupt(DmaChannel) noexcept
{
}
inline uint8_t GetAndClearChannelStatus(DmaChannel) noexcept
{
    return 0;
}
}  // namespace DmacManager
