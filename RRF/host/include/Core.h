#pragma once

#include <ecv_duet3d.h>

#ifdef RRF_HOST_BUILD
#include <Config/Features_Host.h>
#endif

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CoreTypes.h"

#ifdef RRF_HOST_BUILD
#include <HostTiming.h>
#endif
// Host defaults: no MCU-specific peripherals
#ifndef SAMC21
#define SAMC21 0
#endif
#ifndef SAM3XA
#define SAM3XA 0
#endif
#ifndef SAM4E
#define SAM4E 0
#endif
#ifndef SAM4S
#define SAM4S 0
#endif
#ifndef SAME5x
#define SAME5x 0
#endif
#ifndef SAME70
#define SAME70 0
#endif
#ifndef RP2040
#define RP2040 0
#endif
#ifndef STM32
#define STM32 0
#endif

#ifndef CORE_USES_TINYUSB
#define CORE_USES_TINYUSB 0
#endif
#ifndef SUPPORT_SDHC
#define SUPPORT_SDHC 0
#endif
#ifndef SUPPORT_USB
#define SUPPORT_USB 0
#endif
#ifndef SUPPORT_CAN
#define SUPPORT_CAN 1
#endif

#ifndef likely
#define likely(x) (x)
#endif

#ifndef unlikely
#define unlikely(x) (x)
#endif

using coreIrqflags_t = uint32_t;
using irqflags_t = coreIrqflags_t;

enum PinMode : int
{
    PIN_MODE_NOT_CONFIGURED = -1,
    INPUT = 0,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    OUTPUT_LOW,
    OUTPUT_HIGH,
    AIN,
    OUTPUT_PWM_LOW,
    OUTPUT_PWM_HIGH
};

constexpr uint32_t SystemCoreClockFreq = 100'000'000;
inline uint32_t SystemCoreClock = SystemCoreClockFreq;
constexpr uint32_t IRAM_ADDR = 0;
constexpr uint32_t IRAM_SIZE = 0;
constexpr uint32_t IFLASH_SIZE = 16 * 1024 * 1024;

#ifndef __FP16_TYPE_DEFINED
#define __FP16_TYPE_DEFINED
using __fp16 = float;
#endif

extern "C" void AppInit() noexcept;
extern "C" void AppMain() noexcept;

inline void CoreInit() noexcept
{
}
inline void CoreSysTick() noexcept
{
}

struct PinDescriptionBase
{
};

inline const PinDescriptionBase* GetPinDescription(Pin) noexcept
{
    return nullptr;
}

inline uint32_t GetSdhcClockSpeed() noexcept
{
    return 0;
}

struct CoreDebug_Type
{
    uint32_t DEMCR = 0;
};

struct DWT_Type
{
    uint32_t COMP0 = 0;
    uint32_t MASK0 = 0;
    uint32_t FUNCTION0 = 0;
};

inline CoreDebug_Type CoreDebugStorage{};
inline CoreDebug_Type* const CoreDebug = &CoreDebugStorage;

inline DWT_Type DWTStorage{};
inline DWT_Type* const DWT = &DWTStorage;

constexpr uint32_t CoreDebug_DEMCR_TRCENA_Msk = 0;
constexpr uint32_t CoreDebug_DEMCR_MON_EN_Msk = 0;

struct SysTick_Type
{
    uint32_t CTRL{0};
    uint32_t LOAD{0};
    uint32_t VAL{0};
};

inline SysTick_Type SysTickStorage{};
inline SysTick_Type* const SysTick = &SysTickStorage;

struct SCB_Type
{
    uint32_t CCR{0};
    uint32_t VTOR{0};
};

inline SCB_Type SCBStorage{};
inline SCB_Type* const SCB = &SCBStorage;

constexpr uint32_t SysTick_CTRL_TICKINT_Msk = 0;
constexpr uint32_t SCB_CCR_UNALIGN_TRP_Msk = 0;

inline void __DMB() noexcept
{
}
inline void __enable_irq() noexcept
{
}
inline void __disable_irq() noexcept
{
}
inline uint32_t __get_PRIMASK() noexcept
{
    return 0;
}
inline void __set_PRIMASK(uint32_t) noexcept
{
}
inline uint32_t __get_IPSR() noexcept
{
    return 0;
}

inline uint32_t millis() noexcept
{
    return HostTiming::Millis();
}
inline uint64_t millis64() noexcept
{
    return HostTiming::Millis64();
}
inline uint32_t micros() noexcept
{
    return HostTiming::Micros();
}
inline uint64_t micros64() noexcept
{
    return HostTiming::Micros64();
}
inline void delay(uint32_t value) noexcept
{
    HostTiming::DelayMilliseconds(value);
}
inline void delayMicroseconds(uint32_t value) noexcept
{
    HostTiming::DelayMicroseconds(value);
}
inline coreIrqflags_t IrqSave() noexcept
{
    return 0;
}
inline void IrqRestore(coreIrqflags_t) noexcept
{
}
inline void IrqDisable() noexcept
{
}
inline void IrqEnable() noexcept
{
}
inline bool IsIrqEnabledInFlags(coreIrqflags_t) noexcept
{
    return true;
}
inline uint32_t __get_BASEPRI() noexcept
{
    return 0;
}
inline void __set_BASEPRI(uint32_t) noexcept
{
}
inline void __set_BASEPRI_MAX(uint32_t) noexcept
{
}
inline bool inInterrupt() noexcept
{
    return false;
}

inline void memcpyu32(uint32_t* dest, const uint32_t* src, size_t count) noexcept
{
    if (dest != nullptr && src != nullptr && count != 0)
    {
        std::memcpy(dest, src, count * sizeof(uint32_t));
    }
}

inline void memmoveu32(uint32_t* dest, const uint32_t* src, size_t count) noexcept
{
    if (dest != nullptr && src != nullptr && count != 0)
    {
        std::memmove(dest, src, count * sizeof(uint32_t));
    }
}

inline bool memequ32(const uint32_t* a, const uint32_t* b, size_t count) noexcept
{
    if (a == b)
    {
        return true;
    }
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    return std::memcmp(a, b, count * sizeof(uint32_t)) == 0;
}

inline void memsetf(float* dest, float val, size_t count) noexcept
{
    if (dest == nullptr || count == 0)
    {
        return;
    }
    std::fill_n(dest, count, val);
}

inline void memseti32(int32_t* dest, int32_t val, size_t count) noexcept
{
    if (dest == nullptr || count == 0)
    {
        return;
    }
    std::fill_n(dest, count, val);
}

inline bool memeqf(const float* a, const float* b, size_t count) noexcept
{
    if (a == b)
    {
        return true;
    }
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    for (size_t i = 0; i < count; ++i)
    {
        if (a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}

inline bool isDigit(char c) noexcept
{
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
}

inline bool isAlpha(char c) noexcept
{
    return std::isalpha(static_cast<unsigned char>(c)) != 0;
}

inline bool isAlnum(char c) noexcept
{
    return std::isalnum(static_cast<unsigned char>(c)) != 0;
}

inline long random(long limit) noexcept
{
    if (limit <= 0)
    {
        return 0;
    }
    return static_cast<long>(std::rand() % limit);
}

inline const uint32_t* GetStackPointer() noexcept
{
    return nullptr;
}

inline void __DSB() noexcept
{
}

inline void NVIC_SetPriority(int, uint32_t) noexcept
{
}
inline void NVIC_EnableIRQ(int) noexcept
{
}
inline void NVIC_DisableIRQ(int) noexcept
{
}

constexpr int XDMAC_IRQn = 0;
constexpr int PIOA_IRQn = 1;
constexpr int PIOB_IRQn = 2;
constexpr int PIOC_IRQn = 3;
constexpr int USBHS_IRQn = 4;
constexpr int MCAN0_INT0_IRQn = 5;
constexpr int MCAN1_INT0_IRQn = 6;

constexpr uint32_t REG_RSTC_SR = 0;
constexpr uint32_t RSTC_SR_RSTTYP_Msk = 0;
constexpr uint32_t RSTC_SR_RSTTYP_Pos = 0;

constexpr uint32_t SysTick_CTRL_ENABLE_Msk = 0;

inline void NVIC_ClearPendingIRQ(int) noexcept
{
}

constexpr uint32_t NvicPriorityDMA = 0;
constexpr uint32_t NvicPriorityPins = 0;
constexpr uint32_t NvicPriorityUSB = 0;
constexpr uint32_t NvicPriorityCan = 0;
constexpr uint32_t NvicPriorityHSMCI = 0;
constexpr uint32_t NvicPriorityWatchdog = 0;
constexpr uint32_t NvicPriorityStep = 0;
