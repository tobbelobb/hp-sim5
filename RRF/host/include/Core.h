#pragma once

#include <cstddef>
#include <cstdint>

// Host defaults: no MCU-specific peripherals
#ifndef SAMC21
# define SAMC21 0
#endif
#ifndef SAM3XA
# define SAM3XA 0
#endif
#ifndef SAM4E
# define SAM4E 0
#endif
#ifndef SAM4S
# define SAM4S 0
#endif
#ifndef SAME5x
# define SAME5x 0
#endif
#ifndef SAME70
# define SAME70 0
#endif
#ifndef RP2040
# define RP2040 0
#endif
#ifndef STM32
# define STM32 0
#endif

#ifndef CORE_USES_TINYUSB
# define CORE_USES_TINYUSB 0
#endif
#ifndef SUPPORT_SDHC
# define SUPPORT_SDHC 0
#endif
#ifndef SUPPORT_USB
# define SUPPORT_USB 0
#endif
#ifndef SUPPORT_CAN
# define SUPPORT_CAN 1
#endif

#ifndef likely
# define likely(x)   (x)
#endif

#ifndef unlikely
# define unlikely(x) (x)
#endif

using Pin = uint32_t;

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

extern "C" void AppInit() noexcept;
extern "C" void AppMain() noexcept;

inline void CoreInit() noexcept {}
inline void CoreSysTick() noexcept {}

struct PinDescriptionBase {};

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

inline void __DMB() noexcept {}
inline void __enable_irq() noexcept {}
inline void __disable_irq() noexcept {}
inline uint32_t __get_PRIMASK() noexcept { return 0; }
inline void __set_PRIMASK(uint32_t) noexcept {}
inline uint32_t __get_IPSR() noexcept { return 0; }
