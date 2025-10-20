#pragma once

#include "Core.h"

#include <cstdint>
#include <array>

// Provide a generous pin namespace for host builds
constexpr unsigned int NumTotalPins = 256;

enum class GpioPinFunction : uint8_t
{
	A = 0,
	B,
	C,
	D,
	E,
	F,
	G,
	H,
	I,
	J,
	K,
	L,
	M,
	N
};

struct PinCapabilities
{
	bool canPwm = false;
	bool canAnalog = false;
};

inline char *heapTop = nullptr;
inline const char *heapLimit = nullptr;
inline const char *sysStackLimit = nullptr;

inline void SetPinFunction(Pin, GpioPinFunction) noexcept {}
inline void SetDriveStrength(Pin, unsigned int) noexcept {}
inline void ClearPinFunction(Pin) noexcept {}
inline void EnablePullup(Pin) noexcept {}
inline void DisablePullup(Pin) noexcept {}
inline void EnablePulldown(Pin) noexcept {}
inline void DisablePulldown(Pin) noexcept {}

inline void SetPinMode(Pin, PinMode) noexcept {}
inline PinMode GetPinMode(Pin) noexcept { return INPUT; }

inline void DigitalWrite(Pin, bool) noexcept {}
inline bool DigitalRead(Pin) noexcept { return false; }
inline void TogglePin(Pin) noexcept {}

inline float ReadAnalogPin(Pin) noexcept { return 0.0f; }

inline void DelayNs(uint32_t) noexcept {}
inline void DelayUs(uint32_t) noexcept {}
inline void DelayMs(uint32_t) noexcept {}

inline uint32_t GetCycles64() noexcept { return 0; }

inline bool IsPinInput(Pin) noexcept { return true; }
inline bool IsPinOutput(Pin) noexcept { return false; }

inline bool IsPinReserved(Pin) noexcept { return false; }

inline void ConfigurePinAsOutput(Pin, bool = false) noexcept {}
inline void ConfigurePinAsInput(Pin) noexcept {}

inline constexpr Pin PortAPin(unsigned int n) noexcept { return static_cast<Pin>(n); }
inline constexpr Pin PortBPin(unsigned int n) noexcept { return static_cast<Pin>(32 + n); }
inline constexpr Pin PortCPin(unsigned int n) noexcept { return static_cast<Pin>(64 + n); }
inline constexpr Pin PortDPin(unsigned int n) noexcept { return static_cast<Pin>(96 + n); }
inline constexpr Pin PortEPin(unsigned int n) noexcept { return static_cast<Pin>(128 + n); }

union CallbackParameter
{
	void* vp;
	uint32_t u32;
	int32_t i32;

	explicit CallbackParameter(void* pp) noexcept : vp(pp) {}
	explicit CallbackParameter(uint32_t pp) noexcept : u32(pp) {}
	explicit CallbackParameter(int32_t pp) noexcept : i32(pp) {}
	CallbackParameter() noexcept : u32(0) {}
};

using StandardCallbackFunction = void (*)(CallbackParameter) noexcept;

inline void WatchdogInit() noexcept {}
inline void WatchdogReset() noexcept {}
inline void WatchdogResetSecondary() noexcept {}

inline uint32_t RandomNumber(uint32_t howbig) noexcept { return howbig ? (howbig - 1) : 0; }
