#pragma once

#ifdef RRF_HOST_BUILD

#include <RRF3Common.h>
#include <General/StringRef.h>

namespace SmartDrivers
{
	template <typename... Args>
	inline void Init(Args&&...) noexcept {}

	inline void Exit() noexcept {}

	inline void Spin(bool) noexcept {}
	inline void SpinSmartDrivers(bool) noexcept {}
	inline void TurnDriversOff() noexcept {}

	inline StandardDriverStatus GetStatus(size_t, bool, bool) noexcept { return StandardDriverStatus(); }

	inline float GetTmcDriversTemperature(unsigned int) noexcept { return 0.0f; }

	inline bool EnablePhaseStepping(size_t, bool) noexcept { return false; }
	inline bool IsPhaseSteppingEnabled(size_t) noexcept { return false; }
	inline uint16_t GetMicrostepPosition(size_t) noexcept { return 0; }
	inline int GetMicrostepShift(size_t) noexcept { return 0; }

	inline const char* CheckStallDetectionEnabled(size_t, float) noexcept { return nullptr; }

	inline void SetAxisNumber(size_t, size_t) noexcept {}
	inline void SetStallThreshold(unsigned int, int) noexcept {}
	inline void SetStallFilter(unsigned int, int) noexcept {}
	inline void SetStallMinimumStepsPerSecond(unsigned int, float) noexcept {}

	inline void AppendDriverStatus(size_t, StringRef&) noexcept {}
	inline void AppendStallConfig(size_t, StringRef&) noexcept {}

	template <typename... Args>
	inline void SetRegister(Args&&...) noexcept {}
}

#endif
