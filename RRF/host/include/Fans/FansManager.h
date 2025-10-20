#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/StringRef.h>
#include <GCodeResult.h>
#include <RTOSIface/RTOSIface.h>

#include <array>
#include <algorithm>

class GCodeBuffer;
class Fan;
class LocalFan;
#if SUPPORT_CAN_EXPANSION
struct CanMessageFansReport;
#endif
#if SUPPORT_REMOTE_COMMANDS
struct CanMessageGeneric;
struct CanMessageFanParameters;
struct CanMessageSetFanSpeed;
#endif

// Host-only stub for FansManager. All fan control requests are satisfied locally without hardware access.
class FansManager
{
public:
	FansManager() noexcept = default;
	FansManager(const FansManager&) = delete;

	void Init() noexcept { Reset(); }
	void Exit() noexcept { Reset(); }

	bool CheckFans(bool) noexcept { return false; }

	GCodeResult ConfigureFanPort(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
	{
		reply.copy(UnsupportedMessage);
		return GCodeResult::warningNotSupported;
	}

	bool ConfigureFan(unsigned int, size_t, GCodeBuffer&, const StringRef& reply, bool& error) THROWS(GCodeException)
	{
		reply.copy(UnsupportedMessage);
		error = true;
		return false;
	}

	float GetFanValue(size_t fanNum) const noexcept
	{
		return (IsValidFan(fanNum) && states[fanNum].configured) ? states[fanNum].speed : 0.0f;
	}

	GCodeResult SetFanValue(size_t fanNum, float speed, const StringRef&) noexcept
	{
		SetState(fanNum, speed);
		return GCodeResult::ok;
	}

	float SetFanValue(size_t fanNum, float speed) noexcept
	{
		return SetState(fanNum, speed);
	}

	float SetFansValue(FansBitmap whichFans, float speed) noexcept
	{
		const float clamped = std::clamp(speed, 0.0f, 1.0f);
		for (size_t i = 0; i < states.size(); ++i)
		{
			if (whichFans.IsBitSet(i))
			{
				SetState(i, clamped);
			}
		}
		return clamped;
	}

	bool IsFanControllable(size_t fanNum) const noexcept
	{
		return IsValidFan(fanNum) && states[fanNum].configured;
	}

	int32_t GetFanRPM(size_t fanNum) const noexcept
	{
		return (IsValidFan(fanNum) && states[fanNum].configured) ? states[fanNum].rpm : 0;
	}

#if SUPPORT_CAN_EXPANSION
	void ProcessRemoteFanRpms(CanAddress, const CanMessageFansReport&) noexcept {}
#endif

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult ConfigureFanPort(const CanMessageGeneric&, const StringRef& reply) noexcept
	{
		reply.copy(UnsupportedMessage);
		return GCodeResult::warningNotSupported;
	}

	GCodeResult ConfigureFan(const CanMessageFanParameters&, const StringRef& reply) noexcept
	{
		reply.copy(UnsupportedMessage);
		return GCodeResult::warningNotSupported;
	}

	GCodeResult SetFanSpeed(const CanMessageSetFanSpeed&, const StringRef& reply) noexcept
	{
		reply.copy(UnsupportedMessage);
		return GCodeResult::warningNotSupported;
	}

	unsigned int PopulateFansReport(CanMessageFansReport&) noexcept { return 0; }
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteFanSettings(FileStore *) const noexcept { return false; }
#endif

	size_t GetNumFansToReport() const noexcept
	{
		size_t count = 0;
		for (const auto& state : states)
		{
			if (state.configured)
			{
				++count;
			}
		}
		return count;
	}

	ReadLockedPointer<Fan> FindFan(size_t) const noexcept
	{
		return ReadLockedPointer<Fan>(nullptr, nullptr);
	}

	static inline ReadWriteLock fansLock{};

private:
	struct FanState
	{
		bool configured{false};
		float speed{0.0f};
		int32_t rpm{0};
	};

	float SetState(size_t fanNum, float speed) noexcept
	{
		if (!IsValidFan(fanNum))
		{
			return 0.0f;
		}
		auto& state = states[fanNum];
		state.configured = true;
		state.speed = std::clamp(speed, 0.0f, 1.0f);
		state.rpm = 0;
		return state.speed;
	}

	bool IsValidFan(size_t fanNum) const noexcept
	{
		return fanNum < states.size();
	}

	void Reset() noexcept
	{
		for (auto& state : states)
		{
			state = FanState{};
		}
	}

	static constexpr const char *_ecv_array UnsupportedMessage = "Fan control not implemented on host build";

	std::array<FanState, MaxFans> states{};
};

#endif
