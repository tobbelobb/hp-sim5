#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/StringRef.h>
#include <General/String.h>
#include <GCodeResult.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <RTOSIface/RTOSIface.h>

#include <array>
#include <algorithm>
#include <bitset>

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

	GCodeResult ConfigureFanPort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
	{
		const uint32_t fanNum = gb.GetLimitedUIValue('F', MaxFans);
		auto& state = states[fanNum];
		bool seenParameter = false;

		if (gb.Seen('C'))
		{
			String<StringLength50> pinName;
			gb.GetReducedString(pinName.GetRef());
			if (pinName.EqualsIgnoreCase(NoPinName))
			{
				state = FanState{};
				reply.printf("Fan %u deleted", (unsigned)fanNum);
				return GCodeResult::ok;
			}
			state.portName.copy(pinName.c_str());
			state.configured = true;
			seenParameter = true;
		}

		if (gb.Seen('Q'))
		{
			state.frequency = static_cast<float>(gb.GetPwmFrequency());
			state.configured = true;
			seenParameter = true;
		}

		if (gb.Seen('K'))
		{
			state.pulsesPerRev = gb.GetLimitedFValue('K', MinFanPulsesPerRev, MaxFanPulsesPerRev);
			state.configured = true;
			seenParameter = true;
		}

		if (!seenParameter)
		{
			if (!state.configured)
			{
				reply.printf("Fan %u not configured", (unsigned)fanNum);
			}
			else
			{
				reply.printf(
					"Fan %u: port %s, freq %.1f Hz, PPR %.1f",
					(unsigned)fanNum,
					state.portName.IsEmpty() ? "unset" : state.portName.c_str(),
					(double)state.frequency,
					(double)state.pulsesPerRev);
			}
		}

		return GCodeResult::ok;
	}

	bool ConfigureFan(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
	{
		error = false;
	if (!IsValidFan(fanNum))
	{
		reply.printf("Fan number %u out of range", static_cast<unsigned>(fanNum));
		error = true;
		return true;
	}

	auto& state = states[fanNum];
	if (!state.configured)
	{
		state = FanState{};
		state.configured = true;
	}

		bool handled = false;

		if (mcode == 106)
		{
			if (gb.Seen('T'))
			{
				float temps[2] = { state.triggerTemperatures[0], state.triggerTemperatures[1] };
				size_t count = 2;
				gb.GetFloatArray(temps, count, true);
				if (count >= 1)
				{
					state.triggerTemperatures[0] = temps[0];
				}
				if (count >= 2)
				{
					state.triggerTemperatures[1] = temps[1];
				}
				state.thermostatic = state.thermostatic || state.monitoredSensors.any();
				handled = true;
			}

			if (gb.Seen('B'))
			{
				(void)gb.GetFValue();
				handled = true;
			}

			if (gb.Seen('L'))
			{
				state.minSpeed = std::clamp(gb.GetPwmValue(), 0.0f, 1.0f);
				if (state.maxSpeed < state.minSpeed)
				{
					state.maxSpeed = state.minSpeed;
				}
				handled = true;
			}

			if (gb.Seen('X'))
			{
				state.maxSpeed = std::clamp(gb.GetPwmValue(), state.minSpeed, 1.0f);
				handled = true;
			}

			if (gb.Seen('H'))
			{
				int32_t sensors[MaxSensors];
				size_t count = MaxSensors;
				gb.GetIntArray(sensors, count, false);
				state.monitoredSensors.reset();
				bool foundSensor = false;
				for (size_t i = 0; i < count; ++i)
				{
					const int32_t h = sensors[i];
					if (h < 0)
					{
						continue;
					}
					if (h >= static_cast<int32_t>(MaxSensors))
					{
						reply.copy("Sensor number out of range");
						error = true;
					}
					else
					{
						state.monitoredSensors.set(static_cast<size_t>(h));
						foundSensor = true;
					}
				}
				state.thermostatic = foundSensor;
				if (!foundSensor)
				{
					state.speed = std::clamp(state.speed, state.minSpeed, state.maxSpeed);
				}
				else
				{
					state.speed = state.maxSpeed;
				}
				handled = true;
			}

			if (gb.Seen('C'))
			{
				String<MaxFanNameLength> name;
				gb.GetQuotedString(name.GetRef());
				state.name.copy(name.c_str());
				handled = true;
			}
		}

		const bool seenS = gb.Seen('S');
		float desiredSpeed = 0.0f;
		if (seenS)
		{
			desiredSpeed = gb.GetPwmValue();
		}

		if (handled || seenS)
		{
			if (seenS)
			{
				state.speed = std::clamp(desiredSpeed, state.minSpeed, state.maxSpeed);
			}
			state.configured = true;
			return true;
		}

		reply.printf(
			"Fan %u%s, speed %d%%, min %d%%, max %d%%",
			static_cast<unsigned>(fanNum),
			state.name.IsEmpty() ? "" : state.name.c_str(),
			(int)(state.speed * 100.0f),
			(int)(state.minSpeed * 100.0f),
			(int)(state.maxSpeed * 100.0f));
		if (state.thermostatic)
		{
			reply.catf(
				", thermostatic %.1f:%.1fC sensors:",
				(double)state.triggerTemperatures[0],
				(double)state.triggerTemperatures[1]);
			bool first = true;
			for (size_t i = 0; i < MaxSensors; ++i)
			{
				if (state.monitoredSensors.test(i))
				{
					reply.catf("%s%u", first ? " " : ",", static_cast<unsigned>(i));
					first = false;
				}
			}
			if (first)
			{
				reply.cat(" none");
			}
		}

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
		bool thermostatic{false};
		float speed{0.0f};
		float minSpeed{0.0f};
		float maxSpeed{1.0f};
		float triggerTemperatures[2]{DefaultHotEndFanTemperature, DefaultHotEndFanTemperature};
		float frequency{static_cast<float>(DefaultFanPwmFreq)};
		float pulsesPerRev{DefaultFanTachoPulsesPerRev};
		std::bitset<MaxSensors> monitoredSensors{};
		int32_t rpm{0};
		String<StringLength50> portName{};
		String<MaxFanNameLength> name{};
	};

	float SetState(size_t fanNum, float speed) noexcept
	{
		if (!IsValidFan(fanNum))
		{
			return 0.0f;
		}
		auto& state = states[fanNum];
		state.configured = true;
		const float lower = state.minSpeed;
		const float upper = std::max(state.maxSpeed, lower);
		state.speed = std::clamp(speed, lower, upper);
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
