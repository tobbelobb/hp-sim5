#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <Interrupts.h>
#include <PinDescription.h>
#include <General/String.h>
#include <General/StringRef.h>
#include <cstring>

class GCodeBuffer;

// The real firmware exposes a fairly feature rich IoPort abstraction that
// talks to physical pins. For the host build we only need a light-weight
// placeholder that keeps enough state for the motion code to reason about
// virtual brake and enable outputs. All operations simply succeed and
// update a little bit of bookkeeping so diagnostics still have something
// sensible to report.
class IoPort
{
public:
	IoPort() noexcept = default;
	~IoPort() { Release(); }

	bool SetMode(PinAccess) noexcept { return true; }

	void Release() noexcept
	{
		logicalPin = NoLogicalPin;
		hardwareInvert = totalInvert = false;
		isSharedInput = false;
		alternateConfig = false;
		assignedName.Clear();
		state = false;
		analogValue = 0.0f;
	}

	void AppendBasicDetails(const StringRef& str) const noexcept
	{
		if (!assignedName.IsEmpty())
		{
			(void)str.cat(assignedName.c_str());
		}
	}

	static size_t AssignPorts(GCodeBuffer&, const StringRef&, PinUsedBy neededFor, size_t numPorts,
							  IoPort *_ecv_from const ports[], const PinAccess access[]) noexcept
	{
		String<1> dummy;
		StringRef dummyReply = dummy.GetRef();
		size_t assigned = 0;
		for (; assigned < numPorts; ++assigned)
		{
			if (ports[assigned] != nullptr)
			{
				ports[assigned]->Allocate(nullptr, dummyReply, neededFor, access[assigned]);
			}
		}
		return numPorts;
	}

	bool AssignPort(GCodeBuffer&, const StringRef&, PinUsedBy neededFor, PinAccess access) noexcept
	{
		String<1> dummy;
		return Allocate(nullptr, dummy.GetRef(), neededFor, access);
	}

	static size_t AssignPorts(const char *_ecv_array pinNames, const StringRef& reply, PinUsedBy neededFor,
							  size_t numPorts, IoPort *_ecv_from const ports[], const PinAccess access[]) noexcept
	{
		size_t assigned = 0;
		const char* current = pinNames;
		while (assigned < numPorts)
		{
			String<32> name;
			if (current != nullptr && *current != 0)
			{
				const char* nextSep = strchr(current, '+');
				if (nextSep != nullptr)
				{
					name.catn(current, static_cast<size_t>(nextSep - current));
					current = nextSep + 1;
				}
				else
				{
					name.copy(current);
					current = nullptr;
				}
			}
			if (ports[assigned] != nullptr)
			{
				ports[assigned]->Allocate((name.IsEmpty()) ? nullptr : name.c_str(), reply, neededFor, access[assigned]);
			}
			++assigned;
		}
		return numPorts;
	}

	bool AssignPort(const char *_ecv_array pinName, const StringRef& reply, PinUsedBy neededFor,
					PinAccess access) noexcept
	{
		return Allocate(pinName, reply, neededFor, access);
	}

	void AppendPinName(const StringRef& str) const noexcept
	{
		if (!assignedName.IsEmpty())
		{
			(void)str.cat(assignedName.c_str());
		}
		else
		{
			(void)str.cat("none");
		}
	}

	bool IsValid() const noexcept { return logicalPin != NoLogicalPin; }
	bool GetInvert() const noexcept { return totalInvert; }
	void SetInvert(bool pInvert) noexcept { totalInvert = pInvert; }
	void ToggleInvert(bool pInvert) noexcept { totalInvert = pInvert; }
	bool IsHardwareInverted() const noexcept { return hardwareInvert; }
	bool GetTotalInvert() const noexcept { return totalInvert; }

	bool ReadDigital() const noexcept { return totalInvert ? !state : state; }

	bool AttachInterrupt(StandardCallbackFunction, InterruptMode, CallbackParameter) const noexcept { return false; }
	void DetachInterrupt() const noexcept {}

#if SAME5x
	bool SetAnalogCallback(AnalogInCallbackFunction, CallbackParameter, uint32_t) noexcept { return false; }
	void ClearAnalogCallback() noexcept {}
#endif

	uint16_t ReadAnalog() const noexcept { return static_cast<uint16_t>(analogValue * 65535.0f); }

	AnalogChannelNumber GetAnalogChannel() const noexcept { return 0; }

	void WriteDigital(bool high) const noexcept { state = high; }

	void FastDigitalWriteLow() const noexcept { WriteDigital(false); }
	void FastDigitalWriteHigh() const noexcept { WriteDigital(true); }

	Pin GetPin() const noexcept { return logicalPin; }
	PinCapability GetCapability() const noexcept { return PinCapability::none; }

	static void Init() noexcept {}

	static void AppendPinNames(const StringRef& str, size_t numPorts, const IoPort * const ports[]) noexcept
	{
		for (size_t i = 0; i < numPorts; ++i)
		{
			if (i != 0)
			{
				(void)str.cat("+");
			}
			if (ports[i] != nullptr)
			{
				ports[i]->AppendPinName(str);
			}
			else
			{
				(void)str.cat("none");
			}
		}
	}

#if SUPPORT_CAN_EXPANSION
	static CanAddress RemoveBoardAddress(const StringRef&) noexcept { return 0; }
#else
	static bool RemoveBoardAddress(const StringRef&) noexcept { return true; }
#endif

	static void SetPinMode(Pin, PinMode, bool = false) noexcept {}
	static bool ReadPin(Pin) noexcept { return false; }
	static void WriteDigital(Pin, bool) noexcept {}
	static void WriteAnalog(Pin, float, uint16_t) noexcept {}

protected:
	bool Allocate(const char *_ecv_array pinName, const StringRef&, PinUsedBy neededFor, PinAccess access) noexcept
	{
		(void)neededFor;
		(void)access;
		if (pinName != nullptr && pinName[0] != 0)
		{
			assignedName.copy(pinName);
		}
		else
		{
			assignedName.copy("virtual");
		}
		// Treat every assignment as successful and give the port a deterministic but unique logical pin.
		static Pin nextLogical = 0;
		logicalPin = nextLogical++;
		return true;
	}

	Pin GetPinNoCheck() const noexcept { return logicalPin; }

	static const char *_ecv_array TranslatePinAccess(PinAccess) noexcept { return "virtual"; }

	mutable bool state{false};
	mutable float analogValue{0.0f};
	Pin logicalPin{NoLogicalPin};
	bool hardwareInvert{false};
	bool totalInvert{false};
	bool isSharedInput{false};
	bool alternateConfig{false};
	String<32> assignedName;
};

class PwmPort : public IoPort
{
public:
	PwmPort() noexcept = default;

	void AppendFullDetails(const StringRef& str) const noexcept
	{
		AppendPinName(str);
	}

	void AppendFrequency(const StringRef& str) const noexcept
	{
		(void)str.catf("%uHz", static_cast<unsigned int>(frequency));
	}

	void SetFrequency(PwmFrequency freq) noexcept { frequency = freq; }
	PwmFrequency GetFrequency() const noexcept { return frequency; }

	void WriteAnalog(float pwm) const noexcept
	{
		analogValue = pwm;
		WriteDigital(pwm > 0.5f);
	}

	bool SupportsPwm() const noexcept { return true; }

private:
	mutable PwmFrequency frequency{0};
};

#endif
