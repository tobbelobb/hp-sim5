#pragma once

#ifdef RRF_HOST_BUILD

#include <array>
#include <cstddef>
#include <cstdint>

#include <General/StringRef.h>
#include <GCodeResult.h>
#include <GCodes/GCodeChannel.h>
#include <GCodes/GCodeMachineState.h>

class GCodeBuffer;

enum class MachineType : uint8_t
{
	fff,
	cnc,
	laser
};

// Minimal host stub for the GCodes manager.  It exposes just enough surface
// for the current G-code buffer stack to compile and issue replies.
class GCodes
{
public:
	GCodes() noexcept = default;

	MachineType GetMachineType() const noexcept { return machineType; }
	const char* GetAxisLetters() const noexcept { return axisLetters; }

	void AbortPrint(GCodeBuffer&) noexcept {}

	void HandleReply(GCodeBuffer&, GCodeResult, const char*) noexcept {}

	size_t GetNumInputs() const noexcept { return inputAxesRelative.size(); }

	bool GetAxesRelative(size_t index) const noexcept
	{
		return (index < inputAxesRelative.size()) ? inputAxesRelative[index] : false;
	}

	void SetAxesRelative(size_t index, bool value) noexcept
	{
		if (index < inputAxesRelative.size())
		{
			inputAxesRelative[index] = value;
		}
	}

	size_t GetAxisCount() const noexcept { return axisCount; }

	void SetAxisCount(size_t count) noexcept
	{
		axisCount = (count <= userPosition.size()) ? count : userPosition.size();
	}

	float GetUserPosition(size_t axis) const noexcept
	{
		return (axis < userPosition.size()) ? userPosition[axis] : 0.0f;
	}

	void SetUserPosition(size_t axis, float value) noexcept
	{
		if (axis < userPosition.size())
		{
			userPosition[axis] = value;
		}
	}

	void ResetUserPositions() noexcept { userPosition.fill(0.0f); }

private:
	MachineType machineType{MachineType::fff};
	const char* axisLetters{"XYZABC"};
	std::array<bool, 1> inputAxesRelative{{false}};
	std::array<float, MaxAxes> userPosition{};
	size_t axisCount{3};
};

#endif
