#pragma once

#ifdef RRF_HOST_BUILD

#include <array>
#include <cstddef>
#include <cstdint>

#include <General/StringRef.h>
#include <GCodeResult.h>
#include <GCodes/GCodeChannel.h>

class GCodeBuffer;

enum class MachineType : uint8_t
{
	fff,
	cnc,
	laser
};

// Host-side façade that captures just enough of the firmware GCodes surface for the bootstrapping build.
class GCodes
{
public:
	GCodes() noexcept;

	MachineType GetMachineType() const noexcept { return MachineType::fff; }
	const char* GetAxisLetters() const noexcept { return axisLettersString.data(); }

	void AbortPrint(GCodeBuffer&) noexcept {}
	void HandleReply(GCodeBuffer&, GCodeResult, const char*) noexcept {}

	// Host helpers ----------------------------------------------------------------
	void RegisterInput(GCodeBuffer& buffer) noexcept;
	void ClearInputs() noexcept;

	size_t GetNumInputs() const noexcept { return numInputs; }
	bool GetAxesRelative(size_t index) const noexcept;
	void SetAxesRelative(size_t index, bool value) noexcept;

	size_t GetAxisCount() const noexcept { return axisCount; }
	void SetAxisCount(size_t count) noexcept;

	bool TryGetAxisIndex(char letter, size_t& index) const noexcept;
	float GetUserPosition(size_t axis) const noexcept;
	void SetUserPosition(size_t axis, float value) noexcept;
	void ResetUserPositions() noexcept;

private:
	static constexpr size_t kMaxInputs = 4;
	static constexpr size_t kMaxAxes = 10;

	void UpdateAxisLettersString() noexcept;

	std::array<GCodeBuffer*, kMaxInputs> inputs{};
	std::array<bool, kMaxInputs> inputAxesRelative{};
	size_t numInputs{0};

	std::array<char, kMaxAxes> axisLetters{'X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'U', 'V', 'W'};
	std::array<char, kMaxAxes + 1> axisLettersString{};
	size_t axisCount{7};

	std::array<float, kMaxAxes> userPositions{};
};

#endif
