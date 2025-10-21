#pragma once

#include <cstddef>
#include <cstdint>

#include <RepRapFirmware.h>
#include <GCodeResult.h>
#include <RTOSIface/RTOSIface.h>
#include <Tools/Filament.h>
#include <General/function_ref.h>

class FileStore;
class GCodeBuffer;
class OutputBuffer;
class StringRef;
class GCodeException;

enum class ToolState : uint8_t
{
	off,
	active,
	standby
};

constexpr uint8_t TFreeBit = 1u << 0;
constexpr uint8_t TPreBit = 1u << 1;
constexpr uint8_t TPostBit = 1u << 2;
constexpr uint8_t DefaultToolChangeParam = TFreeBit | TPreBit | TPostBit;

class Tool
{
public:
	static Tool* Create(
			unsigned int,
			const char*,
			int32_t*,
			size_t,
			int32_t*,
			size_t,
			AxesBitmap,
			AxesBitmap,
			AxesBitmap,
			FansBitmap,
			int,
			size_t,
			int8_t,
			const StringRef&) noexcept
	{
		return nullptr;
	}

	static void Delete(Tool* t) noexcept { delete t; }

	static AxesBitmap GetXAxes(const Tool*) noexcept { return AxesBitmap(); }
	static AxesBitmap GetYAxes(const Tool*) noexcept { return AxesBitmap(); }
	static AxesBitmap GetZAxes(const Tool*) noexcept { return AxesBitmap(); }
	static AxesBitmap GetAxisMapping(const Tool*, unsigned int) noexcept { return AxesBitmap(); }
	static float GetOffset(const Tool*, size_t) noexcept { return 0.0f; }
	static void FlagTemperatureFault(int8_t) noexcept {}
	static GCodeResult ClearTemperatureFault(int8_t, const StringRef&) noexcept { return GCodeResult::warningNotSupported; }
	static void AddTool(Tool*) noexcept {}
	static void DeleteTool(int) noexcept {}
	static uint16_t GetExtrudersInUse() noexcept { return 0; }
	static uint16_t GetToolHeatersInUse() noexcept { return 0; }
	static uint16_t GetNumToolsToReport() noexcept { return 0; }
	static Tool* GetToolList() noexcept { return nullptr; }
	static ReadLockedPointer<Tool> GetLockedTool(int) noexcept
	{
		return ReadLockedPointer<Tool>(toolListLock, nullptr);
	}
	static unsigned int GetNumberOfContiguousTools() noexcept { return 0; }
	static bool ExtruderMovementAllowed(const Tool* , bool, unsigned int) noexcept { return true; }
	static bool DisplayColdExtrusionWarnings() noexcept { return false; }
	static bool IsHeaterAssignedToTool(int8_t) noexcept { return false; }
	static GCodeResult SetAllToolsFirmwareRetraction(GCodeBuffer&, const StringRef&, OutputBuffer*&) THROWS(GCodeException)
	{
		return GCodeResult::warningNotSupported;
	}
	static void CheckZHopsValid(AxesBitmap) noexcept {}

	float GetOffset(size_t) const noexcept { return 0.0f; }
	void SetOffset(size_t, float, bool) noexcept {}
	AxesBitmap GetAxisOffsetsProbed() const noexcept { return AxesBitmap(); }
	size_t DriveCount() const noexcept { return 0; }
	int GetDrive(size_t) const noexcept { return -1; }
	bool CanDriveExtruder(bool) const noexcept { return true; }
	size_t HeaterCount() const noexcept { return 0; }
	int GetHeater(size_t) const noexcept { return -1; }
	int Number() const noexcept { return -1; }
	void DefineMix(const float*) noexcept {}
	const float* GetMix() const noexcept { return nullptr; }
	void PrintTool(const StringRef&) const noexcept {}
	AxesBitmap GetXAxisMap() const noexcept { return AxesBitmap(); }
	AxesBitmap GetYAxisMap() const noexcept { return AxesBitmap(); }
	AxesBitmap GetZAxisMap() const noexcept { return AxesBitmap(); }
	FansBitmap GetFanMapping() const noexcept { return FansBitmap(); }
	Filament* GetFilament() const noexcept { return nullptr; }
	const char* GetFilamentName() const noexcept { return ""; }
	Tool* Next() const noexcept { return nullptr; }
	ToolState GetState() const noexcept { return ToolState::off; }

	bool IsRetracted() const noexcept { return false; }
	float GetRetractLength() const noexcept { return 0.0f; }
	float GetConfiguredRetractHop() const noexcept { return 0.0f; }
	float GetActualZHop() const noexcept { return 0.0f; }
	void SetActualZHop(float) noexcept {}
	float GetRetractExtra() const noexcept { return 0.0f; }
	float GetRetractSpeed() const noexcept { return 0.0f; }
	float GetUnRetractSpeed() const noexcept { return 0.0f; }
	void SetRetracted(bool) noexcept {}
	int8_t GetSpindleNumber() const noexcept { return -1; }
	uint32_t GetSpindleRpm() const noexcept { return 0; }
	void SetSpindleRpm(uint32_t, bool) noexcept {}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteSettings(FileStore*, const StringRef&) const noexcept { return false; }
#endif

	float GetToolHeaterActiveTemperature(size_t) const noexcept { return 0.0f; }
	float GetToolHeaterStandbyTemperature(size_t) const noexcept { return 0.0f; }
	void SetToolHeaterActiveTemperature(size_t, float) THROWS(GCodeException) {}
	void SetToolHeaterStandbyTemperature(size_t, float) THROWS(GCodeException) {}

	GCodeResult SetFirmwareRetraction(GCodeBuffer&, const StringRef&, OutputBuffer*&) THROWS(GCodeException)
	{
		return GCodeResult::warningNotSupported;
	}
	GCodeResult GetSetFeedForward(GCodeBuffer&, const StringRef&) THROWS(GCodeException)
	{
		return GCodeResult::warningNotSupported;
	}

	bool HasTemperatureFault() const noexcept { return false; }

	void IterateExtruders(function_ref_noexcept<void(unsigned int) noexcept>) const noexcept {}
	void IterateHeaters(function_ref_noexcept<void(int) noexcept>) const noexcept {}
	bool UsesHeater(int8_t) const noexcept { return false; }

	void SetFansPwm(float) const noexcept {}
	void HeatersToOff() const noexcept {}
	void HeatersToActiveOrStandby(bool) const noexcept {}

	uint32_t GetFeedForwardAdvanceClocks() const noexcept { return 0; }
	void ApplyExtrusionFeedForward(float) const noexcept {}
	void StopExtrusionFeedForward() const noexcept {}

	void Activate() noexcept {}
	void Standby() noexcept {}
	void UpdateExtruderAndHeaterCount(uint16_t&, uint16_t&) const noexcept {}

	void ClearExtrusionPending() const noexcept {}

#if SUPPORT_ASYNC_MOVES && PREALLOCATE_TOOL_AXES
	AxesBitmap GetXYAxesAndExtruders() const noexcept { return AxesBitmap(); }
#endif

	static ReadWriteLock toolListLock;
};

inline ReadWriteLock Tool::toolListLock{};
