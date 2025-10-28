#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <GCodeResult.h>
#include <General/StringRef.h>
#include <ObjectModel/ObjectModel.h>
#include <RTOSIface/RTOSIface.h>
#include <GCodes/GCodeFileInfo.h>

#include <array>

class Platform;
class GCodes;
class GCodeBuffer;
class VariableSet;
class GlobalVariables;

enum PrintEstimationMethod
{
	filamentBased,
	fileBased,
	slicerBased
};

// Host-only stub for PrintMonitor. Tracks minimal metadata so higher layers can query print status.
class PrintMonitor : public ObjectModel
{
public:
	PrintMonitor(Platform& p, GCodes& gc) noexcept;

	void Spin() noexcept {}
	void Init() noexcept;

	bool IsPrinting() const noexcept { return isPrinting; }
	void StartingPrint(const char *_ecv_array name) noexcept;
	void StartedPrint() noexcept { isPrinting = true; }
	void StoppedPrint() noexcept;
	void SetLayerNumber(uint32_t layerNumber) noexcept { currentLayer = layerNumber; }
	void SetLayerZ(float) noexcept {}
	void LayerChange() noexcept {}
	float FractionOfFilePrinted() const noexcept { return fileProgress; }

	float EstimateTimeLeft(PrintEstimationMethod method) const noexcept;
	unsigned int GetCurrentLayer() const noexcept { return currentLayer; }
	float GetCurrentLayerTime() const noexcept { return 0.0f; }
	float GetPrintDuration() const noexcept { return printDuration; }
	float GetWarmUpDuration() const noexcept { return warmUpDuration; }
	float GetPauseDuration() const noexcept { return pauseDuration; }

	const char *_ecv_array _ecv_null GetPrintingFilename() const noexcept;
	bool GetPrintingFileInfo(GCodeFileInfo& info, const GlobalVariables *& vars) noexcept;
	void SetPrintingFileInfo(const char *_ecv_array filename, GCodeFileInfo& info) noexcept;

	GCodeResult ProcessM73(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException);
	void SetSlicerTimeLeft(float seconds) noexcept { slicerTimeLeft = seconds; }
	void SetSlicerTimeToPause(float seconds) noexcept { slicerTimeToPause = seconds; }

	ReadLockedPointer<const VariableSet> GetCustomInfoForReading() noexcept
	{
		return ReadLockedPointer<const VariableSet>(nullptr, nullptr);
	}

protected:
	const ObjectModelClassDescriptor *_ecv_null GetObjectModelClassDescriptor() const noexcept override
	{
		return nullptr;
	}

	const ObjectModelArrayTableEntry *_ecv_null GetObjectModelArrayEntry(unsigned int) const noexcept override
	{
		return nullptr;
	}

private:
	void Reset() noexcept;

	static constexpr const char *_ecv_array UnsupportedMessage = "Print monitoring not implemented on host build";

	Platform& platform;
	GCodes& gCodes;

	String<MaxFilenameLength> filenameBeingPrinted;
	GCodeFileInfo printingFileInfo{};

	float fileProgress{0.0f};
	float printDuration{0.0f};
	float warmUpDuration{0.0f};
	float pauseDuration{0.0f};
	float slicerTimeLeft{0.0f};
	float slicerTimeToPause{0.0f};
	float lastLayerHeight{0.0f};

	unsigned int currentLayer{0};
	bool hasPrintingFileInfo{false};
	bool isPrinting{false};
};

inline PrintMonitor::PrintMonitor(Platform& p, GCodes& gc) noexcept
	: platform(p), gCodes(gc)
{
	Init();
}

inline void PrintMonitor::Init() noexcept
{
	Reset();
}

inline void PrintMonitor::StartingPrint(const char *_ecv_array name) noexcept
{
	Reset();
	isPrinting = true;
	if (name != nullptr && *name != 0)
	{
		filenameBeingPrinted.copy(name);
	}
}

inline void PrintMonitor::StoppedPrint() noexcept
{
	isPrinting = false;
	fileProgress = 0.0f;
}

inline float PrintMonitor::EstimateTimeLeft(PrintEstimationMethod method) const noexcept
{
	if (method == slicerBased)
	{
		return slicerTimeLeft;
	}
	return 0.0f;
}

inline const char *_ecv_array _ecv_null PrintMonitor::GetPrintingFilename() const noexcept
{
	return (isPrinting && filenameBeingPrinted.strlen() > 0) ? filenameBeingPrinted.c_str() : nullptr;
}

inline bool PrintMonitor::GetPrintingFileInfo(GCodeFileInfo& info, const GlobalVariables *& vars) noexcept
{
	vars = nullptr;
	if (!hasPrintingFileInfo)
	{
		return false;
	}
	info = printingFileInfo;
	return true;
}

inline void PrintMonitor::SetPrintingFileInfo(const char *_ecv_array filename, GCodeFileInfo& info) noexcept
{
	printingFileInfo = info;
	hasPrintingFileInfo = true;
	if (filename != nullptr && *filename != 0)
	{
		filenameBeingPrinted.copy(filename);
	}
}

inline GCodeResult PrintMonitor::ProcessM73(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	reply.copy(UnsupportedMessage);
	return GCodeResult::warningNotSupported;
}

inline void PrintMonitor::Reset() noexcept
{
	isPrinting = false;
	hasPrintingFileInfo = false;
	fileProgress = 0.0f;
	printDuration = 0.0f;
	warmUpDuration = 0.0f;
	pauseDuration = 0.0f;
	slicerTimeLeft = 0.0f;
	slicerTimeToPause = 0.0f;
	currentLayer = 0;
	lastLayerHeight = 0.0f;
	filenameBeingPrinted.Clear();
}

#endif
