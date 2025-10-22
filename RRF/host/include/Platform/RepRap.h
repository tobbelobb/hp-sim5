#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <General/Bitmap.h>
#include <ObjectModel/GlobalVariables.h>
#include <ObjectModel/ObjectModel.h>
#include <GCodes/GCodeException.h>
#include <Platform/Platform.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Heating/Heat.h>
#include <Fans/FansManager.h>
#include <GCodes/GCodes.h>

#include <cstdint>

class GCodeBuffer;
class OutputBuffer;

using DebugFlags = Bitmap<uint16_t>;

// Minimal host façade of RepRap.  Only the methods exercised by the current
// G-code parsing stack are implemented; everything else is a no-op.
class RepRap
{
public:
	RepRap() noexcept;

	Platform& GetPlatform() noexcept { return platform; }
	const Platform& GetPlatform() const noexcept { return platform; }

	GCodes& GetGCodes() noexcept { return gCodes; }
	const GCodes& GetGCodes() const noexcept { return gCodes; }

	Heat& GetHeat() noexcept { return heat; }
	PrintMonitor& GetPrintMonitor() noexcept { return printMonitor; }
	FansManager& GetFansManager() noexcept { return fansManager; }

	bool UsingSbcInterface() const noexcept { return false; }

	DebugFlags GetDebugFlags(Module) const noexcept { return DebugFlags(); }
	bool Debug(Module) const noexcept { return false; }

	ExpressionValue GetObjectValueUsingTableNumber(ObjectExplorationContext& context,
												   const ObjectModelClassDescriptor* classDescriptor,
												   const char* idString,
												   uint8_t tableNumber) THROWS(GCodeException);

	void InputsUpdated() noexcept {}
	void GlobalUpdated() noexcept {}
	void MoveUpdated() noexcept {}

	ReadLockedPointer<const VariableSet> GetGlobalVariablesForReading() noexcept { return globalVariables.GetForReading(); }
	WriteLockedPointer<VariableSet> GetGlobalVariablesForWriting() noexcept { return globalVariables.GetForWriting(); }

private:
	Platform platform;
	GCodes gCodes;
	Heat heat;
	PrintMonitor printMonitor;
	FansManager fansManager;
	GlobalVariables globalVariables;
};

extern RepRap reprap;

#endif
