#pragma once

#include <cstddef>
#include <cstdint>

#include <RepRapFirmware.h>
#include <GCodeResult.h>
#include <RTOSIface/RTOSIface.h>

class StringRef;
class GCodeBuffer;

#if SUPPORT_REMOTE_COMMANDS
struct CanMessageCreateFilamentMonitor;
struct CanMessageDeleteFilamentMonitor;
struct CanMessageGeneric;
#endif

class FilamentMonitor
{
public:
	virtual ~FilamentMonitor() noexcept = default;
	FilamentMonitor(const FilamentMonitor&) = delete;
	FilamentMonitor& operator=(const FilamentMonitor&) = delete;

	static void InitStatic() noexcept {}
	static void Spin() noexcept {}
	static bool CheckDriveAssignments(const StringRef&) noexcept { return false; }
	static void Exit() noexcept {}

	static GCodeResult Configure(GCodeBuffer&, const StringRef&, unsigned int) noexcept
	{
		return GCodeResult::warningNotSupported;
	}

	static void AllDiagnostics(const StringRef&) noexcept {}
	static size_t GetNumMonitorsToReport() noexcept { return 0; }
	static FilamentMonitor* GetMonitorAlreadyLocked(size_t) noexcept { return nullptr; }

#if SUPPORT_REMOTE_COMMANDS
	static GCodeResult Create(const CanMessageCreateFilamentMonitor&, const StringRef&) noexcept
	{
		return GCodeResult::warningNotSupported;
	}

	static GCodeResult Delete(const CanMessageDeleteFilamentMonitor&, const StringRef&) noexcept
	{
		return GCodeResult::warningNotSupported;
	}

	static GCodeResult Configure(const CanMessageGeneric&, const StringRef&) noexcept
	{
		return GCodeResult::warningNotSupported;
	}

	static void DeleteAll() noexcept {}
#endif

	static ReadWriteLock filamentMonitorsLock;

protected:
	FilamentMonitor() noexcept = default;
};

inline ReadWriteLock FilamentMonitor::filamentMonitorsLock{};
