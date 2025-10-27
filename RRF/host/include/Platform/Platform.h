#pragma once

#ifdef RRF_HOST_BUILD

#include <Platform/MessageType.h>
#include <Storage/FileStore.h>
#include <General/StringRef.h>
#include <General/String.h>
#include <Platform/OutputMemory.h>
#include <Endstops/EndstopsManager.h>

class RepRap;
extern RepRap reprap;

#include <cstdarg>

// Enumeration of error condition bits
enum class ErrorCode : uint32_t
{
	BadTemp = 1u << 0,
	BadMove = 1u << 1,
	OutputStarvation = 1u << 2,
	OutputStackOverflow = 1u << 3,
	HsmciTimeout = 1u << 4
};

// Minimal host representation of the Platform abstraction.  Enough of the
// firmware API is provided so that the G-code parser stack can call into
// logging and filesystem helpers without touching any MCU peripherals.
class Platform
{
public:
	Platform() noexcept;

	static inline bool shouldTurnOffHeaters{false};
	static inline bool hasGenericDebug{false};

	void Init() noexcept;
	void Spin() noexcept;
	void Exit() noexcept;

	void LogError(ErrorCode e) noexcept { }

	bool SysFileExists(const char* filename) const noexcept;
	FileStore* OpenSysFile(const char* filename, OpenMode mode) const noexcept;
	bool MakeSysFileName(const StringRef& result, const char* filename) const noexcept;
	FileStore* OpenFile(const char* directory, const char* filename, OpenMode mode, unsigned int preAllocSize = 0) const noexcept;
	bool FileExists(const char* directory, const char* filename) const noexcept;

	void Message(MessageType type, const char* message) noexcept;
	void MessageF(MessageType type, const char* fmt, ...) noexcept __attribute__((format(printf, 3, 4)));
	void MessageV(MessageType type, const char* fmt, va_list vargs) noexcept;
	void Message(MessageType type, OutputBuffer* buffer) noexcept;
	void RawMessage(MessageType type, const char* message) noexcept;
	void DebugMessage(const char* fmt, va_list vargs) noexcept;

	EndstopsManager& GetEndstops() noexcept { return endstops; }

	const String<MaxFilenameLength>& GetSysDir() const noexcept { return sysDir; }
	void SetSysDir(const char* path) noexcept;
	void AppendSysDir(const StringRef& result) const noexcept;

private:
	String<MaxFilenameLength> sysDir;
	EndstopsManager endstops;
};

#endif
