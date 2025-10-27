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

	void LogError(ErrorCode e) noexcept { } // Called by real OutputMemory.cpp

	bool SysFileExists(const char* filename) const noexcept;
	FileStore* OpenSysFile(const char* filename, OpenMode mode) const noexcept;
	bool MakeSysFileName(const StringRef& result, const char* filename) const noexcept;
	void AppendWebDir(const StringRef & path) const noexcept {  path.cat("www"); };
	FileStore* OpenFile(const char* directory, const char* filename, OpenMode mode, unsigned int preAllocSize = 0) const noexcept;
	bool FileExists(const char* directory, const char* filename) const noexcept;

	void Message(MessageType type, const char* message) noexcept;
	void MessageF(MessageType type, const char* fmt, ...) noexcept __attribute__((format(printf, 3, 4)));
	void MessageV(MessageType type, const char* fmt, va_list vargs) noexcept;
	void Message(MessageType type, OutputBuffer* buffer) noexcept;
	void RawMessage(MessageType type, const char* message) noexcept;
	void DebugMessage(const char* fmt, va_list vargs) noexcept;

	EndstopsManager& GetEndstops() noexcept { return endstops; }

	ReadLockedPointer<const char> GetSysDir() const noexcept;	// where the system files are
	ReadLockedPointer<const char> GetWebDir() const noexcept;	// where the web files are
	void SetSysDir(const char* path) noexcept;
	void AppendSysDir(const StringRef& result) const noexcept;

private:
	EndstopsManager endstops;
};

#endif
