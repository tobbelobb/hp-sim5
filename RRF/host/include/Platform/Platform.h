#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdarg>
#include <ctime> // For time_t and struct tm

// Required RRF headers
#include <Platform/MessageType.h>
#include <Storage/FileStore.h>
#include <General/StringRef.h>
#include <General/String.h>
#include <Platform/OutputMemory.h>
#include <Endstops/EndstopsManager.h>
#include <RTOSIface/RTOSIface.h>
#include <Tools/Spindle.h>
#include <Fans/FansManager.h>

// --- Forward declarations for all major modules Platform is expected to know about ---
// This is critical. Platform acts as a "service locator" for the rest of the firmware.
class RepRap;
class GCodes;
class Move;
class Heat;
class FansManager;
class Tool;

// A minimal enum to satisfy code that checks the board type.
enum class BoardType : uint8_t
{
	Host,
};

// Enumeration of error condition bits
enum class ErrorCode : uint32_t
{
	BadTemp = 1u << 0,
	BadMove = 1u << 1,
	OutputStarvation = 1u << 2,
	OutputStackOverflow = 1u << 3,
	HsmciTimeout = 1u << 4
};

// --- The main Platform class for the host build ---
class Platform
{
public:
	Platform() noexcept;

	// --- Lifecycle ---
	void Init() noexcept;
	void Spin() noexcept;
	void Exit() noexcept;

	// --- Simulation Timekeeping ---
	// The motion planner (DDA) depend on millis().
	uint32_t millis() const noexcept;
	uint32_t micros() const noexcept;

	// --- Real-Time Clock ---
	// The host system's clock will be used here.
	bool IsDateTimeSet() const noexcept { return true; }
	time_t GetDateTime() const noexcept;
	bool GetDateTime(struct tm& rslt) const noexcept;
	bool SetDateTime(time_t t) noexcept;

	void AppendUsbReply(OutputBuffer *buffer, bool rawMessage) noexcept {};
	void AppendAuxReply(size_t auxNumber, OutputBuffer *buf, bool rawMessage) noexcept {};
	void AppendAuxReply(size_t auxNumber, const char *_ecv_array msg, bool rawMessage) noexcept {};

	// --- Logging and Messaging ---
	void Message(MessageType type, const char* message) noexcept;
	void MessageF(MessageType type, const char* fmt, ...) noexcept __attribute__((format(printf, 3, 4)));
	void MessageV(MessageType type, const char* fmt, va_list vargs) noexcept;
	void Message(MessageType type, OutputBuffer* buffer) noexcept;
	void RawMessage(MessageType type, const char* message) noexcept;
	void DebugMessage(const char* fmt, va_list vargs) noexcept;
	void LogError(ErrorCode e) noexcept { }

	// --- Service Locators (CRITICAL) ---
	// Provide access to the other major components.
	RepRap& GetRepRap() const noexcept { return *reprap; }
	GCodes& GetGCodes() const noexcept { return *gCodes; }
	Move& GetMove() const noexcept { return *move; }
	Heat& GetHeat() const noexcept { return *heat; }
	FansManager& GetFansManager() const noexcept { return *fans; }
	EndstopsManager& GetEndstops() noexcept { return endstops; }

	// --- Filesystem Abstraction ---
	bool SysFileExists(const char* filename) const noexcept;
# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool DeleteSysFile(const char *_ecv_array filename) const noexcept;
# endif
	FileStore* OpenSysFile(const char* filename, OpenMode mode) const noexcept;
	bool MakeSysFileName(const StringRef& result, const char* filename) const noexcept;
	FileStore* OpenFile(const char* directory, const char* filename, OpenMode mode, uint32_t preAllocSize = 0) const noexcept;
	bool FileExists(const char* directory, const char* filename) const noexcept;

	static const char *_ecv_array GetGCodeDir() noexcept; 		// Where the gcodes are
	static const char *_ecv_array GetMacroDir() noexcept;		// Where the user-defined macros are
                                                          //
	ReadLockedPointer<const char> GetSysDir() const noexcept;
	void AppendSysDir(const StringRef& result) const noexcept;
	void SetSysDir(const char* path) noexcept;
	// We can fake the web directory easily since it's only used for path construction
	ReadLockedPointer<const char> GetWebDir() const noexcept;
	void AppendWebDir(const StringRef & path) const noexcept {  path.cat("www"); };


	// --- Stubbed Hardware/State Functions ---
	// These are called by G-code handlers and need to exist and return sensible values.
	void EmergencyStop() noexcept;
	bool GetAtxPowerState() const noexcept { return true; } // Pretend PSU is always on
	BoardType GetBoardType() const noexcept { return BoardType::Host; }
	const char* GetElectronicsString() const noexcept { return "RRF_Host"; }
#if HAS_VOLTAGE_MONITOR
	float GetCurrentPowerVoltage() const noexcept { return 24.0f; } // Prevent low-voltage warnings
#endif
	void Beep(unsigned int freq, unsigned int ms) noexcept {} // Do nothing for beeps
                                                            //
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WritePlatformParameters(FileStore *f, bool includingG31) const noexcept { return true; };
#endif
                                                            //
	bool IsOutputOnExtrudeActive() const noexcept { return true; }
	void ExtrudeOn() noexcept { }
	void ExtrudeOff() noexcept { }
	Spindle& AccessSpindle(size_t slot) noexcept { return spindles[slot]; }

	// --- Static debug members (copied from your version) ---
	static inline bool shouldTurnOffHeaters{false};
	static inline bool hasGenericDebug{false};
	static inline String<StringLength256> genericDebugBuffer;


private:
	// Pointers to the real high-level modules. These must be set during initialization.
	RepRap* reprap;
	GCodes* gCodes;
	Move* move;
	Heat* heat;
	FansManager* fans;

	// Fake hardware modules that Platform owns
	EndstopsManager endstops;

	// Our fake simulation clock
	uint64_t sim_micros;

	// Filesystem state
	mutable ReadWriteLock sysDirLock;
	String<MaxFilenameLength> sysDir;
	Spindle spindles[MaxSpindles];
};

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

// Where the gcodes are
inline const char *_ecv_array Platform::GetGCodeDir() noexcept
{
	return GCODE_DIR;
}

inline const char *_ecv_array Platform::GetMacroDir() noexcept
{
	return MACRO_DIR;
}

#endif

#endif
