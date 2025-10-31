#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdarg>
#include <ctime> // For time_t and struct tm
#include <mutex>

// Required RRF headers
#include <Platform/MessageType.h>
#include <Storage/FileStore.h>
#include <General/StringRef.h>
#include <General/String.h>
#include <Config/Configuration.h>
#include <RRF3Common.h>
#include <Platform/OutputMemory.h>
#include <Platform/UniqueId.h>
#include <Endstops/EndstopsManager.h>
#include <RTOSIface/RTOSIface.h>
#include <Tools/Spindle.h>
#include <Fans/FansManager.h>
#include <GPIO/GpOutPort.h>
#include <GPIO/GpInPort.h>
#include <ObjectModel/ObjectModel.h>
#include <Hardware/IoPorts.h>

// --- Forward declarations for all major modules Platform is expected to know about ---
// This is critical. Platform acts as a "service locator" for the rest of the firmware.
class RepRap;
class GCodes;
class Move;
class Heat;
class FansManager;
class Tool;
class Platform;

class ConfigurableFolder
{
public:
	ConfigurableFolder(Platform& owner,
					   ReadWriteLock& lock,
					   String<MaxFilenameLength>& storage,
					   const char* defaultValue) noexcept;

	ReadLockedPointer<const char> GetLockedPointer() const noexcept;
	void AppendToString(const StringRef& path) const noexcept;
	GCodeResult Configure(const char* dir, const StringRef& reply) noexcept;
	void SetAbsolute(const char* absolutePath) noexcept;

private:
	Platform& owner;
	ReadWriteLock& lockRef;
	String<MaxFilenameLength>& storageRef;
	const char* defaultValue;

	const char* GetUnlockedPointer() const noexcept;
	bool EnsureTrailingSlash(String<MaxFilenameLength>& path) const noexcept;
	void AssignLocked(const char* newPath, bool notifyChange) noexcept;
};

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
class Platform final INHERIT_OBJECT_MODEL
{
public:
	Platform() noexcept;
	Platform(const Platform&) = delete;

	// --- Lifecycle ---
	void Init() noexcept;
	void Spin() noexcept;
	void Exit() noexcept;

	GCodeResult DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf, unsigned int d) { return GCodeResult::ok; };
	static constexpr unsigned int NumPlatformDiagnosticParts = 7;
	static bool SetDebugBufferSize(unsigned int) noexcept { return true; }

	// --- Simulation Timekeeping ---
	// The motion planner (DDA) depend on millis().
	uint32_t millis() const noexcept;
	uint32_t micros() const noexcept;
	void Tick() noexcept {}

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
	bool FlushMessages() noexcept;
	void LogError(ErrorCode e) noexcept { }

	// --- Service Locators ---
	// Provide access to the other major components.
	RepRap& GetRepRap() const noexcept { return *reprap; }
	GCodes& GetGCodes() const noexcept { return *gCodes; }
	Move& GetMove() const noexcept { return *move; }
	Heat& GetHeat() const noexcept { return *heat; }
	FansManager& GetFansManager() const noexcept { return *fans; }
	EndstopsManager& GetEndstops() noexcept { return endstops; }
	ReadLockedPointer<ZProbe> GetZProbeOrDefault(size_t probeNumber) noexcept { return ReadLockedPointer<ZProbe>(nullptr, nullptr); }

	// --- Filesystem Abstraction ---
	bool SysFileExists(const char* filename) const noexcept;
# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool Delete(const char *_ecv_array folder, const char *_ecv_array filename) const noexcept;
	bool DeleteSysFile(const char *_ecv_array filename) const noexcept;
# endif
	FileStore* OpenSysFile(const char* filename, OpenMode mode) const noexcept;
	bool MakeSysFileName(const StringRef& result, const char* filename) const noexcept;
	FileStore* OpenFile(const char* directory, const char* filename, OpenMode mode, uint32_t preAllocSize = 0) const noexcept;
	bool FileExists(const char* directory, const char* filename) const noexcept;

	static const char *_ecv_array GetGCodeDir() noexcept; 		// Where the gcodes are
	static const char *_ecv_array GetMacroDir() noexcept;		// Where the user-defined macros are
                                                          //
	ReadLockedPointer<const char> GetSysDir() const noexcept { return sysFolder.GetLockedPointer(); }
	void AppendSysDir(const StringRef& path) const noexcept { sysFolder.AppendToString(path); }
	void SetSysDir(const char* path) noexcept;
	GCodeResult SetSysDir(const char *_ecv_array dir, const StringRef& reply) noexcept { return sysFolder.Configure(dir, reply); }
	GCodeResult SetWebDir(const char *_ecv_array dir, const StringRef& reply) noexcept { return webFolder.Configure(dir, reply); }
	ReadLockedPointer<const char> GetWebDir() const noexcept { return webFolder.GetLockedPointer(); }
	void AppendWebDir(const StringRef& path) const noexcept { webFolder.AppendToString(path); }
	GCodeResult ConfigureLogging(GCodeBuffer& gb, const StringRef& reply) noexcept;
	void StopLogging() noexcept;
	const char* GetLogLevel() const noexcept;
	const char* GetLogFileName() const noexcept;


	// --- Stubbed Hardware/State Functions ---
	// These are called by G-code handlers and need to exist and return sensible values.
	void EmergencyStop() noexcept;
	bool GetAtxPowerState() const noexcept { return true; } // Pretend PSU is always on
	bool IsAtxPowerControlled() const noexcept { return false; }
	bool IsDeferredPowerDown() const noexcept { return false; }
	GCodeResult HandleM80(GCodeBuffer& gb, const StringRef& reply) { return GCodeResult::ok; };
	GCodeResult HandleM81(GCodeBuffer& gb, const StringRef& reply) { return GCodeResult::ok; };
	GCodeResult HandleM575(GCodeBuffer& gb, const StringRef& reply) { return GCodeResult::ok; };
	GCodeResult SendI2cOrModbus(GCodeBuffer& gb, const StringRef &reply) { return GCodeResult::ok; };			// Handle M260
	GCodeResult ReceiveI2cOrModbus(GCodeBuffer& gb, const StringRef &reply) { return GCodeResult::ok; };			// Handle M261
                                                          //
	// Hotend configuration
	float GetFilamentWidth() const noexcept { return filamentWidth; }
	void SetFilamentWidth(float width) noexcept { filamentWidth = width; }
                                                          //
	BoardType GetBoardType() const noexcept { return BoardType::Host; }
	const char* GetElectronicsString() const noexcept { return "RRF_Host"; }
#if HAS_VOLTAGE_MONITOR
	float GetCurrentPowerVoltage() const noexcept { return 24.0f; } // Prevent low-voltage warnings
#endif
	GCodeResult SetBuzzerPort(GCodeBuffer& gb, const StringRef& reply) { return GCodeResult::ok; }
	bool Beep(unsigned int freq, unsigned int ms) noexcept; // Do nothing for beeps
	bool IsChanEnabled(size_t chan) const noexcept { return false; }
	bool IsChanRaw(size_t chan) const noexcept { return false; }
	void PanelDueBeep(int freq, int ms) noexcept { }
	void SendPanelDueMessage(size_t chan, const char *_ecv_array msg) noexcept { (void)chan; (void)msg; }
	const IoPort& GetAtxPowerPort() const noexcept;
	size_t GetNumGpInputsToReport() const noexcept { return 0; }
	size_t GetNumGpOutputsToReport() const noexcept { return 0; }
	void ResetVoltageMonitors() noexcept { }
	void Diagnostics(unsigned int, const StringRef&) noexcept { }
	const UniqueId& GetUniqueId() const noexcept { return uniqueId; }
                                                            //
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WritePlatformParameters(FileStore *f, bool includingG31) const noexcept { return true; };
#endif
                                                            //
	bool IsOutputOnExtrudeActive() const noexcept { return true; }
	void ExtrudeOn() noexcept { }
	void ExtrudeOff() noexcept { }

	GCodeResult GetSetAncillaryPwm(GCodeBuffer& gb, const StringRef& reply) {  return GCodeResult::ok; }
	Spindle& AccessSpindle(size_t slot) noexcept { return spindles[slot]; }

	GCodeResult ConfigurePort(GCodeBuffer& gb, const StringRef& reply) {  return GCodeResult::ok; }

	GpOutputPort& stubbedGpoutPort;
	GpOutputPort& GetGpOutPort(size_t gpoutPortNumber) noexcept { return stubbedGpoutPort; }
	GpInputPort& stubbedGpinPort;
	const GpInputPort& GetGpInPort(size_t gpinPortNumber) const noexcept { return stubbedGpinPort; }

	// --- Static debug members (copied from your version) ---
	static inline bool shouldTurnOffHeaters{false};
	static inline bool hasGenericDebug{false};
	static inline String<StringLength256> genericDebugBuffer;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS
  // GetObjectModelClassDescriptor;
  // objectModelTable[]
  // objectModelTableDescriptor[]
  // objectModelClassDescriptor
  // objectModelArrayTable[]
  // GetObjectModelArrayEntry

// -- Explicit version, to make Platform concrete despite inheriting ObjectModel --
//protected:
//	// Minimal ObjectModel implementation so Platform is concrete
//	const ObjectModelClassDescriptor *_ecv_null GetObjectModelClassDescriptor() const noexcept override {
//		static const ObjectModelTableEntry table[] = { };
//		static const uint8_t descriptor[] = { 0 };
//		static const ObjectModelClassDescriptor cls{ table, descriptor, nullptr };
//		return &cls;
//	}
//
//	// Stub JSON reporting (empty object)
//	void ReportAsJson(OutputBuffer *buf,
//	                  ObjectExplorationContext& /*context*/,
//	                  const ObjectModelClassDescriptor * null /*classDescriptor*/,
//	                  uint8_t /*tableNumber*/,
//	                  const char *_ecv_array /*filter*/) const THROWS(GCodeException) override
//	{
//		buf->cat("{}");
//	}
//
//	// No arrays/locks/limits
//	const ObjectModelArrayTableEntry *_ecv_null GetObjectModelArrayEntry(unsigned int) const noexcept override { return nullptr; }
//	ReadWriteLock *_ecv_null GetObjectLock(unsigned int) const noexcept override { return nullptr; }
//	size_t GetMaxElementsToReturn(const ObjectModelArrayTableEntry*, const ObjectExplorationContext&) const noexcept override { return 0; }

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
	mutable ReadWriteLock webDirLock;
	String<MaxFilenameLength> sysDir;
	String<MaxFilenameLength> webDir;
	ConfigurableFolder sysFolder;
	ConfigurableFolder webFolder;
	Spindle spindles[MaxSpindles];
	LogLevel logLevelSetting;
	FileStore* logFile;
	bool logWriteInProgress;
	String<MaxFilenameLength> logFileRrfPath;
	mutable std::mutex loggingMutex;

  // Hotend state
	float filamentWidth;
	UniqueId uniqueId;

	friend class ConfigurableFolder;
	void NotifyDirectoriesChanged() noexcept;
	bool IsLoggingActive() const noexcept { return logFile != nullptr; }
	GCodeResult StartLogging(const char* filename, const StringRef& reply) noexcept;
	static uint32_t ExtractMessageLogLevel(MessageType type) noexcept;
	bool ShouldLog(uint32_t messageLogLevel) const noexcept;
	void LogToFile(MessageType type, const char* message) noexcept;
	void WriteLogEntry(uint32_t messageLogLevel, const char* message) noexcept;
	void WriteLogEntryUnlocked(uint32_t messageLogLevel, const char* message) noexcept;
	void FlushLog() noexcept;
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
