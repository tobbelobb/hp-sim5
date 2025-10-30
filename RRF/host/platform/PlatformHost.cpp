#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <Storage/FileStore.h>
#include <General/StringFunctions.h>
#include <General/SafeVsnprintf.h>
#include <Platform/OutputMemory.h>

// Include headers for the modules Platform needs to know about
#include <RepRap.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
#include <Heating/Heat.h>
#include <Fans/FansManager.h>

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>      // For exit()
#include <mutex>
#include <string>
#include <ctime>        // For time() and localtime_r()

namespace
{
	std::mutex logMutex;

	constexpr const char* PrefixForType(MessageType type) noexcept
	{
		if ((type & ErrorMessageFlag) != 0)
		{
			return "Error: ";
		}
		if ((type & WarningMessageFlag) != 0)
		{
			return "Warning: ";
		}
		return "";
	}

	void WriteLine(MessageType type, const char* message) noexcept
	{
		if (message == nullptr)
		{
			return;
		}

		FILE* out = ((type & ErrorMessageFlag) != 0) ? stderr : stdout;
		const char* prefix = PrefixForType(type);
		if (*prefix != '\0')
		{
			std::fputs(prefix, out);
		}
		std::fputs(message, out);
		const size_t len = std::strlen(message);
		if (len == 0 || message[len - 1] != '\n')
		{
			std::fputc('\n', out);
		}
		std::fflush(out);
	}
}

// --- Lifecycle and Initialization ---

namespace
{
	constexpr const char* MessageLogLevelNames[] = { "debug", "info", "warn", "off" };
	constexpr uint32_t LogEnabledThreshold = 3;
}

Platform::Platform() noexcept
	: objectModelProxy(*this),
	  sysFolder(*this, sysDirLock, sysDir, DEFAULT_SYS_DIR),
	  webFolder(*this, webDirLock, webDir, DEFAULT_WEB_DIR),
	  logLevelSetting(LogLevel::off),
	  logFile(nullptr),
	  logWriteInProgress(false)
{
	// Initialize the simulation clock
	sim_micros = 0;

	// Initialize all service locator pointers to null. They must be set by the simulator's main function.
	reprap = nullptr;
	gCodes = nullptr;
	move = nullptr;
	heat = nullptr;
	fans = nullptr;

	sysFolder.SetAbsolute(DEFAULT_SYS_DIR);
	webFolder.SetAbsolute(DEFAULT_WEB_DIR);
	logFileRrfPath.Clear();
	uniqueId.SetFromCurrentBoard();
}

// In your main() function, you will need to create the instances of all the major
// modules and then set the pointers in this Platform instance before calling Init().
void Platform::Init() noexcept
{
	// This is where board-level hardware would be initialized. For the host build,
	// we can leave it empty or add a log message.
	MessageF(GenericMessage, "Host Platform Initialized.\n");
}

void Platform::Spin() noexcept
{
	// This is the heartbeat of the simulation. Each call advances the fake clock.
	// The motion planner and other time-sensitive code depend on this.
	// Advancing by 1ms (1000us) per tick is a reasonable starting point.
	sim_micros += 1000;
	FlushLog();
}

void Platform::Exit() noexcept
{
	MessageF(GenericMessage, "Host Platform Shutdown.\n");
	StopLogging();
}

void Platform::EmergencyStop() noexcept
{
	std::lock_guard<std::mutex> lock(logMutex);
	std::fputs("\n!!! EMERGENCY STOP CALLED !!!\n", stderr);
	std::fflush(stderr);
	exit(1);
}

// --- Timekeeping ---

uint32_t Platform::millis() const noexcept
{
	return sim_micros / 1000;
}

uint32_t Platform::micros() const noexcept
{
	return sim_micros;
}

time_t Platform::GetDateTime() const noexcept
{
	return std::time(nullptr);
}

// This function fixes your original build error
bool Platform::GetDateTime(struct tm& rslt) const noexcept
{
	const time_t current_time = std::time(nullptr);
	// Use localtime_r for thread safety
	return (localtime_r(&current_time, &rslt) != nullptr);
}

bool Platform::SetDateTime(time_t t) noexcept
{
	// We can't (and shouldn't) set the host system clock.
	// We just return true to signal to the firmware that the command was "successful".
	(void)t; // suppress unused parameter warning
	return true;
}


// --- Filesystem ---

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
bool Platform::Delete(const char *_ecv_array folder, const char *_ecv_array filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MassStorage::CombineName(location.GetRef(), folder, filename) && MassStorage::Delete(location.GetRef(), ErrorMessageMode::messageUnlessMissing);
}

bool Platform::DeleteSysFile(const char *_ecv_array filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MakeSysFileName(location.GetRef(), filename) && MassStorage::Delete(location.GetRef(), ErrorMessageMode::messageUnlessMissing);
}
#endif

bool Platform::SysFileExists(const char* filename) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	return MakeSysFileName(fullPath.GetRef(), filename) && MassStorage::FileExists(fullPath.c_str());
#else
	return false;
#endif
}

FileStore* Platform::OpenSysFile(const char* filename, OpenMode mode) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	return MakeSysFileName(fullPath.GetRef(), filename) ? MassStorage::OpenFile(fullPath.c_str(), mode, 0) : nullptr;
#else
	return nullptr;
#endif
}

bool Platform::MakeSysFileName(const StringRef& result, const char* filename) const noexcept
{
#if HAS_MASS_STORAGE
	auto sysDirPtr = GetSysDir();
	return MassStorage::CombineName(result, sysDirPtr.Ptr(), filename);
#else
	return false;
#endif
}

FileStore* Platform::OpenFile(const char* directory, const char* filename, OpenMode mode, unsigned int preAllocSize) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	if (!MassStorage::CombineName(fullPath.GetRef(), directory, filename))
	{
		return nullptr;
	}
	return MassStorage::OpenFile(fullPath.c_str(), mode, preAllocSize);
#else
	return nullptr;
#endif
}

bool Platform::FileExists(const char* folder, const char* filename) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	return MassStorage::CombineName(fullPath.GetRef(), folder, filename) && MassStorage::FileExists(fullPath.c_str());
#else
	return false;
#endif
}

void Platform::SetSysDir(const char* path) noexcept
{
	sysFolder.SetAbsolute(path);
}



void Platform::MessageV(MessageType type, const char* fmt, va_list vargs) noexcept
{
	char buffer[512];
	SafeVsnprintf(buffer, sizeof(buffer), fmt, vargs);
	Message(type, buffer);
}

void Platform::MessageF(MessageType type, const char* fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	MessageV(type, fmt, vargs);
	va_end(vargs);
}

void Platform::Message(MessageType type, const char* message) noexcept
{
	if (message == nullptr)
	{
		return;
	}

	std::lock_guard<std::mutex> lock(logMutex);
	WriteLine(type, message);
	LogToFile(type, message);
}

void Platform::Message(MessageType type, OutputBuffer* buffer) noexcept
{
	if (buffer == nullptr)
	{
		return;
	}

	std::string combined;
	combined.reserve(FormatStringLength);
	for (const OutputBuffer *cur = buffer; cur != nullptr; cur = cur->Next())
	{
		combined.append(cur->Data(), cur->DataLength());
	}
	Message(type, combined.c_str());
}

void Platform::RawMessage(MessageType type, const char* message) noexcept
{
	std::lock_guard<std::mutex> lock(logMutex);
	WriteLine(type, message);
	LogToFile(type, message);
}

void Platform::DebugMessage(const char* fmt, va_list vargs) noexcept
{
	MessageV(GenericMessage, fmt, vargs);
}

bool Platform::Beep(unsigned int freq, unsigned int ms) noexcept
{
	(void)freq;
	(void)ms;
	return false;
}

bool Platform::FlushMessages() noexcept
{
	FlushLog();
	return false;
}

const IoPort& Platform::GetAtxPowerPort() const noexcept
{
	static IoPort dummyPort;
	return dummyPort;
}

const ObjectModelClassDescriptor *_ecv_null Platform::ObjectModelProxy::GetObjectModelClassDescriptor() const noexcept
{
	static constexpr uint8_t descriptor[] = { 0 };
	static const ObjectModelClassDescriptor descriptorWrapper{ nullptr, descriptor, nullptr };
	return &descriptorWrapper;
}

const char* Platform::GetLogLevel() const noexcept
{
	static const LogLevel off = LogLevel::off;
	return (IsLoggingActive()) ? logLevelSetting.ToString() : off.ToString();
}

const char* Platform::GetLogFileName() const noexcept
{
	return (IsLoggingActive() && !logFileRrfPath.IsEmpty()) ? logFileRrfPath.c_str() : nullptr;
}

void Platform::FlushLog() noexcept
{
	std::lock_guard<std::mutex> guard(loggingMutex);
	if (logFile != nullptr)
	{
		logFile->Flush();
	}
}

uint32_t Platform::ExtractMessageLogLevel(MessageType type) noexcept
{
	return (static_cast<uint32_t>(type) & LogLevelMask) >> LogLevelShift;
}

bool Platform::ShouldLog(uint32_t messageLogLevel) const noexcept
{
	if (!IsLoggingActive())
	{
		return false;
	}
	if (messageLogLevel >= 4u)
	{
		return false;
	}
	const uint32_t platformLevel = logLevelSetting.ToBaseType();
	if (platformLevel == LogLevel::off.ToBaseType())
	{
		return false;
	}
	return (messageLogLevel + platformLevel) >= LogEnabledThreshold;
}

void Platform::LogToFile(MessageType type, const char* message) noexcept
{
	if (message == nullptr)
	{
		return;
	}
	const size_t len = std::strlen(message);
	if (len == 0 || (len == 1 && message[0] == '\n'))
	{
		return;
	}
	const uint32_t messageLogLevel = ExtractMessageLogLevel(type);
	if (!ShouldLog(messageLogLevel))
	{
		return;
	}

	WriteLogEntry(messageLogLevel, message);
}

void Platform::WriteLogEntry(uint32_t messageLogLevel, const char* message) noexcept
{
	std::lock_guard<std::mutex> guard(loggingMutex);
	WriteLogEntryUnlocked(messageLogLevel, message);
}

void Platform::WriteLogEntryUnlocked(uint32_t messageLogLevel, const char* message) noexcept
{
	if (logFile == nullptr || message == nullptr || logWriteInProgress)
	{
		return;
	}

	const size_t len = std::strlen(message);
	if (len == 0 || (len == 1 && message[0] == '\n'))
	{
		return;
	}

	logWriteInProgress = true;

	String<StringLength50> prefix;
	const time_t currentTime = GetDateTime();
	if (currentTime == 0)
	{
		const uint32_t secondsSinceStart = millis() / 1000u;
		const unsigned hours = secondsSinceStart / 3600u;
		const unsigned minutes = (secondsSinceStart % 3600u) / 60u;
		const unsigned seconds = secondsSinceStart % 60u;
		prefix.printf("power up + %02u:%02u:%02u ", hours, minutes, seconds);
	}
	else
	{
		struct tm timeInfo;
		if (gmtime_r(&currentTime, &timeInfo) != nullptr)
		{
			prefix.printf("%04d-%02d-%02d %02d:%02d:%02d ",
				timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
				timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}
		else
		{
			prefix.copy("power up + unknown ");
		}
	}

	const char* levelName = (messageLogLevel < 4u) ? MessageLogLevelNames[messageLogLevel] : "off";
	prefix.catf("[%s] ", levelName);

	bool ok = logFile->Write(prefix.c_str(), prefix.strlen());
	if (ok)
	{
		ok = logFile->Write(message, len);
	}
	if (ok && message[len - 1] != '\n')
	{
		ok = logFile->Write('\n');
	}
	if (ok)
	{
		logFile->Flush();
	}
	else
	{
		logFile->Close();
		delete logFile;
		logFile = nullptr;
		logFileRrfPath.Clear();
	}

	logWriteInProgress = false;
}

GCodeResult Platform::StartLogging(const char* filename, const StringRef& reply) noexcept
{
	const char* requested = (filename != nullptr && filename[0] != 0) ? filename : DEFAULT_LOG_FILE;

	String<MaxFilenameLength> fullPath;
	auto sysDirPtr = GetSysDir();
	if (!CombineRrfPath(fullPath, sysDirPtr.Ptr(), requested))
	{
		reply.copy("Path name too long");
		return GCodeResult::error;
	}

	if (!MassStorage::EnsurePath(fullPath.c_str(), false))
	{
		reply.printf("Unable to prepare path \"%s\"", fullPath.c_str());
		return GCodeResult::error;
	}

	FileStore* newLogFile = MassStorage::OpenFile(fullPath.c_str(), OpenMode::append, 0);
	if (newLogFile == nullptr)
	{
		reply.printf("Unable to create or open file %s", fullPath.c_str());
		return GCodeResult::error;
	}

	{
		std::lock_guard<std::mutex> guard(loggingMutex);
		logFile = newLogFile;
		logFileRrfPath.copy(fullPath.c_str());
		logWriteInProgress = false;

		String<StringLength50> startMessage;
		startMessage.printf("Event logging started at level %s", logLevelSetting.ToString());
		WriteLogEntryUnlocked(1u, startMessage.c_str());

		String<StringLength50> firmwareInfo;
		firmwareInfo.printf("Running: %s (host build)", GetElectronicsString());
		WriteLogEntryUnlocked(1u, firmwareInfo.c_str());
	}

	if (reprap != nullptr)
	{
		reprap->StateUpdated();
	}

	return GCodeResult::ok;
}

void Platform::StopLogging() noexcept
{
	bool notify = false;
	{
		std::lock_guard<std::mutex> guard(loggingMutex);
		if (logFile != nullptr)
		{
			String<StringLength50> stopMessage;
			stopMessage.copy("Event logging stopped");
			WriteLogEntryUnlocked(1u, stopMessage.c_str());

			logFile->Flush();
			logFile->Close();
			delete logFile;
			logFile = nullptr;
			logWriteInProgress = false;
			logFileRrfPath.Clear();
			notify = true;
		}
		logLevelSetting = LogLevel::off;
	}

	if (notify && reprap != nullptr)
	{
		reprap->StateUpdated();
	}
}

GCodeResult Platform::ConfigureLogging(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (gb.Seen('S'))
	{
		const auto newLevelRaw = static_cast<LogLevel::RawType>(gb.GetLimitedUIValue('S', LogLevel::off, LogLevel::NumValues));
		const LogLevel newLevel(newLevelRaw);

		StopLogging();

		if (newLevel > LogLevel::off)
		{
			logLevelSetting = newLevel;
			String<MaxFilenameLength> filename;
			if (gb.Seen('P'))
			{
				gb.GetQuotedString(filename.GetRef());
			}
			else
			{
				filename.copy(DEFAULT_LOG_FILE);
			}

			const GCodeResult result = StartLogging(filename.c_str(), reply);
			if (result != GCodeResult::ok)
			{
				logLevelSetting = LogLevel::off;
				return result;
			}
		}
		else
		{
			logLevelSetting = LogLevel::off;
			if (reprap != nullptr)
			{
				reprap->StateUpdated();
			}
		}
	}
	else
	{
		if (!IsLoggingActive())
		{
			reply.copy("Event logging is disabled");
		}
		else
		{
			reply.printf("Event logging is enabled at log level %s", logLevelSetting.ToString());
		}
	}

	return GCodeResult::ok;
}

void Platform::NotifyDirectoriesChanged() noexcept
{
	if (reprap != nullptr)
	{
		reprap->DirectoriesUpdated();
	}
}

ConfigurableFolder::ConfigurableFolder(Platform& owner,
									   ReadWriteLock& lock,
									   String<MaxFilenameLength>& storage,
									   const char* defaultValue) noexcept
	: owner(owner),
	  lockRef(lock),
	  storageRef(storage),
	  defaultValue(defaultValue)
{
}

const char* ConfigurableFolder::GetUnlockedPointer() const noexcept
{
	return storageRef.c_str();
}

bool ConfigurableFolder::EnsureTrailingSlash(String<MaxFilenameLength>& path) const noexcept
{
	if (!path.EndsWith('/'))
	{
		if (path.cat('/'))
		{
			return false;
		}
	}
	return true;
}

void ConfigurableFolder::AssignLocked(const char* newPath, bool notifyChange) noexcept
{
	if (newPath == nullptr || *newPath == 0)
	{
		if (!storageRef.Equals(defaultValue))
		{
			(void)storageRef.copy(defaultValue);
			if (notifyChange)
			{
				owner.NotifyDirectoriesChanged();
			}
		}
		return;
	}

	if (storageRef.Equals(newPath))
	{
		return;
	}

	if (!storageRef.copy(newPath))
	{
		if (notifyChange)
		{
			owner.NotifyDirectoriesChanged();
		}
	}
}

ReadLockedPointer<const char> ConfigurableFolder::GetLockedPointer() const noexcept
{
	return ReadLockedPointer<const char>(lockRef, GetUnlockedPointer());
}

void ConfigurableFolder::AppendToString(const StringRef& path) const noexcept
{
	ReadLocker locker(lockRef);
	path.cat(GetUnlockedPointer());
}

static bool CombineRrfPath(String<MaxFilenameLength>& result, const char* base, const char* fragment) noexcept
{
	const char* frag = (fragment != nullptr) ? fragment : "";
	bool hadError = false;
	const bool isRelative = frag[0] != '/' && (std::strlen(frag) < 2 || !std::isdigit(static_cast<unsigned char>(frag[0])) || frag[1] != ':');

	if (base != nullptr && base[0] != 0 && isRelative)
	{
		hadError = result.copy(base);
		if (!hadError)
		{
			const size_t len = result.strlen();
			if (len != 0 && result[len - 1] != '/')
			{
				hadError = result.cat('/');
			}
		}
	}
	else
	{
		result.Clear();
	}

	if (!hadError)
	{
		hadError = result.cat(frag);
	}

	if (hadError)
	{
		result.copy("?????");
	}
	return !hadError;
}

GCodeResult ConfigurableFolder::Configure(const char* dir, const StringRef& reply) noexcept
{
	WriteLocker locker(lockRef);

	const char* base = GetUnlockedPointer();
	String<MaxFilenameLength> newDir;
	if (!CombineRrfPath(newDir, base, dir))
	{
		reply.copy("Path name too long");
		return GCodeResult::error;
	}

	if (!EnsureTrailingSlash(newDir))
	{
		reply.copy("Path name too long");
		return GCodeResult::error;
	}

	if (!MassStorage::DirectoryExists(newDir.c_str()))
	{
		reply.printf("Path \"%s\" not found", newDir.c_str());
		return GCodeResult::error;
	}

	AssignLocked(newDir.c_str(), true);
	return GCodeResult::ok;
}

void ConfigurableFolder::SetAbsolute(const char* absolutePath) noexcept
{
	WriteLocker locker(lockRef);

	if (absolutePath == nullptr || absolutePath[0] == 0)
	{
		AssignLocked(defaultValue, true);
		return;
	}

	String<MaxFilenameLength> newDir;
	if (newDir.copy(absolutePath))
	{
		AssignLocked(defaultValue, true);
		return;
	}
	if (!EnsureTrailingSlash(newDir))
	{
		AssignLocked(defaultValue, true);
		return;
	}

	AssignLocked(newDir.c_str(), true);
}
