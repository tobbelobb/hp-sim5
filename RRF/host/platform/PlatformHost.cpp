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

Platform::Platform() noexcept
	: sysFolder(*this, sysDirLock, sysDir, DEFAULT_SYS_DIR),
	  webFolder(*this, webDirLock, webDir, DEFAULT_WEB_DIR)
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
}

void Platform::Exit() noexcept
{
	MessageF(GenericMessage, "Host Platform Shutdown.\n");
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
}

void Platform::DebugMessage(const char* fmt, va_list vargs) noexcept
{
	MessageV(GenericMessage, fmt, vargs);
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
