#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <Storage/FileStore.h>
#include <General/StringFunctions.h>
#include <General/SafeVsnprintf.h>
#include <General/String.h>
#include <Platform/OutputMemory.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace
{
	std::mutex logMutex;

	constexpr const char* DefaultSysDir = "0:/sys/";
	constexpr const char* DefaultWebDir = "0:/www/";
	constexpr const char* DefaultGcodeDir = "0:/gcodes/";

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

Platform::Platform() noexcept
{
	sysDir.copy(DefaultSysDir);
}

void Platform::Init() noexcept {}

void Platform::Spin() noexcept {}

void Platform::Exit() noexcept {}

bool Platform::SysFileExists(const char* filename) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	return MakeSysFileName(fullPath.GetRef(), filename) && MassStorage::FileExists(fullPath.c_str());
#else
	(void)filename;
	return false;
#endif
}

FileStore* Platform::OpenSysFile(const char* filename, OpenMode mode) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	return MakeSysFileName(fullPath.GetRef(), filename) ? MassStorage::OpenFile(fullPath.c_str(), mode, 0) : nullptr;
#else
	(void)filename;
	(void)mode;
	return nullptr;
#endif
}

bool Platform::MakeSysFileName(const StringRef& result, const char* filename) const noexcept
{
#if HAS_MASS_STORAGE
	auto sysDirPtr = GetSysDir();
	return MassStorage::CombineName(result, sysDirPtr.Ptr(), filename);
#else
	(void)result;
	(void)filename;
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
	(void)directory;
	(void)filename;
	(void)mode;
	(void)preAllocSize;
	return nullptr;
#endif
}

bool Platform::FileExists(const char* folder, const char* filename) const noexcept
{
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fullPath;
	return MassStorage::CombineName(fullPath.GetRef(), folder, filename) && MassStorage::FileExists(fullPath.c_str());
#else
	(void)folder;
	(void)filename;
	return false;
#endif
}

void Platform::MessageV(MessageType type, const char* fmt, va_list vargs) noexcept
{
	char buffer[512];
	const int written = SafeVsnprintf(buffer, sizeof(buffer), fmt, vargs);
	if (written < 0)
	{
		return;
	}
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
    while (buffer != nullptr)
    {
        combined.append(buffer->Data(), buffer->DataLength());
        buffer = buffer->Next();
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

void Platform::SetSysDir(const char* path) noexcept
{
	WriteLocker locker(sysDirLock);
	if (path != nullptr)
	{
		sysDir.copy(path);
	}
	else
	{
		sysDir.copy(DefaultSysDir);
	}
}

void Platform::AppendSysDir(const StringRef& result) const noexcept
{
	auto sysDirPtr = GetSysDir();
	result.copy(sysDirPtr.Ptr());
}

ReadLockedPointer<const char> Platform::GetSysDir() const noexcept
{
	return ReadLockedPointer<const char>(sysDirLock, sysDir.c_str());
}

ReadLockedPointer<const char> Platform::GetWebDir() const noexcept
{
	return ReadLockedPointer<const char>(nullptr, DefaultWebDir);
}
