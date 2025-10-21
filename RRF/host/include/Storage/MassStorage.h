#pragma once

#include <RepRapFirmware.h>
#include <GCodeResult.h>
#include <GCodes/GCodeFileInfo.h>
#include <General/StringRef.h>
#include <General/String.h>
#include <Storage/FileStore.h>
#include <Storage/FileWriteBuffer.h>

#include <ctime>
#include <string>

class GlobalVariables;
class GCodeBuffer;
class GCodeException;
class ObjectModel;

struct FileInfo
{
	time_t lastModified;
	uint32_t size;
	String<MaxFilenameLength> fileName;
	bool isDirectory;
};

enum class ErrorMessageMode : uint8_t
{
	noMessage = 0,
	messageUnlessMissing,
	messageAlways
};

namespace MassStorage
{

	void SetHostRoot(const std::string& absolutePath) noexcept;
	const std::string& GetHostRoot() noexcept;

	bool CombineName(const StringRef& outbuf, const char* directory, const char* fileName) noexcept;
	const char* GetMonthName(uint8_t month) noexcept;

	void Init() noexcept;
	FileStore* OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept;
	bool FileExists(const char* filePath) noexcept;
	void CloseAllFiles() noexcept;
	void Spin() noexcept;
	size_t GetNumVolumes() noexcept;

	FileWriteBuffer* AllocateWriteBuffer() noexcept;
	size_t GetFileWriteBufferLength() noexcept;
	void ReleaseWriteBuffer(FileWriteBuffer* buffer) noexcept;
	bool Delete(const StringRef& filePath, ErrorMessageMode errorMessageMode, bool recursive = false) noexcept;

	bool DirectoryExists(const StringRef& path) noexcept;
	bool DirectoryExists(const char* path) noexcept;
	unsigned int GetNumFreeFiles() noexcept;
	bool IsDriveMounted(size_t drive) noexcept;
	bool FindFirst(const char* directory, FileInfo& file_info) noexcept;
	bool FindNext(FileInfo& file_info) noexcept;
	void AbandonFindNext() noexcept;
	GCodeResult GetFileInfo(const char* filePath, GCodeFileInfo& info, bool quitEarly, GlobalVariables* customVars) noexcept;
	GCodeResult Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept;
	GCodeResult Unmount(size_t card, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;

	bool EnsurePath(const char* filePath, bool messageIfFailed) noexcept;
	bool MakeDirectory(const char* directory, bool messageIfFailed) noexcept;
	bool Rename(const char* oldFilePath, const char* newFilePath, bool deleteExisting, bool messageIfFailed) noexcept;
	time_t GetLastModifiedTime(const char* filePath) noexcept;
	bool SetLastModifiedTime(const char* file, time_t t) noexcept;
	bool CheckDriveMounted(const char* path) noexcept;
	bool IsCardDetected(size_t card) noexcept;
	unsigned int InvalidateFiles(const void*) noexcept;
	bool AnyFileOpen(const void*) noexcept;
	void RecordSimulationTime(const char* printingFilePath, uint32_t simSeconds) noexcept;
	uint16_t GetVolumeSeq(unsigned int volume) noexcept;

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	struct SdCardReturnedInfo
	{
		uint64_t cardCapacity;
		uint64_t partitionSize;
		uint64_t freeSpace;
		uint32_t clSize;
		uint32_t speed;
	};

	InfoResult GetCardInfo(size_t slot, SdCardReturnedInfo& returnedInfo) noexcept;

	GCodeResult ConfigureSdCard(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	const ObjectModel* GetVolume(size_t vol) noexcept;
}
