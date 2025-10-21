#include <Storage/MassStorage.h>

#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>
#include <cctype>
#include <Storage/CRC32.h>

namespace fs = std::filesystem;

namespace
{
	fs::path& RootPath() noexcept
	{
		static fs::path root = fs::absolute("host/vsd");
		return root;
	}

	std::string& RootString() noexcept
	{
		static std::string rootString = RootPath().string();
		return rootString;
	}

	std::mutex& FilesMutex() noexcept
	{
		static std::mutex mtx;
		return mtx;
	}

	std::vector<std::unique_ptr<FileStore>>& OpenFiles() noexcept
	{
		static std::vector<std::unique_ptr<FileStore>> files;
		return files;
	}

	std::vector<FileWriteBuffer*>& BufferPool() noexcept
	{
		static std::vector<FileWriteBuffer*> pool;
		return pool;
	}

	struct DirectoryIteratorState
	{
		fs::directory_iterator iter;
		fs::directory_iterator end;
		fs::path directory;
		bool active{false};
	};

	DirectoryIteratorState& DirState() noexcept
	{
		static DirectoryIteratorState state;
		return state;
	}

	constexpr const char* MonthNames[] =
	{
		"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"
	};

	bool ValidateVolumePrefix(std::string& path) noexcept
	{
		if (path.size() >= 3 && std::isdigit(static_cast<unsigned char>(path[0])) && path[1] == ':' && (path[2] == '/' || path[2] == '\\'))
		{
			if (path[0] != '0')
			{
				return false;
			}
			path.erase(0, 3);
		}
		return true;
	}

	bool ResolvePath(const char* rrfPath, fs::path& resolved) noexcept
	{
		fs::path root = RootPath();
		if (!rrfPath || rrfPath[0] == '\0')
		{
			resolved = root;
			return true;
		}

		std::string path(rrfPath);
		for (char& c : path)
		{
			if (c == '\\')
			{
				c = '/';
			}
		}

		if (!ValidateVolumePrefix(path))
		{
			return false;
		}

		while (!path.empty() && path.front() == '/')
		{
			path.erase(path.begin());
		}

		fs::path combined = root / fs::path(path);
		fs::path normalised = combined.lexically_normal();

		auto rootStr = root.string();
		auto normalisedStr = normalised.string();
		if (normalisedStr.size() >= rootStr.size())
		{
			if (normalisedStr.compare(0, rootStr.size(), rootStr) == 0)
			{
				resolved = normalised;
				return true;
			}
		}
		return false;
	}

	bool EnsureParentPath(const fs::path& p) noexcept
	{
		std::error_code ec;
		fs::create_directories(p.parent_path(), ec);
		return !ec;
	}

	time_t ToTimeT(const fs::file_time_type& ft) noexcept
	{
		using namespace std::chrono;
		const auto sctp = time_point_cast<system_clock::duration>(ft - fs::file_time_type::clock::now() + system_clock::now());
		return system_clock::to_time_t(sctp);
	}

	bool FillFileInfo(const fs::directory_entry& entry, MassStorage::FileInfo& info) noexcept
	{
		info.fileName.copy(entry.path().filename().string().c_str());
		info.isDirectory = entry.is_directory();
		std::error_code ec;
		info.size = entry.is_directory() ? 0 : static_cast<uint32_t>(entry.file_size(ec));
		if (ec)
		{
			info.size = 0;
		}
		info.lastModified = ToTimeT(entry.last_write_time(ec));
		return true;
	}

	FileStore* GetFreeFileStore() noexcept
	{
		auto& files = OpenFiles();
		for (auto& file : files)
		{
			if (file->IsFree())
			{
				return file.get();
			}
		}
		files.emplace_back(std::make_unique<FileStore>());
		return files.back().get();
	}

	bool EnsureRootSubdirs() noexcept
	{
		static constexpr const char* kSubDirs[] = { "sys", "gcodes", "firmware" };
		std::error_code ec;
		for (const char* sub : kSubDirs)
		{
			fs::create_directories(RootPath() / sub, ec);
			if (ec)
			{
				return false;
			}
		}
		return true;
	}
}

void MassStorage::SetHostRoot(const std::string& absolutePath) noexcept
{
	std::lock_guard<std::mutex> lock(FilesMutex());
	fs::path newRoot = fs::absolute(absolutePath);
	RootPath() = newRoot;
	RootString() = RootPath().string();
	EnsureRootSubdirs();
}

const std::string& MassStorage::GetHostRoot() noexcept
{
	return RootString();
}

bool MassStorage::CombineName(const StringRef& outbuf, const char* directory, const char* fileName) noexcept
{
	fs::path base;
	if (!ResolvePath(directory, base))
	{
		return false;
	}

	fs::path combined = base / (fileName ? fileName : "");
	combined = combined.lexically_normal();
	outbuf.copy(combined.string().c_str());
	return true;
}

const char* MassStorage::GetMonthName(uint8_t month) noexcept
{
	return (month >= 1 && month <= 12) ? MonthNames[month - 1] : "";
}

void MassStorage::Init() noexcept
{
	std::lock_guard<std::mutex> lock(FilesMutex());
	EnsureRootSubdirs();
}

FileStore* MassStorage::OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	std::lock_guard<std::mutex> lock(FilesMutex());

	fs::path hostPath;
	if (!ResolvePath(filePath, hostPath))
	{
		return nullptr;
	}

	if ((mode == OpenMode::write || mode == OpenMode::writeWithCrc || mode == OpenMode::append) && !EnsureParentPath(hostPath))
	{
		return nullptr;
	}

	FileStore* store = GetFreeFileStore();
	if (store != nullptr && store->Open(hostPath.string().c_str(), mode, preAllocSize))
	{
		return store;
	}
	return nullptr;
}

bool MassStorage::FileExists(const char* filePath) noexcept
{
	fs::path hostPath;
	return ResolvePath(filePath, hostPath) && fs::exists(hostPath);
}

void MassStorage::CloseAllFiles() noexcept
{
	std::lock_guard<std::mutex> lock(FilesMutex());
	for (auto& file : OpenFiles())
	{
		file->ForceClose();
	}
}

void MassStorage::Spin() noexcept
{
}

size_t MassStorage::GetNumVolumes() noexcept
{
	return 1;
}

FileWriteBuffer* MassStorage::AllocateWriteBuffer() noexcept
{
	std::lock_guard<std::mutex> lock(FilesMutex());
	auto& pool = BufferPool();
	if (!pool.empty())
	{
		FileWriteBuffer* buf = pool.back();
		pool.pop_back();
		return buf;
	}
	return new FileWriteBuffer();
}

size_t MassStorage::GetFileWriteBufferLength() noexcept
{
	return HostStorage::FileWriteBufLen;
}

void MassStorage::ReleaseWriteBuffer(FileWriteBuffer* buffer) noexcept
{
	if (buffer == nullptr)
	{
		return;
	}
	std::lock_guard<std::mutex> lock(FilesMutex());
	BufferPool().push_back(buffer);
}

bool MassStorage::Delete(const StringRef& filePath, ErrorMessageMode errorMessageMode, bool recursive) noexcept
{
	(void)errorMessageMode;
	fs::path hostPath;
	if (!ResolvePath(filePath.c_str(), hostPath))
	{
		return false;
	}
	std::error_code ec;
	if (recursive)
	{
		fs::remove_all(hostPath, ec);
	}
	else
	{
		fs::remove(hostPath, ec);
	}
	return !ec;
}

bool MassStorage::DirectoryExists(const StringRef& path) noexcept
{
	return DirectoryExists(path.c_str());
}

bool MassStorage::DirectoryExists(const char* path) noexcept
{
	fs::path hostPath;
	return ResolvePath(path, hostPath) && fs::is_directory(hostPath);
}

unsigned int MassStorage::GetNumFreeFiles() noexcept
{
	return 1024;
}

bool MassStorage::IsDriveMounted(size_t) noexcept
{
	return true;
}

bool MassStorage::FindFirst(const char* directory, FileInfo& file_info) noexcept
{
	DirectoryIteratorState& state = DirState();
	fs::path hostPath;
	if (!ResolvePath(directory, hostPath))
	{
		state.active = false;
		return false;
	}

	std::error_code ec;
	state.iter = fs::directory_iterator(hostPath, ec);
	state.end = fs::directory_iterator();
	state.directory = hostPath;
	state.active = !ec;

	if (!state.active || state.iter == state.end)
	{
		state.active = false;
		return false;
	}

	FillFileInfo(*state.iter, file_info);
	return true;
}

bool MassStorage::FindNext(FileInfo& file_info) noexcept
{
	DirectoryIteratorState& state = DirState();
	if (!state.active)
	{
		return false;
	}

	++state.iter;
	if (state.iter == state.end)
	{
		state.active = false;
		return false;
	}

	FillFileInfo(*state.iter, file_info);
	return true;
}

void MassStorage::AbandonFindNext() noexcept
{
	DirState().active = false;
}

GCodeResult MassStorage::GetFileInfo(const char* filePath, GCodeFileInfo& info, bool quitEarly, GlobalVariables* customVars) noexcept
{
	(void)customVars;
	info.Init();

	fs::path hostPath;
	if (!ResolvePath(filePath, hostPath) || !fs::exists(hostPath))
	{
		return GCodeResult::warning;
	}

	std::error_code ec;
	info.fileSize = static_cast<FilePosition>(fs::file_size(hostPath, ec));
	if (ec)
	{
		return GCodeResult::warning;
	}

	info.lastModifiedTime = ToTimeT(fs::last_write_time(hostPath, ec));
	info.isValid = true;
	info.incomplete = false;
	info.generatedBy.copy("RRF host");

	if (!quitEarly)
	{
		std::ifstream in(hostPath, std::ios::binary);
		if (in)
		{
			std::vector<char> buffer(8192);
			CRC32 crc;
			while (in)
			{
				in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
				const size_t bytesRead = static_cast<size_t>(in.gcount());
				if (bytesRead == 0)
				{
					break;
				}
				crc.Update(buffer.data(), bytesRead);
			}
			info.generatedBy.catf(" crc:%08X", crc.Get());
		}
	}

	return GCodeResult::ok;
}

GCodeResult MassStorage::Mount(size_t, const StringRef& reply, bool reportSuccess) noexcept
{
	if (!EnsureRootSubdirs())
	{
		if (reportSuccess)
		{
			reply.cat("Failed to prepare host VSD");
		}
		return GCodeResult::error;
	}

	if (reportSuccess)
	{
		reply.cat("Virtual SD mounted");
	}
	return GCodeResult::ok;
}

GCodeResult MassStorage::Unmount(size_t, const StringRef& reply) noexcept
{
	reply.cat("Virtual SD unmounted");
	return GCodeResult::ok;
}

void MassStorage::Diagnostics(const StringRef& reply) noexcept
{
	reply.catf("Host VSD root: %s", RootString().c_str());
}

bool MassStorage::EnsurePath(const char* filePath, bool messageIfFailed) noexcept
{
	(void)messageIfFailed;
	fs::path hostPath;
	if (!ResolvePath(filePath, hostPath))
	{
		return false;
	}
	return EnsureParentPath(hostPath);
}

bool MassStorage::MakeDirectory(const char* directory, bool messageIfFailed) noexcept
{
	(void)messageIfFailed;
	fs::path hostPath;
	if (!ResolvePath(directory, hostPath))
	{
		return false;
	}
	std::error_code ec;
	fs::create_directories(hostPath, ec);
	return !ec;
}

bool MassStorage::Rename(const char* oldFilePath, const char* newFilePath, bool deleteExisting, bool messageIfFailed) noexcept
{
	(void)messageIfFailed;
	fs::path oldPath;
	fs::path newPath;
	if (!ResolvePath(oldFilePath, oldPath) || !ResolvePath(newFilePath, newPath))
	{
		return false;
	}

	std::error_code ec;
	if (deleteExisting && fs::exists(newPath))
	{
		fs::remove(newPath, ec);
		if (ec)
		{
			return false;
		}
	}

	fs::rename(oldPath, newPath, ec);
	return !ec;
}

time_t MassStorage::GetLastModifiedTime(const char* filePath) noexcept
{
	fs::path hostPath;
	if (!ResolvePath(filePath, hostPath))
	{
		return 0;
	}
	std::error_code ec;
	return ToTimeT(fs::last_write_time(hostPath, ec));
}

bool MassStorage::SetLastModifiedTime(const char* file, time_t t) noexcept
{
	fs::path hostPath;
	if (!ResolvePath(file, hostPath))
	{
		return false;
	}
	std::error_code ec;
	const auto tp = std::chrono::system_clock::from_time_t(t);
	const auto ftime = fs::file_time_type::clock::now() + (tp - std::chrono::system_clock::now());
	fs::last_write_time(hostPath, ftime, ec);
	return !ec;
}

bool MassStorage::CheckDriveMounted(const char* path) noexcept
{
	(void)path;
	return true;
}

bool MassStorage::IsCardDetected(size_t card) noexcept
{
	(void)card;
	return true;
}

unsigned int MassStorage::InvalidateFiles(const void*) noexcept
{
	return 0;
}

bool MassStorage::AnyFileOpen(const void*) noexcept
{
	return false;
}

void MassStorage::RecordSimulationTime(const char* printingFilePath, uint32_t simSeconds) noexcept
{
	fs::path hostPath;
	if (!ResolvePath(printingFilePath, hostPath))
	{
		return;
	}

	std::ofstream out(hostPath, std::ios::app);
	if (!out)
	{
		return;
	}

	out << "\n; Simulated printing time (s): " << simSeconds << '\n';
}

uint16_t MassStorage::GetVolumeSeq(unsigned int volume) noexcept
{
	(void)volume;
	return 1;
}

MassStorage::InfoResult MassStorage::GetCardInfo(size_t, SdCardReturnedInfo& returnedInfo) noexcept
{
	std::error_code ec;
	const auto capacity = fs::space(RootPath(), ec);
	if (ec)
	{
		return InfoResult::noCard;
	}

	returnedInfo.cardCapacity = capacity.capacity;
	returnedInfo.partitionSize = capacity.capacity;
	returnedInfo.freeSpace = capacity.available;
	returnedInfo.clSize = 4096;
	returnedInfo.speed = 0;
	return InfoResult::ok;
}

GCodeResult MassStorage::ConfigureSdCard(GCodeBuffer& gb, const StringRef& reply) noexcept(false)
{
	(void)gb;
	reply.cat("Configuring SD slots not supported in host build");
	return GCodeResult::warningNotSupported;
}

const ObjectModel* MassStorage::GetVolume(size_t) noexcept
{
	return nullptr;
}
