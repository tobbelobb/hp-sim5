#include <Storage/FileStore.h>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace fs = std::filesystem;

struct FileStore::Handle
{
    fs::path hostPath;
    OpenMode mode;
    std::fstream stream;
    FilePosition length{0};
    FilePosition position{0};
    bool writeMode{false};
    CRC32 crc{};

    Handle() noexcept = default;
};

FileStore::FileStore() noexcept
    : handle(),
      usageMode(FileUseMode::free),
      openCount(0),
      closeRequested(false),
      calcCrc(false),
      writeBuffer()
{
}

void FileStore::Init() noexcept
{
    handle.reset();
    usageMode = FileUseMode::free;
    openCount = 0;
    closeRequested = false;
    calcCrc = false;
    writeBuffer.reset();
}

bool FileStore::EnsureOpen() const noexcept
{
    return handle != nullptr && handle->stream.is_open();
}

bool FileStore::Open(const char* filePath, OpenMode mode,
                     uint32_t /*preAllocSize*/) noexcept
{
    Init();

    fs::path path(filePath);

    std::ios::openmode flags = std::ios::binary;
    bool writeMode = false;
    calcCrc = (mode == OpenMode::writeWithCrc);
    closeRequested = false;

    switch (mode)
    {
        case OpenMode::read:
            flags |= std::ios::in;
            usageMode = FileUseMode::readOnly;
            break;

        case OpenMode::write:
        case OpenMode::writeWithCrc:
            flags |= std::ios::out | std::ios::trunc;
            writeMode = true;
            usageMode = FileUseMode::readWrite;
            break;

        case OpenMode::append:
            flags |= std::ios::out | std::ios::app;
            writeMode = true;
            usageMode = FileUseMode::readWrite;
            break;
    }

    auto newHandle = std::make_shared<Handle>();
    newHandle->hostPath = path;
    newHandle->mode = mode;
    newHandle->writeMode = writeMode;
    newHandle->stream.open(path, flags);

    if (!newHandle->stream.is_open())
    {
        usageMode = FileUseMode::free;
        return false;
    }

    if (writeMode)
    {
        writeBuffer = std::make_unique<FileWriteBuffer>();
    }

    if (calcCrc)
    {
        newHandle->crc.Reset();
    }

    if (!writeMode)
    {
        newHandle->stream.seekg(0, std::ios::beg);
        newHandle->position = static_cast<FilePosition>(newHandle->stream.tellg());
        newHandle->stream.seekg(0, std::ios::end);
        newHandle->length = static_cast<FilePosition>(newHandle->stream.tellg());
        newHandle->stream.seekg(0, std::ios::beg);
    }
    else
    {
        newHandle->stream.seekp(0, (mode == OpenMode::append) ? std::ios::end
                                                              : std::ios::beg);
        newHandle->position = static_cast<FilePosition>(newHandle->stream.tellp());
        newHandle->length = newHandle->position;
    }

    handle = std::move(newHandle);
    openCount = 1;
    return true;
}

bool FileStore::Read(char& b) noexcept
{
    if (!EnsureOpen() || usageMode == FileUseMode::free)
    {
        return false;
    }

    handle->stream.read(&b, 1);
    if (!handle->stream)
    {
        return false;
    }

    handle->position = static_cast<FilePosition>(handle->stream.tellg());
    return true;
}

int FileStore::Read(char* buf, size_t nBytes) noexcept
{
    if (!EnsureOpen() || usageMode == FileUseMode::free || nBytes == 0)
    {
        return -1;
    }

    handle->stream.read(buf, static_cast<std::streamsize>(nBytes));
    const std::streamsize bytesRead = handle->stream.gcount();
    handle->position = static_cast<FilePosition>(handle->stream.tellg());

    return static_cast<int>(bytesRead);
}

int FileStore::ReadLine(char* buf, size_t nBytes) noexcept
{
    if (!EnsureOpen() || usageMode == FileUseMode::free || nBytes == 0)
    {
        return -1;
    }

    size_t count = 0;
    char ch = 0;
    while (count + 1 < nBytes && Read(ch))
    {
        if (ch == '\r')
        {
            continue;
        }
        if (ch == '\n')
        {
            break;
        }
        buf[count++] = ch;
    }

    buf[count] = '\0';
    return static_cast<int>(count);
}

bool FileStore::Close() noexcept
{
    if (!EnsureOpen())
    {
        Init();
        return true;
    }

    if (openCount > 1)
    {
        --openCount;
        return true;
    }

    handle->stream.close();
    Init();
    return true;
}

bool FileStore::Seek(FilePosition pos) noexcept
{
    if (!EnsureOpen())
    {
        return false;
    }

    if (handle->writeMode)
    {
        handle->stream.seekp(static_cast<std::streamoff>(pos), std::ios::beg);
        handle->position = static_cast<FilePosition>(handle->stream.tellp());
    }
    else
    {
        handle->stream.seekg(static_cast<std::streamoff>(pos), std::ios::beg);
        handle->position = static_cast<FilePosition>(handle->stream.tellg());
    }
    return true;
}

FilePosition FileStore::Length() const noexcept
{
    if (!EnsureOpen())
    {
        return 0;
    }

    if (!handle->writeMode)
    {
        return handle->length;
    }

    std::error_code ec;
    const auto size = fs::file_size(handle->hostPath, ec);
    return ec ? 0 : static_cast<FilePosition>(size);
}

FilePosition FileStore::Position() const noexcept
{
    if (!EnsureOpen())
    {
        return 0;
    }

    return handle->position;
}

void FileStore::Duplicate() noexcept
{
    ++openCount;
}

bool FileStore::CopyFrom(const FileStore* f) noexcept
{
    if (f == nullptr || !f->EnsureOpen() || f->usageMode != FileUseMode::readOnly)
    {
        return false;
    }

    Init();

    auto newHandle = std::make_shared<Handle>();
    newHandle->hostPath = f->handle->hostPath;
    newHandle->mode = f->handle->mode;
    newHandle->writeMode = false;
    newHandle->length = f->handle->length;
    newHandle->position = f->handle->position;
    newHandle->crc = f->handle->crc;

    newHandle->stream.open(newHandle->hostPath, std::ios::binary | std::ios::in);
    if (!newHandle->stream.is_open())
    {
        return false;
    }

    newHandle->stream.seekg(static_cast<std::streamoff>(newHandle->position),
                            std::ios::beg);
    if (!newHandle->stream)
    {
        Init();
        return false;
    }

    handle = std::move(newHandle);
    usageMode = FileUseMode::readOnly;
    openCount = 1;
    closeRequested = false;
    calcCrc = false;

    return true;
}

bool FileStore::Write(char b) noexcept
{
    return Write(&b, 1);
}

bool FileStore::Write(const char* s, size_t len) noexcept
{
    if (!EnsureOpen() || !handle->writeMode)
    {
        return false;
    }

    handle->stream.write(s, static_cast<std::streamsize>(len));
    if (!handle->stream)
    {
        return false;
    }

    if (calcCrc)
    {
        handle->crc.Update(s, len);
    }

    handle->position = static_cast<FilePosition>(handle->stream.tellp());
    handle->length = std::max(handle->length, handle->position);
    return true;
}

bool FileStore::Write(const char* s) noexcept
{
    return Write(s, std::strlen(s));
}

bool FileStore::Flush() noexcept
{
    if (!EnsureOpen())
    {
        return false;
    }

    handle->stream.flush();
    return static_cast<bool>(handle->stream);
}

bool FileStore::Truncate() noexcept
{
    if (!EnsureOpen())
    {
        return false;
    }

    std::error_code ec;
    fs::resize_file(handle->hostPath, handle->position, ec);
    return !ec;
}

uint32_t FileStore::GetCRC32() const noexcept
{
    return handle ? handle->crc.Get() : 0;
}
