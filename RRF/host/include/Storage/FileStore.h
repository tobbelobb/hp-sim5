#pragma once

#ifndef RRF_HOST_BUILD
#error "Host FileStore header should only be used in host build"
#endif

#define FILESTORE_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <RepRapFirmware.h>
#include <Storage/CRC32.h>
#include <Storage/FileWriteBuffer.h>

class FileStore;
class FileWriteBuffer;

enum class OpenMode : uint8_t
{
    read,
    write,
    writeWithCrc,
    append
};

enum class FileUseMode : uint8_t
{
    free,
    readOnly,
    readWrite,
    invalidated
};

class FileStore
{
public:
    FileStore() noexcept;

    bool Open(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept;
    bool Read(char& b) noexcept;
    bool Read(uint8_t& b) noexcept;
    int Read(char* buf, size_t nBytes) noexcept;
    int Read(uint8_t* buf, size_t nBytes) noexcept;
    int ReadLine(char* buf, size_t nBytes) noexcept;
    bool Close() noexcept;
    bool ForceClose() noexcept
    {
        openCount = 1;
        return Close();
    }
    bool Seek(FilePosition pos) noexcept;
    FilePosition Length() const noexcept;
    bool IsCloseRequested() const noexcept
    {
        return closeRequested;
    }
    bool IsFree() const noexcept
    {
        return usageMode == FileUseMode::free;
    }
    bool IsOpen() const noexcept
    {
        return usageMode != FileUseMode::free && usageMode != FileUseMode::invalidated;
    }
    FilePosition Position() const noexcept;
    void Duplicate() noexcept;

    bool Write(char b) noexcept;
    bool Write(const char* s, size_t len) noexcept;
    bool Write(const uint8_t* s, size_t len) noexcept;
    bool Write(const char* s) noexcept;
    bool Flush() noexcept;
    bool Truncate() noexcept;
    uint32_t GetCRC32() const noexcept;

    FileWriteBuffer* GetWriteBuffer() const noexcept
    {
        return writeBuffer.get();
    }

private:
    struct Handle;
    std::shared_ptr<Handle> handle;

    void Init() noexcept;
    bool EnsureOpen() const noexcept;

    FileUseMode usageMode;
    unsigned int openCount;
    bool closeRequested;
    bool calcCrc;
    std::unique_ptr<FileWriteBuffer> writeBuffer;
};

// Inline wrappers
inline bool FileStore::Read(uint8_t& b) noexcept
{
    char ch;
    const bool ok = Read(ch);
    b = static_cast<uint8_t>(ch);
    return ok;
}

inline int FileStore::Read(uint8_t* buf, size_t nBytes) noexcept
{
    return Read(reinterpret_cast<char*>(buf), nBytes);
}

inline bool FileStore::Write(const uint8_t* s, size_t len) noexcept
{
    return Write(reinterpret_cast<const char*>(s), len);
}
