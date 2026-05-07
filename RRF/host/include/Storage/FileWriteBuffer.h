#pragma once

#include <array>
#include <cstddef>
#include <cstring>

// Host implementation of the firmware FileWriteBuffer abstraction.
// Provides a fixed-size scratch buffer so higher layers can keep the same call sites.

constexpr size_t FileWriteBufLen = 8192;
constexpr size_t NumFileWriteBuffers = 1;
constexpr size_t SbcFileWriteBufLen = 4096;

class FileWriteBuffer
{
public:
    explicit FileWriteBuffer(FileWriteBuffer* nextBuffer = nullptr) noexcept
        : next(nextBuffer), index(0)
    {
    }

    static void UsingSbcMode() noexcept
    {
    }

    FileWriteBuffer* Next() const noexcept
    {
        return next;
    }
    void SetNext(FileWriteBuffer* n) noexcept
    {
        next = n;
    }

    char* Data() noexcept
    {
        return buffer.data();
    }
    const char* Data() const noexcept
    {
        return buffer.data();
    }

    size_t BytesStored() const noexcept
    {
        return index;
    }
    size_t BytesLeft() const noexcept
    {
        return FileWriteBufLen - index;
    }

    size_t Store(const char* data, size_t length) noexcept
    {
        const size_t bytesToStore = (length < BytesLeft()) ? length : BytesLeft();
        if (bytesToStore != 0)
        {
            std::memcpy(buffer.data() + index, data, bytesToStore);
            index += bytesToStore;
        }
        return bytesToStore;
    }

    void DataTaken() noexcept
    {
        index = 0;
    }
    void DataStored(size_t numBytes) noexcept
    {
        index += numBytes;
    }

private:
    FileWriteBuffer* next;
    size_t index;
    std::array<char, FileWriteBufLen> buffer{};
};
