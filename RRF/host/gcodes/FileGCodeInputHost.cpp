#include <GCodes/GCodeInput.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#include <algorithm>

#if HAS_MASS_STORAGE

bool StandardGCodeInput::FillBuffer(GCodeBuffer* gb) noexcept
{
	const size_t bytesToPass = BytesCached();
	for (size_t i = 0; i < bytesToPass; ++i)
	{
		const char c = ReadByte();
		if (gb->Put(c))
		{
			return true;
		}
	}
	return false;
}

RegularGCodeInput::RegularGCodeInput() noexcept
	: state(GCodeInputState::idle), writingPointer(0), readingPointer(0)
{
}

void RegularGCodeInput::Reset() noexcept
{
	state = GCodeInputState::idle;
	writingPointer = readingPointer = 0;
}

char RegularGCodeInput::ReadByte() noexcept
{
	char c = buffer[readingPointer++];
	if (readingPointer == GCodeInputBufferSize)
	{
		readingPointer = 0;
	}
	return c;
}

size_t RegularGCodeInput::BytesCached() const noexcept
{
	return (writingPointer - readingPointer) % GCodeInputBufferSize;
}

size_t RegularGCodeInput::BufferSpaceLeft() const noexcept
{
	return (readingPointer - writingPointer - 1u) % GCodeInputBufferSize;
}

namespace
{
constexpr size_t kReadChunk = 256;
}

void FileGCodeInput::Reset() noexcept
{
	lastFileRead.Close();
	RegularGCodeInput::Reset();
}

void FileGCodeInput::Reset(const FileData& file) noexcept
{
	if (lastFileRead == file)
	{
		Reset();
	}
}

size_t FileGCodeInput::FileBytesCached(const FileData& file) const noexcept
{
	return (lastFileRead == file) ? BytesCached() : 0;
}

GCodeInputReadResult FileGCodeInput::ReadFromFile(FileData& file) noexcept
{
	if (!file.IsLive())
	{
		return GCodeInputReadResult::error;
	}

	const size_t spaceLeft = BufferSpaceLeft();
	if (spaceLeft == 0)
	{
		return GCodeInputReadResult::haveData;
	}

	const size_t chunk = std::min(spaceLeft, kReadChunk);
	char temp[kReadChunk];
	const int readCount = file.Read(temp, chunk);
	if (readCount <= 0)
	{
		return GCodeInputReadResult::noData;
	}

	if (lastFileRead != file)
	{
		lastFileRead.Close();
		RegularGCodeInput::Reset();
		lastFileRead.CopyFrom(file);
	}

	for (int i = 0; i < readCount; ++i)
	{
		buffer[writingPointer] = temp[i];
		writingPointer = (writingPointer + 1) % GCodeInputBufferSize;
	}

	return BytesCached() > 0 ? GCodeInputReadResult::haveData : GCodeInputReadResult::noData;
}

#endif
