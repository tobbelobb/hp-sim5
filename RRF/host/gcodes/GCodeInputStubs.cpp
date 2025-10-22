#include <GCodes/GCodeInput.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

bool StandardGCodeInput::FillBuffer(GCodeBuffer* gb) noexcept
{
	if (gb == nullptr)
	{
		return false;
	}
	const size_t bytes = BytesCached();
	for (size_t i = 0; i < bytes; ++i)
	{
		const char c = ReadByte();
		if (gb->Put(c))
		{
			return true;
		}
	}
	return false;
}

RegularGCodeInput::RegularGCodeInput() noexcept : state(GCodeInputState::idle), writingPointer(0), readingPointer(0)
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

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

void FileGCodeInput::Reset(const FileData& file) noexcept
{
	RegularGCodeInput::Reset();
	lastFileRead.CopyFrom(file);
}

size_t FileGCodeInput::FileBytesCached(const FileData&) const noexcept
{
	return BytesCached();
}

GCodeInputReadResult FileGCodeInput::ReadFromFile(FileData& file) noexcept
{
	if (!file.IsLive())
	{
		return GCodeInputReadResult::error;
	}
	return GCodeInputReadResult::noData;
}

#endif

#endif
