#pragma once

#ifdef RRF_HOST_BUILD

#include <Platform/MessageType.h>
#include <General/StringRef.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

class OutputBuffer
{
public:
	explicit OutputBuffer(OutputBuffer* n = nullptr) noexcept : next(n) {}

	void Append(OutputBuffer* other) noexcept
	{
		if (other == nullptr)
		{
			return;
		}
		OutputBuffer* tail = this;
		while (tail->next != nullptr)
		{
			tail = tail->next;
		}
		tail->next = other;
	}

	OutputBuffer* Next() const noexcept { return next; }
	bool IsReferenced() const noexcept { return false; }
	bool HadOverflow() const noexcept { return false; }
	void IncreaseReferences(size_t) noexcept {}

	const char* Data() const noexcept { return storage.c_str(); }
	const char* UnreadData() const noexcept { return storage.c_str() + readPos; }
	size_t DataLength() const noexcept { return storage.size(); }
	size_t Length() const noexcept
	{
		size_t total = storage.size();
		for (OutputBuffer* cur = next; cur != nullptr; cur = cur->next)
		{
			total += cur->storage.size();
		}
		return total;
	}

	char operator[](size_t index) const noexcept { return storage[index]; }
	const char* Read(size_t len) noexcept
	{
		const size_t start = readPos;
		readPos = std::min(readPos + len, storage.size());
		return storage.c_str() + start;
	}
	void Taken(size_t len) noexcept { readPos = std::min(readPos + len, storage.size()); }
	size_t BytesLeft() const noexcept { return storage.size() - readPos; }

	uint32_t WhenQueued() const noexcept { return whenQueued; }
	void UpdateWhenQueued() noexcept { whenQueued = 0; }

	size_t vprintf(const char* fmt, va_list vargs) noexcept
	{
		char buffer[512];
		const int written = std::vsnprintf(buffer, sizeof(buffer), fmt, vargs);
		if (written > 0)
		{
			storage.append(buffer, static_cast<size_t>(written));
		}
		return storage.size();
	}

	size_t printf(const char* fmt, ...) noexcept __attribute__((format(printf, 2, 3)))
	{
		va_list vargs;
		va_start(vargs, fmt);
		const size_t result = vprintf(fmt, vargs);
		va_end(vargs);
		return result;
	}

	size_t vcatf(const char* fmt, va_list vargs) noexcept { return vprintf(fmt, vargs); }
	size_t catf(const char* fmt, ...) noexcept __attribute__((format(printf, 2, 3)))
	{
	va_list vargs;
	va_start(vargs, fmt);
	const size_t result = vprintf(fmt, vargs);
	va_end(vargs);
	return result;
}
	size_t lcatf(const char* fmt, ...) noexcept __attribute__((format(printf, 2, 3)))
	{
		va_list vargs;
		va_start(vargs, fmt);
		const size_t result = vprintf(fmt, vargs);
		va_end(vargs);
		return result;
	}

	size_t copy(char c) noexcept
	{
		storage.assign(1, c);
		readPos = 0;
		return storage.size();
	}
	size_t copy(const char* src) noexcept
	{
		storage.assign(src != nullptr ? src : "");
		readPos = 0;
		return storage.size();
	}
	size_t copy(const char* src, size_t len) noexcept
	{
		storage.assign((src != nullptr) ? src : "", len);
		readPos = 0;
		return storage.size();
	}

	size_t cat(char c) noexcept
	{
		storage.push_back(c);
		return storage.size();
	}
	size_t cat(const char* src) noexcept
	{
		if (src != nullptr)
		{
			storage.append(src);
		}
		return storage.size();
	}
	size_t lcat(const char* src) noexcept { return cat(src); }
	size_t cat(const char* src, size_t len) noexcept
	{
		if (src != nullptr)
		{
			storage.append(src, len);
		}
		return storage.size();
	}
	size_t lcat(const char* src, size_t len) noexcept { return cat(src, len); }
	size_t cat(StringRef& str) noexcept
	{
		storage.append(str.c_str());
		return storage.size();
	}

	size_t EncodeChar(char c) noexcept { return cat(c); }
	size_t EncodeReply(OutputBuffer* src) noexcept
	{
		if (src != nullptr)
		{
			storage.append(src->Data(), src->DataLength());
		}
		return storage.size();
	}

	uint32_t GetAge() const noexcept { return 0; }

	static void Init() noexcept {}
	static bool Allocate(OutputBuffer*& buf, bool = true) noexcept
	{
		buf = new OutputBuffer();
		return buf != nullptr;
	}
	static size_t GetBytesLeft(const OutputBuffer*) noexcept { return 1'000'000; }
	static size_t Truncate(OutputBuffer*, size_t) noexcept { return 0; }
	static OutputBuffer* Release(OutputBuffer* buf) noexcept
	{
		if (buf == nullptr)
		{
			return nullptr;
		}
		OutputBuffer* nextBuf = buf->next;
		delete buf;
		return nextBuf;
	}
	static void ReleaseAll(OutputBuffer*& buf) noexcept
	{
		while (buf != nullptr)
		{
			buf = Release(buf);
		}
	}
	static void ReleaseAll(OutputBuffer* volatile& buf) noexcept
	{
		OutputBuffer* current = const_cast<OutputBuffer*>(buf);
		while (current != nullptr)
		{
			current = Release(current);
		}
		buf = nullptr;
	}
	static void Diagnostics(const StringRef& reply) noexcept
	{
		reply.copy("OutputBuffer diagnostics unavailable on host build");
	}
	static unsigned int GetFreeBuffers() noexcept { return 1024; }

private:
	OutputBuffer* next{nullptr};
	std::string storage{};
	size_t readPos{0};
	uint32_t whenQueued{0};
};

class OutputStack
{
public:
	OutputStack() noexcept = default;
	bool IsEmpty() const noexcept { return buffers.empty(); }
	void Clear() noexcept { buffers.clear(); }
	bool Push(OutputBuffer* buffer, MessageType = NoDestinationMessage) noexcept
	{
		if (buffer == nullptr)
		{
			return false;
		}
		buffers.push_back(buffer);
		return true;
	}
	OutputBuffer* Pop() noexcept
	{
		if (buffers.empty())
		{
			return nullptr;
		}
		OutputBuffer* buf = buffers.back();
		buffers.pop_back();
		return buf;
	}
	OutputBuffer* GetFirstItem() const noexcept
	{
		return buffers.empty() ? nullptr : buffers.front();
	}
	MessageType GetFirstItemType() const noexcept { return NoDestinationMessage; }
	void ReleaseFirstItem() noexcept
	{
		if (buffers.empty())
		{
			return;
		}
		OutputBuffer::Release(buffers.front());
		buffers.erase(buffers.begin());
	}
	bool ApplyTimeout(uint32_t) noexcept { return false; }
	OutputBuffer* GetLastItem() const noexcept
	{
		return buffers.empty() ? nullptr : buffers.back();
	}
	MessageType GetLastItemType() const noexcept { return NoDestinationMessage; }

private:
	std::vector<OutputBuffer*> buffers{};
};

#endif
