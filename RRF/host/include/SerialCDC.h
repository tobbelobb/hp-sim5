#pragma once

#include "Stream.h"
#include "CoreIO.h"

#include <cstring>

class SerialCDC : public Stream
{
public:
	void begin(uint32_t = 0) noexcept {}
	void end() noexcept {}

	void Start() noexcept {}
	void Start(Pin) noexcept {}

	bool IsConnected() const noexcept { return true; }

	size_t canWrite() noexcept override { return 1024; }

	size_t print(const char* str) noexcept
	{
		return (str != nullptr) ? std::strlen(str) : 0;
	}

	size_t print(const char* data, size_t len) noexcept
	{
		return (data != nullptr) ? len : 0;
	}

	using Stream::write;

	size_t write(uint8_t) noexcept override { return 1; }
	size_t write(const uint8_t*, size_t len) noexcept override { return len; }
};
