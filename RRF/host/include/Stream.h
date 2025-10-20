#pragma once

#include <cstddef>
#include <cstdint>

class Stream
{
public:
	virtual ~Stream() = default;
	virtual int available() noexcept { return 0; }
	virtual int read() noexcept { return -1; }
	virtual void flush() noexcept {}
	virtual size_t write(uint8_t) noexcept { return 0; }
	virtual size_t write(const uint8_t*, size_t) noexcept { return 0; }
	virtual size_t canWrite() noexcept { return 0; }
};

