#pragma once

#include "Stream.h"

class SerialCDC : public Stream
{
public:
	void begin(uint32_t = 0) noexcept {}
	void end() noexcept {}
	using Stream::write;
};

