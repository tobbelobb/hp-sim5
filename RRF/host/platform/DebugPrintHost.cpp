#include <RepRapFirmware.h>

#include <cstdarg>
#include <cstdio>

extern "C" void debugPrintf(const char* fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	std::vfprintf(stderr, fmt, vargs);
	va_end(vargs);
}

