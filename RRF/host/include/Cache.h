#pragma once

#include "CoreIO.h"

namespace Cache
{
inline void Init() noexcept
{
}
inline void Enable() noexcept
{
}
inline bool Disable() noexcept
{
    return false;
}
inline void Flush(const volatile void*, size_t) noexcept
{
}
inline void Invalidate(const volatile void*, size_t) noexcept
{
}
inline void FlushBeforeDMAReceive(const volatile void* start, size_t length) noexcept
{
    Flush(start, length);
}
inline void InvalidateAfterDMAReceive(const volatile void* start, size_t length) noexcept
{
    Invalidate(start, length);
}
inline void FlushBeforeDMASend(const volatile void* start, size_t length) noexcept
{
    Flush(start, length);
}
}  // namespace Cache

inline void CacheFlushBeforeDMAReceive(const volatile void* start, size_t length) noexcept
{
    Cache::Flush(start, length);
}

inline void CacheInvalidateAfterDMAReceive(const volatile void* start,
                                           size_t length) noexcept
{
    Cache::Invalidate(start, length);
}

inline void CacheFlushBeforeDMASend(const volatile void* start, size_t length) noexcept
{
    Cache::Flush(start, length);
}
