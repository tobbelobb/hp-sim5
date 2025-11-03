#pragma once

#include <cstddef>
#include <new>

// Host build shim: the embedded firmware uses a freelist-backed allocator for
// small objects.  For the desktop port we just defer to the default new/delete
// operators so code that relies on the macro keeps compiling.
#define DECLARE_FREELIST_NEW_DELETE(Type)           \
    static void* operator new(std::size_t sz)       \
    {                                               \
        return ::operator new(sz);                  \
    }                                               \
    static void operator delete(void* ptr) noexcept \
    {                                               \
        ::operator delete(ptr);                     \
    }
