#include <Platform/Heap.h>

#include <new>

void* CoreAllocPermanent(size_t sz, std::align_val_t align) noexcept
{
    return ::operator new(sz, align, std::nothrow);
}

namespace Heap
{
ReadWriteLock heapLock;
}
