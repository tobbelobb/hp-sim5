#pragma once

#include <Platform/Heap.h.orig>

namespace Heap
{
	inline IndexSlot* AllocateHandle() noexcept { return nullptr; }
	inline StorageSpace* AllocateSpace(size_t) noexcept { return nullptr; }
	inline void CheckSlotGood(IndexSlot*) noexcept { }
	inline void DeleteSlot(IndexSlot*) noexcept { }
	inline void GarbageCollect() noexcept { }
	inline bool CheckIntegrity(const StringRef&) noexcept { return true; }
	inline void Diagnostics(const StringRef&, Platform&) noexcept { }
}
