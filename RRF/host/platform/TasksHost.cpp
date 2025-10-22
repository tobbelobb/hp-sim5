#include <Platform/Tasks.h>

#include <new>

void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept
{
	try
	{
		return ::operator new(sz, align);
	}
	catch (...)
	{
		return nullptr;
	}
}

void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept
{
	::operator delete(ptr, align);
}

namespace Tasks
{
	void Diagnostics(const StringRef& reply) noexcept
	{
		reply.copy("Tasks diagnostics unavailable on host build");
	}

	TaskHandle GetMainTask() noexcept
	{
		return nullptr;
	}

	void TerminateMainTask() noexcept {}

	ptrdiff_t GetNeverUsedRam() noexcept
	{
		return 0;
	}

	void* AllocPermanent(size_t sz, std::align_val_t align) noexcept
	{
		try
		{
			return ::operator new(sz, align);
		}
		catch (...)
		{
			return nullptr;
		}
	}

	const char* GetHeapTop() noexcept
	{
		return nullptr;
	}

	Mutex* GetI2CMutex() noexcept
	{
		static Mutex mutex;
		return &mutex;
	}

	void* GetNVMBuffer(const uint32_t*) noexcept
	{
		return nullptr;
	}
}
