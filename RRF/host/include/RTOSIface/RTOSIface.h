#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

class TaskBase;
using TaskHandle = TaskBase*;

inline void EnableInterrupts() noexcept {}
inline void DisableInterrupts() noexcept {}

class Mutex
{
public:
	Mutex() noexcept = default;

	void Create(const char*) noexcept {}

	bool Take(uint32_t timeout = TimeoutUnlimited) noexcept
	{
		bool expected = false;
		if (timeout == 0)
		{
			return locked.compare_exchange_strong(expected, true, std::memory_order_acquire);
		}
		while (!locked.compare_exchange_strong(expected, true, std::memory_order_acquire))
		{
			expected = false;
		}
		return true;
	}

	bool Release() noexcept
	{
		locked.store(false, std::memory_order_release);
		return true;
	}

	TaskHandle GetHolder() const noexcept { return nullptr; }

	static constexpr uint32_t TimeoutUnlimited = 0xFFFFFFFFu;

private:
	std::atomic<bool> locked{false};
};

class MutexLocker
{
public:
	explicit MutexLocker(Mutex* m, uint32_t timeout = Mutex::TimeoutUnlimited) noexcept
	: handle(m)
	{
		if (handle != nullptr)
		{
			acquired = handle->Take(timeout);
		}
	}

	explicit MutexLocker(Mutex& m, uint32_t timeout = Mutex::TimeoutUnlimited) noexcept
	: MutexLocker(&m, timeout)
	{
	}

	~MutexLocker()
	{
		Release();
	}

	void Release() noexcept
	{
		if (handle != nullptr && acquired)
		{
			handle->Release();
			acquired = false;
		}
	}

	bool ReAcquire(uint32_t timeout = Mutex::TimeoutUnlimited) noexcept
	{
		if (handle == nullptr)
		{
			return true;
		}
		if (acquired)
		{
			return true;
		}
		acquired = handle->Take(timeout);
		return acquired;
	}

	bool IsAcquired() const noexcept { return acquired; }

private:
	Mutex* handle{nullptr};
	bool acquired{false};
};

class BinarySemaphore
{
public:
	BinarySemaphore() noexcept = default;

	bool Take(uint32_t timeout = TimeoutUnlimited) noexcept
	{
		return mutex.Take(timeout);
	}

	bool Give() noexcept
	{
		return mutex.Release();
	}

	static constexpr uint32_t TimeoutUnlimited = 0xFFFFFFFFu;

private:
	Mutex mutex;
};

class TaskBase
{
public:
	using TaskId = uint32_t;

	TaskBase() noexcept = default;
	~TaskBase() noexcept = default;

	TaskId GetTaskId() const noexcept { return taskId; }

	TaskHandle_t GetFreeRTOSHandle() noexcept
	{
		return reinterpret_cast<TaskHandle_t>(this);
	}

	void AddToList() noexcept
	{
		running = true;
		taskId = NextTaskId();
	}

	void TerminateAndUnlink() noexcept
	{
		running = false;
	}

	TaskBase* GetNext() noexcept { return nullptr; }

	void Suspend() noexcept {}
	void Resume() noexcept {}

	void SetPriority(unsigned int) noexcept {}

	static unsigned int GetCurrentTaskPriority() noexcept { return 0; }
	static void SetCurrentTaskPriority(unsigned int) noexcept {}

	bool IsRunning() const noexcept { return running; }

	static void GiveFromISR(TaskBase*, uint32_t) noexcept {}
	void GiveFromISR(uint32_t) noexcept {}
	void Give(uint32_t) noexcept {}

	static bool TakeIndexed(uint32_t, uint32_t = TimeoutUnlimited) noexcept { return true; }

	static uint32_t ClearNotifyCount(TaskBase*, uint32_t) noexcept { return 0; }
	static uint32_t ClearCurrentTaskNotifyCount(uint32_t) noexcept { return 0; }

	static TaskBase* GetCallerTaskHandle() noexcept { return nullptr; }
	static TaskId GetCallerTaskId() noexcept { return 0; }

	static void Yield() noexcept {}

	static TaskBase* GetTaskList() noexcept { return nullptr; }

	static const uint32_t* GetCurrentTaskStackBase() noexcept { return nullptr; }

	static constexpr uint32_t TimeoutUnlimited = 0xFFFFFFFFu;

private:
	static TaskId NextTaskId() noexcept
	{
		static std::atomic<TaskId> counter{1};
		return counter++;
	}

	TaskId taskId{0};
	bool running{false};
};

template<unsigned int StackWords> class Task : public TaskBase
{
public:
	void Create(TaskFunction_t,
				const char*,
				void*,
				unsigned int) noexcept
	{
		AddToList();
	}

	StaticTask_t* GetTaskMemory() noexcept { return &taskStorage; }
	uint32_t* GetStackBase() noexcept { return stack; }
	uint32_t GetStackSize() const noexcept { return StackWords; }

private:
	uint32_t stack[StackWords]{};
	StaticTask_t taskStorage{};
};

namespace RTOSIface
{
	inline TaskBase* GetCurrentTask() noexcept
	{
		return nullptr;
	}

	static inline thread_local unsigned int interruptCriticalSectionNesting = 0;

	inline void EnterInterruptCriticalSection() noexcept
	{
		++interruptCriticalSectionNesting;
	}

	inline bool LeaveInterruptCriticalSection() noexcept
	{
		if (interruptCriticalSectionNesting > 0)
		{
			--interruptCriticalSectionNesting;
		}
		return interruptCriticalSectionNesting == 0;
	}

	inline void EnterTaskCriticalSection() noexcept {}
	inline bool LeaveTaskCriticalSection() noexcept { return false; }
	inline void Yield() noexcept {}
}

class InterruptCriticalSectionLocker
{
public:
	InterruptCriticalSectionLocker() noexcept { RTOSIface::EnterInterruptCriticalSection(); }
	~InterruptCriticalSectionLocker() noexcept { (void)RTOSIface::LeaveInterruptCriticalSection(); }

	InterruptCriticalSectionLocker(const InterruptCriticalSectionLocker&) = delete;
	InterruptCriticalSectionLocker& operator=(const InterruptCriticalSectionLocker&) = delete;
};

class TaskCriticalSectionLocker
{
public:
	TaskCriticalSectionLocker() noexcept { RTOSIface::EnterTaskCriticalSection(); }
	~TaskCriticalSectionLocker() noexcept { RTOSIface::LeaveTaskCriticalSection(); }

	TaskCriticalSectionLocker(const TaskCriticalSectionLocker&) = delete;
	TaskCriticalSectionLocker& operator=(const TaskCriticalSectionLocker&) = delete;
};

class ConditionalTaskCriticalSectionLocker
{
public:
	explicit ConditionalTaskCriticalSectionLocker(bool doLock) noexcept : locked(doLock)
	{
		if (locked)
		{
			RTOSIface::EnterTaskCriticalSection();
		}
	}

	~ConditionalTaskCriticalSectionLocker() noexcept
	{
		if (locked)
		{
			RTOSIface::LeaveTaskCriticalSection();
		}
	}

private:
	bool locked;
};

class ReadWriteLock
{
public:
	ReadWriteLock() noexcept = default;

	void LockForReading() noexcept {}
	bool ConditionalLockForReading() noexcept { return true; }
	void ReleaseReader() noexcept {}
	void LockForWriting() noexcept {}
	bool ConditionalLockForWriting() noexcept { return true; }
	void ReleaseWriter() noexcept {}
	void DowngradeWriter() noexcept {}
	bool IsWriteLocked() const noexcept { return false; }
};

class ReadLocker
{
public:
	explicit ReadLocker(ReadWriteLock& lock) noexcept : lockPtr(&lock) { lockPtr->LockForReading(); }
	explicit ReadLocker(ReadWriteLock* lock) noexcept : lockPtr(lock)
	{
		if (lockPtr != nullptr)
		{
			lockPtr->LockForReading();
		}
	}
	ReadLocker(ReadLocker&& other) noexcept : lockPtr(other.lockPtr) { other.lockPtr = nullptr; }
	~ReadLocker()
	{
		if (lockPtr != nullptr)
		{
			lockPtr->ReleaseReader();
		}
	}

private:
	ReadWriteLock* lockPtr;
};

class WriteLocker
{
public:
	explicit WriteLocker(ReadWriteLock& lock) noexcept : lockPtr(&lock) { lockPtr->LockForWriting(); }
	explicit WriteLocker(ReadWriteLock* lock) noexcept : lockPtr(lock)
	{
		if (lockPtr != nullptr)
		{
			lockPtr->LockForWriting();
		}
	}
	WriteLocker(WriteLocker&& other) noexcept : lockPtr(other.lockPtr) { other.lockPtr = nullptr; }
	~WriteLocker()
	{
		if (lockPtr != nullptr)
		{
			lockPtr->ReleaseWriter();
		}
	}

	void Release() noexcept
	{
		if (lockPtr != nullptr)
		{
			lockPtr->ReleaseWriter();
			lockPtr = nullptr;
		}
	}

	void Downgrade() noexcept
	{
		if (lockPtr != nullptr)
		{
			lockPtr->DowngradeWriter();
		}
	}

private:
	ReadWriteLock* lockPtr;
};

template<class T> class WriteLockedPointer
{
public:
	WriteLockedPointer(ReadWriteLock&, T* ptr) noexcept : pointer(ptr) {}
	WriteLockedPointer(WriteLocker&, T* ptr) noexcept : pointer(ptr) {}
	WriteLockedPointer(std::nullptr_t, T* ptr) noexcept : pointer(ptr) {}
	WriteLockedPointer(WriteLockedPointer&& other) noexcept : pointer(other.pointer) { other.pointer = nullptr; }

	WriteLockedPointer& operator=(WriteLockedPointer&& other) noexcept
	{
		if (this != &other)
		{
			pointer = other.pointer;
			other.pointer = nullptr;
		}
		return *this;
	}

	~WriteLockedPointer() = default;

	T* operator->() const noexcept { return pointer; }
	T& operator*() const noexcept { return *pointer; }
	T* Get() const noexcept { return pointer; }
	explicit operator bool() const noexcept { return pointer != nullptr; }

private:
	T* pointer;
};

template<class T> class ReadLockedPointer
{
public:
	ReadLockedPointer(ReadWriteLock&, T* ptr) noexcept : pointer(ptr) {}
	ReadLockedPointer(ReadLocker&, T* ptr) noexcept : pointer(ptr) {}
	ReadLockedPointer(std::nullptr_t, T* ptr) noexcept : pointer(ptr) {}
	ReadLockedPointer(ReadLockedPointer&& other) noexcept : pointer(other.pointer) { other.pointer = nullptr; }

	ReadLockedPointer& operator=(ReadLockedPointer&& other) noexcept
	{
		if (this != &other)
		{
			pointer = other.pointer;
			other.pointer = nullptr;
		}
		return *this;
	}

	~ReadLockedPointer() = default;

	bool IsNull() const noexcept { return pointer == nullptr; }
	bool IsNotNull() const noexcept { return pointer != nullptr; }
	T* operator->() const noexcept { return pointer; }
	T& operator*() const noexcept { return *pointer; }
	T* Ptr() const noexcept { return pointer; }
	void Release() noexcept { pointer = nullptr; }

private:
	T* pointer;
};
