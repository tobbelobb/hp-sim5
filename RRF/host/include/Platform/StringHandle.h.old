#pragma once

#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>

#include <RTOSIface/RTOSIface.h>

class StringHandle
{
public:
	StringHandle() noexcept;
	explicit StringHandle(const char* s) noexcept;
	StringHandle(const char* s, std::size_t len) noexcept;
	StringHandle(const StringHandle& other) noexcept;
	StringHandle(StringHandle&& other) noexcept;
	StringHandle& operator=(const StringHandle& other) noexcept;
	StringHandle& operator=(StringHandle&& other) noexcept;
	~StringHandle();

	ReadLockedPointer<const char> Get() const noexcept;
	std::size_t GetLength() const noexcept { return data_ ? data_->str.size() : 0; }

	void Delete() noexcept
	{
		if (data_ != nullptr && data_->refCount.fetch_sub(1, std::memory_order_acq_rel) == 1)
		{
			delete data_;
		}
		data_ = nullptr;
	}

	const StringHandle& IncreaseRefCount() const noexcept
	{
		const_cast<StringHandle*>(this)->IncreaseRef();
		return *this;
	}

	bool IsNull() const noexcept { return data_ == nullptr; }

	void Assign(const char* s) noexcept { InternalAssign(s, (s != nullptr) ? std::strlen(s) : 0); }
	void Assign(const char* s, std::size_t len) noexcept { InternalAssign(s, len); }

protected:
	void InternalAssign(const char* s, std::size_t len) noexcept;

private:
	struct HostStringData
	{
		std::atomic<std::size_t> refCount{1};
		std::string str;
	};

	void IncreaseRef() noexcept
	{
		if (data_ != nullptr)
		{
			data_->refCount.fetch_add(1, std::memory_order_relaxed);
		}
	}

	mutable HostStringData* data_{nullptr};
};

class AutoStringHandle : public StringHandle
{
public:
	AutoStringHandle() noexcept = default;
	explicit AutoStringHandle(const char* s) noexcept : StringHandle(s) { }
	AutoStringHandle(const char* s, std::size_t len) noexcept : StringHandle(s, len) { }
	AutoStringHandle(const AutoStringHandle& other) noexcept = default;
	AutoStringHandle(AutoStringHandle&& other) noexcept = default;
	AutoStringHandle& operator=(const AutoStringHandle& other) noexcept = default;
	~AutoStringHandle() = default;
};
