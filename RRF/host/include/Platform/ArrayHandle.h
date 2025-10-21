#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <ObjectModel/TypeCode.h>

class ExpressionValue;

class ArrayHandle
{
public:
	ArrayHandle() noexcept = default;
	ArrayHandle(const ArrayHandle& other) noexcept : data_(other.data_)
	{
		IncreaseRef();
	}

	ArrayHandle(ArrayHandle&& other) noexcept : data_(other.data_)
	{
		other.data_ = nullptr;
	}

	ArrayHandle& operator=(const ArrayHandle& other) noexcept
	{
		if (this != &other)
		{
			Delete();
			data_ = other.data_;
			IncreaseRef();
		}
		return *this;
	}

	ArrayHandle& operator=(ArrayHandle&& other) noexcept
	{
		if (this != &other)
		{
			Delete();
			data_ = other.data_;
			other.data_ = nullptr;
		}
		return *this;
	}

	~ArrayHandle()
	{
		Delete();
	}

	void Allocate(std::size_t numElements);
	void AssignElement(std::size_t index, ExpressionValue& val);
	void AssignIndexed(const ExpressionValue& ev, std::size_t numIndices, const uint32_t* indices);

	std::size_t GetNumElements() const noexcept;
	bool GetElement(std::size_t index, ExpressionValue& rslt) const noexcept;
	TypeCode GetElementType(std::size_t index) const noexcept;

	void Delete() noexcept;
	const ArrayHandle& IncreaseRefCount() const noexcept
	{
		const_cast<ArrayHandle*>(this)->IncreaseRef();
		return *this;
	}

	bool IsNull() const noexcept { return data_ == nullptr; }

private:
	struct HostArrayData
	{
		std::atomic<std::size_t> refCount{1};
		std::vector<ExpressionValue> values;
	};

	void EnsureStorage(std::size_t requiredSize);
	void IncreaseRef() noexcept
	{
		if (data_ != nullptr)
		{
			data_->refCount.fetch_add(1, std::memory_order_relaxed);
		}
	}

	HostArrayData* data_{nullptr};
};

class AutoArrayHandle final : public ArrayHandle
{
public:
	AutoArrayHandle() noexcept = default;
	AutoArrayHandle(const AutoArrayHandle&) noexcept = default;
	AutoArrayHandle(AutoArrayHandle&&) noexcept = default;
	AutoArrayHandle& operator=(const AutoArrayHandle&) noexcept = default;
	~AutoArrayHandle() = default;
};

