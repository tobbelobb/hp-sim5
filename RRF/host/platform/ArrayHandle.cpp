#include <Platform/ArrayHandle.h>

#include <ObjectModel/ObjectModel.h>

void ArrayHandle::EnsureStorage(std::size_t requiredSize)
{
	if (data_ == nullptr)
	{
		data_ = new HostArrayData;
		data_->values.resize(requiredSize);
	}
	else if (data_->values.size() < requiredSize)
	{
		data_->values.resize(requiredSize);
	}
}

void ArrayHandle::Allocate(std::size_t numElements)
{
	Delete();
	if (numElements != 0)
	{
		data_ = new HostArrayData;
		data_->values.resize(numElements);
	}
}

void ArrayHandle::AssignElement(std::size_t index, ExpressionValue& val)
{
	EnsureStorage(index + 1);
	data_->values[index] = val;
}

void ArrayHandle::AssignIndexed(const ExpressionValue& ev, std::size_t numIndices, const uint32_t* indices)
{
	if (numIndices == 0 || indices == nullptr)
	{
		return;
	}
	ExpressionValue copy(ev);
	AssignElement(indices[0], copy);
}

std::size_t ArrayHandle::GetNumElements() const noexcept
{
	return (data_ != nullptr) ? data_->values.size() : 0;
}

bool ArrayHandle::GetElement(std::size_t index, ExpressionValue& rslt) const noexcept
{
	if (data_ == nullptr || index >= data_->values.size())
	{
		return false;
	}
	rslt = data_->values[index];
	return true;
}

TypeCode ArrayHandle::GetElementType(std::size_t index) const noexcept
{
	if (data_ == nullptr || index >= data_->values.size())
	{
		return TypeCode::None;
	}
	return data_->values[index].GetType();
}

void ArrayHandle::Delete() noexcept
{
	if (data_ != nullptr && data_->refCount.fetch_sub(1, std::memory_order_acq_rel) == 1)
	{
		delete data_;
	}
	data_ = nullptr;
}
