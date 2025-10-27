#include <Platform/StringHandle.h>

StringHandle::StringHandle() noexcept : data_(nullptr) { }

StringHandle::StringHandle(const char* s) noexcept : data_(nullptr)
{
	Assign(s);
}

StringHandle::StringHandle(const char* s, std::size_t len) noexcept : data_(nullptr)
{
	Assign(s, len);
}

StringHandle::StringHandle(const StringHandle& other) noexcept : data_(other.data_)
{
	IncreaseRef();
}

StringHandle::StringHandle(StringHandle&& other) noexcept : data_(other.data_)
{
	other.data_ = nullptr;
}

StringHandle& StringHandle::operator=(const StringHandle& other) noexcept
{
	if (this != &other)
	{
		Delete();
		data_ = other.data_;
		IncreaseRef();
	}
	return *this;
}

StringHandle& StringHandle::operator=(StringHandle&& other) noexcept
{
	if (this != &other)
	{
		Delete();
		data_ = other.data_;
		other.data_ = nullptr;
	}
	return *this;
}

StringHandle::~StringHandle()
{
	Delete();
}

ReadLockedPointer<const char> StringHandle::Get() const noexcept
{
	return ReadLockedPointer<const char>(nullptr, (data_ != nullptr) ? data_->str.c_str() : nullptr);
}

void StringHandle::InternalAssign(const char* s, std::size_t len) noexcept
{
	Delete();
	if (s != nullptr)
	{
		data_ = new HostStringData;
		data_->str.assign(s, len);
	}
}
