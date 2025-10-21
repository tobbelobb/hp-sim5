#include <ObjectModel/ObjectModel.h>

#include <General/StringRef.h>
#include <Platform/OutputMemory.h>

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <utility>

ExpressionValue::ExpressionValue(const MacAddress& mac) noexcept
{
	type = static_cast<uint32_t>(TypeCode::MacAddress_tc);
	param = mac.HighWord();
	uVal = mac.LowWord();
}

ExpressionValue::ExpressionValue(const ExpressionValue& other) noexcept
{
	type = other.type;
	param = other.param;
	switch (other.GetType())
	{
	case TypeCode::Bool:
		bVal = other.bVal;
		break;

	case TypeCode::Char:
		cVal = other.cVal;
		break;

	case TypeCode::Float:
		fVal = other.fVal;
		break;

	case TypeCode::Int32:
		iVal = other.iVal;
		break;

	case TypeCode::Uint32:
	case TypeCode::Duration:
	case TypeCode::Enum32:
		uVal = other.uVal;
		break;

	case TypeCode::Uint64:
		uVal = other.uVal;
		break;

	case TypeCode::CString:
		sVal = other.sVal;
		break;

	case TypeCode::HeapString:
		new (&shVal) StringHandle(other.shVal);
		break;

	case TypeCode::HeapArray:
		new (&ahVal) ArrayHandle(other.ahVal);
		break;

	default:
		whole = other.whole;
		break;
	}
}

ExpressionValue::ExpressionValue(ExpressionValue&& other) noexcept
{
	type = other.type;
	param = other.param;
	whole = other.whole;
	other.type = static_cast<uint32_t>(TypeCode::None);
	other.param = 0;
	other.uVal = 0;
}

ExpressionValue& ExpressionValue::operator=(const ExpressionValue& other) noexcept
{
	if (this != &other)
	{
		Release();
		type = other.type;
		param = other.param;
		switch (other.GetType())
		{
		case TypeCode::Bool:
			bVal = other.bVal;
			break;

		case TypeCode::Char:
			cVal = other.cVal;
			break;

		case TypeCode::Float:
			fVal = other.fVal;
			break;

		case TypeCode::Int32:
			iVal = other.iVal;
			break;

		case TypeCode::Uint32:
		case TypeCode::Duration:
		case TypeCode::Enum32:
			uVal = other.uVal;
			break;

		case TypeCode::Uint64:
			uVal = other.uVal;
			break;

		case TypeCode::CString:
			sVal = other.sVal;
			break;

		case TypeCode::HeapString:
			new (&shVal) StringHandle(other.shVal);
			break;

		case TypeCode::HeapArray:
			new (&ahVal) ArrayHandle(other.ahVal);
			break;

		default:
			whole = other.whole;
			break;
		}
	}
	return *this;
}

void ExpressionValue::Release() noexcept
{
	switch (GetType())
	{
	case TypeCode::HeapString:
		shVal.Delete();
		type = static_cast<uint32_t>(TypeCode::None);
		break;

	case TypeCode::HeapArray:
		ahVal.Delete();
		type = static_cast<uint32_t>(TypeCode::None);
		break;

	default:
		break;
	}
}

void ExpressionValue::SetBool(bool b) noexcept
{
	Release();
	type = static_cast<uint32_t>(TypeCode::Bool);
	bVal = b;
}

void ExpressionValue::SetInt(int32_t i) noexcept
{
	Release();
	type = static_cast<uint32_t>(TypeCode::Int32);
	iVal = i;
}

void ExpressionValue::SetChar(char c) noexcept
{
	Release();
	type = static_cast<uint32_t>(TypeCode::Char);
	cVal = c;
}

void ExpressionValue::SetFloat(float f, uint32_t digits) noexcept
{
	Release();
	type = static_cast<uint32_t>(TypeCode::Float);
	fVal = f;
	param = digits;
}

void ExpressionValue::SetDriverId(DriverId did) noexcept
{
	Release();
	type = static_cast<uint32_t>(TypeCode::DriverId_tc);
#if SUPPORT_CAN_EXPANSION
	param = did.boardAddress;
#else
	param = 0;
#endif
	uVal = did.localDriver;
}

bool ExpressionValue::IsHeapStringArrayType() const noexcept
{
	if (type != static_cast<uint32_t>(TypeCode::HeapArray))
	{
		return false;
	}

	const std::size_t numElems = ahVal.GetNumElements();
	for (std::size_t i = 0; i < numElems; ++i)
	{
		if (ahVal.GetElementType(i) != TypeCode::HeapString)
		{
			return false;
		}
	}
	return true;
}

void ExpressionValue::AppendAsString(const StringRef& str) const noexcept
{
	switch (GetType())
	{
	case TypeCode::Char:
		str.cat(cVal);
		break;

	case TypeCode::CString:
		if (sVal != nullptr)
		{
			str.cat(sVal);
		}
		break;

	case TypeCode::HeapString:
		{
			const auto ptr = shVal.Get();
			if (ptr.Ptr() != nullptr)
			{
				str.cat(ptr.Ptr());
			}
		}
		break;

	case TypeCode::Float:
		str.catf(GetFloatFormatString(), static_cast<double>(fVal));
		break;

	case TypeCode::Uint32:
	case TypeCode::Duration:
		str.catf("%" PRIu32, uVal);
		break;

	case TypeCode::Int32:
		str.catf("%" PRIi32, iVal);
		break;

	case TypeCode::Bool:
		str.cat(bVal ? "true" : "false");
		break;

	default:
		str.cat("null");
		break;
	}
}

ObjectModel::ObjectModel() noexcept = default;

void ObjectModel::ReportAsJson(const GCodeBuffer*, OutputBuffer* buf, const char*, const char*, bool) const THROWS(GCodeException)
{
	if (buf != nullptr)
	{
		buf->cat("{}");
	}
}

ExpressionValue ObjectModel::GetObjectValueUsingTableNumber(ObjectExplorationContext&, const ObjectModelClassDescriptor*, const char*, uint8_t) const THROWS(GCodeException)
{
	return ExpressionValue();
}
