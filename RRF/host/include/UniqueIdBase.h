#pragma once

#include "Core.h"
#include <General/StringRef.h>
#include <General/function_ref.h>
#include <array>

class UniqueIdBase
{
public:
	UniqueIdBase() { Clear(); }

	bool IsValid() const noexcept { return valid; }
	uint32_t GetHash() const noexcept { return 0; }

	void Clear() noexcept
	{
		data.fill(0);
		valid = false;
	}

	void SetFromCurrentBoard() noexcept { valid = true; }
	void SetFromRemote(const uint8_t[16]) noexcept { valid = true; }

	void AppendCharsTo(function_ref_noexcept<void(char) noexcept> fn) const noexcept
	{
		constexpr char placeholder[] = "00000-00000-00000-00000-00000-00000";
		for (char c : placeholder)
		{
			fn(c);
		}
	}

	void AppendCharsToString(const StringRef& str) const noexcept
	{
		str.copy("00000-00000-00000-00000-00000-00000");
	}

	const uint8_t* GetRaw() const { return reinterpret_cast<const uint8_t*>(data.data()); }
	const uint32_t* GetDwords() const { return data.data(); }

protected:
	void SetChecksumWord() noexcept {}

private:
	std::array<uint32_t, 5> data{};
	bool valid = false;
};

