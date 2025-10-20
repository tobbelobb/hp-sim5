#pragma once

#include <cstdint>

using DmaChannel = uint8_t;
using DmaPriority = uint8_t;

struct Pin
{
	int port{-1};
	int bit{-1};

	constexpr Pin() noexcept = default;
	constexpr Pin(int p, int b) noexcept : port(p), bit(b) {}
	constexpr explicit Pin(uint32_t raw) noexcept
	: port((raw == 0xFFFFFFFFu) ? -1 : static_cast<int>(raw / 32u))
	, bit((raw == 0xFFFFFFFFu) ? -1 : static_cast<int>(raw % 32u))
	{
	}

	constexpr explicit operator bool() const noexcept
	{
		return port >= 0 && bit >= 0;
	}

	constexpr operator uint32_t() const noexcept
	{
		return (port >= 0 && bit >= 0)
			? static_cast<uint32_t>(static_cast<uint32_t>(port) * 32u + static_cast<uint32_t>(bit))
			: 0xFFFFFFFFu;
	}
};

constexpr inline bool operator==(Pin lhs, Pin rhs) noexcept
{
	return lhs.port == rhs.port && lhs.bit == rhs.bit;
}

constexpr inline bool operator!=(Pin lhs, Pin rhs) noexcept
{
	return !(lhs == rhs);
}

using PwmFrequency = uint16_t;
using CanAddress = uint8_t;
using NvicPriority = uint32_t;
using ExintNumber = uint8_t;
using EventNumber = uint8_t;
using AnalogChannelNumber = uint8_t;
using IRQn_Type = int;

#ifndef __FP16_TYPE_DEFINED
#define __FP16_TYPE_DEFINED
using __fp16 = float;
#endif

#ifndef FLOAT16_T_DEFINED
# define FLOAT16_T_DEFINED
using float16_t = __fp16;
#endif

constexpr Pin NoPin{};
constexpr Pin Nx{};
