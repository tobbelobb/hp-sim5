#pragma once

#include "CoreTypes.h"

#include <cstdint>

enum class PinCapability : uint8_t
{
    none = 0
};

inline constexpr PinCapability operator|(PinCapability lhs, PinCapability rhs) noexcept
{
    return static_cast<PinCapability>(static_cast<uint8_t>(lhs) |
                                      static_cast<uint8_t>(rhs));
}

inline constexpr PinCapability operator&(PinCapability lhs, PinCapability rhs) noexcept
{
    return static_cast<PinCapability>(static_cast<uint8_t>(lhs) &
                                      static_cast<uint8_t>(rhs));
}

inline constexpr bool HasCapability(PinCapability value,
                                    PinCapability capability) noexcept
{
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(capability)) != 0;
}

struct PinDescription
{
    Pin pin{};
    PinCapability capability{PinCapability::none};
    bool inverted{false};
};
