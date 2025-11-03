#include <cstddef>

#include <Networking/NetworkDefs.h>

uint32_t MacAddress::LowWord() const noexcept
{
    return (static_cast<uint32_t>(bytes[3]) << 24) |
           (static_cast<uint32_t>(bytes[2]) << 16) |
           (static_cast<uint32_t>(bytes[1]) << 8) | static_cast<uint32_t>(bytes[0]);
}

uint16_t MacAddress::HighWord() const noexcept
{
    return static_cast<uint16_t>(bytes[5]) << 8 | static_cast<uint16_t>(bytes[4]);
}

void MacAddress::SetFromBytes(const uint8_t mb[6]) noexcept
{
    for (std::size_t i = 0; i < 6; ++i)
    {
        bytes[i] = mb[i];
    }
}
