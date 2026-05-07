#include <Platform/RepRap.h>

#if RRF_HOST_BUILD
void RepRap::UpdateFirmware(const char*, const char*) noexcept
{
    // No firmware/IAP update in host simulator.
}
#endif
