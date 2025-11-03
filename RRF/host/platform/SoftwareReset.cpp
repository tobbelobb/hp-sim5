#include <Hardware/SoftwareReset.h>

#include <General/StringRef.h>

#include <cstdio>
#include <cstdlib>

const char* const SoftwareResetData::ReasonText[] = {"user",
                                                     "erase",
                                                     "NMI",
                                                     "hardFault",
                                                     "stuckInSpin",
                                                     "wdtFault",
                                                     "usageFault",
                                                     "otherFault",
                                                     "stackOverflow",
                                                     "assertCalled",
                                                     "heaterWatchdog",
                                                     "memFault",
                                                     "terminateCalled",
                                                     "pureVirtual",
                                                     "outOfMemory"};

uint8_t SoftwareResetData::extraDebugInfo = 0;

bool SoftwareResetData::IsVacant() const noexcept
{
    return true;
}

void SoftwareResetData::Clear() noexcept
{
    magic = 0;
    resetReason = 0;
}

void SoftwareResetData::Populate(uint16_t, const uint32_t*) noexcept
{
}

void SoftwareResetData::PrintPart1(unsigned int, const StringRef& reply) const noexcept
{
    reply.copy("software reset data not available");
}

void SoftwareResetData::PrintPart2(const StringRef& reply) const noexcept
{
    reply.copy("");
}

[[noreturn]] void SoftwareReset(SoftwareResetReason reason, const uint32_t*) noexcept
{
    std::fprintf(stderr, "Software reset requested: %u\n", static_cast<unsigned>(reason));
    std::abort();
}
