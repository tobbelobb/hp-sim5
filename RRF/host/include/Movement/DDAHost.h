#pragma once

#ifdef RRF_HOST_BUILD

#include <RepRapFirmware.h>
#include <cstdint>

struct RawMove;

namespace host
{

class DDAHost
{
public:
    static constexpr int kMaxDrives = MaxAxesPlusExtruders;

    struct DrivePlan
    {
        uint8_t id = 0;
        int64_t steps = 0;
        double stepsPerSecTop = 0.0;
    };

    struct PrepOut
    {
        uint64_t startClock = 0;
        uint32_t accelClocks = 0;
        uint32_t steadyClocks = 0;
        uint32_t decelClocks = 0;
        double vEntry = 0.0;
        double vTop = 0.0;
        double vExit = 0.0;
        double acceleration = 0.0;
        double deceleration = 0.0;
        double accelDistance = 0.0;
        double decelDistanceStart = 0.0;
        double totalDistance = 0.0;
        int numDrives = 0;
        DrivePlan drives[kMaxDrives];
    };

    static bool Prepare(const RawMove& mv,
                        const float startMachineCoords[MaxAxesPlusExtruders],
                        const int32_t startSteps[MaxAxesPlusExtruders],
                        const int32_t endSteps[MaxAxesPlusExtruders], double entrySpeed,
                        double topSpeed, double exitSpeed, double accelLimit,
                        PrepOut& out) noexcept;
};

}  // namespace host

#endif  // RRF_HOST_BUILD
