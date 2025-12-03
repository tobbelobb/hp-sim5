#include <HostTorqueMode.h>

#include <cmath>
#include <cstdio>
#include <utility>

HostTorqueMode& HostTorqueMode::Instance()
{
    static HostTorqueMode instance;
    return instance;
}

const char* HostTorqueMode::SetTorqueMode(uint8_t driverAddress, float torqueNm)
{
    if (driverAddress < MIN_DRIVER || driverAddress > MAX_DRIVER)
    {
        std::snprintf(responseBuffer_, sizeof(responseBuffer_),
                      "Error: Invalid driver %d", driverAddress);
        return responseBuffer_;
    }

    const size_t index = driverAddress - MIN_DRIVER;

    if (std::fabs(torqueNm) < MIN_TORQUE_THRESHOLD)
    {
        torques_[index] = 0.0f;
        std::snprintf(responseBuffer_, sizeof(responseBuffer_), "pos_mode, ");

        if (callback_)
        {
            callback_(driverAddress, 0.0f);
        }
    }
    else
    {
        torques_[index] = torqueNm;
        std::snprintf(responseBuffer_, sizeof(responseBuffer_), "%.6f Nm, ", torqueNm);

        if (callback_)
        {
            callback_(driverAddress, torqueNm);
        }
    }

    return responseBuffer_;
}

float HostTorqueMode::GetTorque(uint8_t driverAddress) const
{
    if (driverAddress < MIN_DRIVER || driverAddress > MAX_DRIVER)
    {
        return 0.0f;
    }
    return torques_[driverAddress - MIN_DRIVER];
}

bool HostTorqueMode::IsInTorqueMode(uint8_t driverAddress) const
{
    return GetTorque(driverAddress) != 0.0f;
}

void HostTorqueMode::SetCallback(TorqueModeCallback callback)
{
    callback_ = std::move(callback);
}

std::string HostTorqueMode::GetStatusJson() const
{
    std::string json = "{\"torqueMode\":{";
    for (size_t i = 0; i < NUM_DRIVERS; ++i)
    {
        if (i > 0)
        {
            json += ",";
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "\"driver%zu\":{\"address\":%u,\"torqueNm\":%.6f}",
                      i, static_cast<unsigned int>(MIN_DRIVER + i), torques_[i]);
        json += buf;
    }
    json += "}}";
    return json;
}
