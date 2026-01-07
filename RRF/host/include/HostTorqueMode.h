#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>

// Callback fired when torque mode changes; torqueNm == 0 means position mode
using TorqueModeCallback = std::function<void(uint8_t driverAddress, float torqueNm)>;

class HostTorqueMode
{
public:
    static HostTorqueMode& Instance();

    // Set torque mode for a driver and return formatted response string.
    // Returns an error string for invalid driver addresses.
    const char* SetTorqueMode(uint8_t driverAddress, float torqueNm);
    // Set torque mode without applying any threshold; positionMode overrides torque value.
    const char* SetTorqueModeExplicit(uint8_t driverAddress, float torqueNm, bool positionMode);

    float GetTorque(uint8_t driverAddress) const;
    bool IsInTorqueMode(uint8_t driverAddress) const;

    void SetCallback(TorqueModeCallback callback);

    // Returns JSON blob with torque mode state for drivers 40-43.
    std::string GetStatusJson() const;

private:
    HostTorqueMode() = default;

    static constexpr uint8_t MIN_DRIVER = 40;
    static constexpr uint8_t MAX_DRIVER = 43;
    static constexpr size_t NUM_DRIVERS = MAX_DRIVER - MIN_DRIVER + 1;
    static constexpr float MIN_TORQUE_THRESHOLD = 0.0001f;

    std::array<float, NUM_DRIVERS> torques_{};
    TorqueModeCallback callback_;
    char responseBuffer_[32]{};
};
