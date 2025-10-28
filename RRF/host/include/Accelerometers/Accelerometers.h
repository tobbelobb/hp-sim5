#pragma once


#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
#endif

class CanMessageAccelerometerData;

namespace Accelerometers
{
	bool HasLocalAccelerometer() noexcept { return false; }
	unsigned int GetLocalAccelerometerRuns() noexcept { return 0; }
	unsigned int GetLocalAccelerometerDataPoints() noexcept { return 0; }
	uint8_t GetLocalAccelerometerOrientation() noexcept { return 0; }
	GCodeResult ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void Exit() noexcept {}
#if SUPPORT_CAN_EXPANSION
	void ProcessReceivedData(CanAddress src, const CanMessageAccelerometerData& msg, size_t msgLen) noexcept {}
#endif
}
