#include <CAN/CanMotion.h>
#if SUPPORT_CAN_EXPANSION
# include <CanMessageBuffer.h>

namespace CanMotion
{
	void Init() noexcept {}

	void StartMovement() noexcept {}

	void AddAxisMovement(const PrepParams&, DriverId, int32_t) noexcept {}

	void AddExtruderMovement(const PrepParams&, DriverId, float, bool) noexcept {}

	uint32_t FinishMovement(const DDA&, uint32_t moveStartTime, bool) noexcept
	{
		// Host build does not push segments over CAN, so just return the original start time.
		return moveStartTime;
	}

	bool CanPrepareMove() noexcept { return true; }

	CanMessageBuffer *GetUrgentMessage() noexcept { return nullptr; }

	void StopDriverWhenProvisional(DriverId) noexcept {}

	bool StopDriverWhenExecuting(DriverId, int32_t) noexcept { return false; }

	void FinishedStoppingDrivers() noexcept {}

	bool RevertStoppedDrivers() noexcept { return true; }
}

#endif
