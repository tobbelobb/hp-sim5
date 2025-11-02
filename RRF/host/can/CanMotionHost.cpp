#include <CAN/CanMotion.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CanMessageBuffer.h>
# include <CanMessageFormats.h>
# include <CanId.h>
# include <algorithm>

namespace
{
	CanMessageBuffer *movementBufferList = nullptr;
	uint32_t currentMoveClocks = 0;
	uint8_t nextSeq[CanId::MaxCanAddress + 1] = { 0 };

	void FreeMovementBuffers() noexcept
	{
		while (movementBufferList != nullptr)
		{
			CanMessageBuffer *buf = movementBufferList;
			movementBufferList = buf->next;
			CanMessageBuffer::Free(buf);
		}
	}

	CanMessageBuffer *_ecv_null GetOrCreateBuffer(const PrepParams& params, DriverId canDriver) noexcept
	{
		if (canDriver.localDriver >= MaxLinearDriversPerCanSlave)
		{
			return nullptr;
		}

		for (CanMessageBuffer *buf = movementBufferList; buf != nullptr; buf = buf->next)
		{
			if (buf->id.Dst() == canDriver.boardAddress)
			{
				if (canDriver.localDriver >= buf->msg.moveLinearShaped.numDrivers)
				{
					buf->msg.moveLinearShaped.numDrivers = canDriver.localDriver + 1;
				}
				return buf;
			}
		}

		CanMessageBuffer *buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			return nullptr;
		}

		buf->next = movementBufferList;
		movementBufferList = buf;

		auto move = buf->SetupRequestMessageNoRid<CanMessageMovementLinearShaped>(CanInterface::GetCurrentMasterAddress(), canDriver.boardAddress);
		if (buf->next == nullptr)
		{
			move->accelerationClocks = params.TotalAccelClocks();
			move->steadyClocks = params.SteadyClocks();
			move->decelClocks = params.TotalDecelClocks();
			currentMoveClocks = params.TotalClocks();
		}
		else
		{
			move->accelerationClocks = buf->next->msg.moveLinearShaped.accelerationClocks;
			move->steadyClocks = buf->next->msg.moveLinearShaped.steadyClocks;
			move->decelClocks = buf->next->msg.moveLinearShaped.decelClocks;
		}

# if SUPPORT_S_CURVE
		if (params.totalDistance > 0.0f)
		{
			if (params.jerk != 0.0f)
			{
				move->acceleration = (params.peakAcceleration * params.TotalAccelClocks() - 0.5f * params.jerk * (fsquare(params.phaseClocks[0]) + fsquare(params.phaseClocks[2])))/(params.TotalAccelClocks() * params.totalDistance);
				move->deceleration = (-params.peakDeceleration * params.TotalDecelClocks() - 0.5f * params.jerk * (fsquare(params.phaseClocks[4]) + fsquare(params.phaseClocks[6])))/(params.TotalDecelClocks() * params.totalDistance);
			}
			else
			{
				move->acceleration = params.peakAcceleration/params.totalDistance;
				move->deceleration = -params.peakDeceleration/params.totalDistance;
			}
		}
		else
		{
			move->acceleration = 0.0f;
			move->deceleration = 0.0f;
		}
# else
		if (params.totalDistance > 0.0f)
		{
			move->acceleration = params.acceleration/params.totalDistance;
			move->deceleration = params.deceleration/params.totalDistance;
		}
		else
		{
			move->acceleration = 0.0f;
			move->deceleration = 0.0f;
		}
# endif

		move->extruderDrives = 0;
		move->numDrivers = canDriver.localDriver + 1;
		move->zero1 = move->zero2 = 0;
		move->useLateInputShaping = params.useInputShaping;
		for (size_t drive = 0; drive < ARRAY_SIZE(move->perDrive); ++drive)
		{
			move->perDrive[drive].Init();
		}
		return buf;
	}
}

namespace CanMotion
{
	void Init() noexcept
	{
		FreeMovementBuffers();
		currentMoveClocks = 0;
		std::fill(std::begin(nextSeq), std::end(nextSeq), 0);
	}

	void StartMovement() noexcept
	{
		FreeMovementBuffers();
		currentMoveClocks = 0;
	}

	void AddAxisMovement(const PrepParams& params, DriverId canDriver, int32_t steps) noexcept
	{
		if (steps == 0)
		{
			return;
		}

		if (CanMessageBuffer *buf = GetOrCreateBuffer(params, canDriver))
		{
			buf->msg.moveLinearShaped.perDrive[canDriver.localDriver].steps = steps;
		}
	}

	void AddExtruderMovement(const PrepParams& params, DriverId canDriver, float extrusion, bool usePressureAdvance) noexcept
	{
		if (extrusion == 0.0f)
		{
			return;
		}

		if (CanMessageBuffer *buf = GetOrCreateBuffer(params, canDriver))
		{
			auto& msg = buf->msg.moveLinearShaped;
			msg.perDrive[canDriver.localDriver].extrusion = extrusion;
			msg.extruderDrives |= 1u << canDriver.localDriver;
			msg.usePressureAdvance = usePressureAdvance;
		}
	}

	uint32_t FinishMovement(const DDA& dda, uint32_t moveStartTime, bool simulating) noexcept
	{
		(void)dda;
		(void)simulating;
		if (movementBufferList == nullptr)
		{
			return 0;
		}

		uint32_t clocks = 0;
		CanMessageBuffer *buf = movementBufferList;
		movementBufferList = nullptr;

		while (buf != nullptr)
		{
			CanMessageBuffer *nextBuffer = buf->next;
			auto& msg = buf->msg.moveLinearShaped;

			if (msg.HasMotion())
			{
				msg.whenToExecute = moveStartTime;
				uint8_t& seq = nextSeq[buf->id.Dst()];
				msg.seq = seq;
				seq = (seq + 1) & 0x7F;
				buf->dataLength = msg.GetActualDataLength();
				clocks = currentMoveClocks;
				CanInterface::SendMotion(buf);
			}
			else
			{
				CanMessageBuffer::Free(buf);
			}

			buf = nextBuffer;
		}

		currentMoveClocks = 0;
		return clocks;
	}

	bool CanPrepareMove() noexcept { return true; }

	CanMessageBuffer *GetUrgentMessage() noexcept { return nullptr; }

	void StopDriverWhenProvisional(DriverId) noexcept {}

	bool StopDriverWhenExecuting(DriverId, int32_t) noexcept { return false; }

	void FinishedStoppingDrivers() noexcept {}

	bool RevertStoppedDrivers() noexcept { return true; }
}

#endif
