#include <CAN/CanInterface.h>

#include "CanCapture.h"
#include <CanMessageBuffer.h>
#include <HostTiming.h>
#include <RepRapFirmware.h>

#if SUPPORT_CAN_EXPANSION

namespace CanInterface
{
namespace
{
constexpr uint64_t MasterClocksPerStepTick = HostTiming::StepClockFrequencyHz/StepClockRate;
static_assert(StepClockRate != 0, "Step clock rate must not be zero");
static_assert(HostTiming::StepClockFrequencyHz % StepClockRate == 0, "Master clock must be multiple of step clock rate");

inline void ConvertMovementToMasterClock(CanMessageMovementLinearShaped& msg) noexcept
{
	if (MasterClocksPerStepTick == 1)
	{
		return;
	}

	msg.whenToExecute = static_cast<uint32_t>(static_cast<uint64_t>(msg.whenToExecute) * MasterClocksPerStepTick);
	msg.accelerationClocks = static_cast<uint32_t>(static_cast<uint64_t>(msg.accelerationClocks) * MasterClocksPerStepTick);
	msg.steadyClocks = static_cast<uint32_t>(static_cast<uint64_t>(msg.steadyClocks) * MasterClocksPerStepTick);
	msg.decelClocks = static_cast<uint32_t>(static_cast<uint64_t>(msg.decelClocks) * MasterClocksPerStepTick);
}

inline void FreeBuffer(CanMessageBuffer *&buf) noexcept
{
	if (buf != nullptr)
	{
		CanMessageBuffer::Free(buf);
		buf = nullptr;
	}
}

inline GCodeResult ReturnOk(const StringRef& reply, const char *message = nullptr) noexcept
{
	if (message != nullptr)
	{
		reply.copy(message);
	}
	return GCodeResult::ok;
}

inline GCodeResult ReturnOk(const StringRef& reply, OutputBuffer *&buffer, const char *message = nullptr) noexcept
{
	buffer = nullptr;
	return ReturnOk(reply, message);
}
}

void Init() noexcept {}

void Shutdown() noexcept {}

CanAddress GetCanAddress() noexcept
{
	return 0;
}

void SendMotion(CanMessageBuffer *buf) noexcept
{
	if (buf != nullptr)
	{
		if (buf->id.MsgType() == CanMessageType::movementLinearShaped)
		{
			ConvertMovementToMasterClock(buf->msg.moveLinearShaped);
		}
		else if (buf->id.MsgType() == CanMessageType::revertPosition && MasterClocksPerStepTick != 1)
		{
			buf->msg.revertPosition.clocksAllowed = static_cast<uint32_t>(static_cast<uint64_t>(buf->msg.revertPosition.clocksAllowed) * MasterClocksPerStepTick);
		}
		HostCanCapture::LogMotion(*buf);
		CanMessageBuffer::Free(buf);
	}
}

void WakeAsyncSender() noexcept {}

void WakeAsyncSenderFromIsr() noexcept {}

CanRequestId AllocateRequestId(CanAddress, CanMessageBuffer*) noexcept
{
	return 0;
}

GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId, const StringRef& reply, uint8_t*) noexcept
{
	FreeBuffer(buf);
	return ReturnOk(reply);
}

GCodeResult SendRequestAndGetCustomReply(CanMessageBuffer *buf, CanRequestId, const StringRef& reply, uint8_t*, CanMessageType, function_ref_noexcept<void(const CanMessageBuffer*) noexcept>) noexcept
{
	FreeBuffer(buf);
	return ReturnOk(reply);
}

void SendResponseNoFree(CanMessageBuffer *buf) noexcept
{
	FreeBuffer(buf);
}

void SendBroadcastNoFree(CanMessageBuffer *buf) noexcept
{
	FreeBuffer(buf);
}

void SendMessageNoReplyNoFree(CanMessageBuffer *buf) noexcept
{
	FreeBuffer(buf);
}

void Diagnostics(const StringRef& reply) noexcept
{
	ReturnOk(reply, "CAN diagnostics unavailable on host");
}

CanMessageBuffer *AllocateBuffer(const GCodeBuffer*) THROWS(GCodeException)
{
	return CanMessageBuffer::Allocate();
}

void CheckCanAddress(uint32_t, const GCodeBuffer&) THROWS(GCodeException) {}

uint16_t GetTimeStampCounter() noexcept { return 0; }

GCodeResult GetRemoteFirmwareDetails(uint32_t, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply, "No remote boards present (host build)");
}

GCodeResult RemoteDiagnostics(MessageType, uint32_t, unsigned int, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult RemoteM408(uint32_t, unsigned int, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult HandleM111(uint32_t, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult EnableRemoteDrivers(const CanDriversList&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

void EnableRemoteDrivers(const CanDriversList&) noexcept {}

GCodeResult DisableRemoteDrivers(const CanDriversList&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

void DisableRemoteDrivers(const CanDriversList&) noexcept {}

void SetRemoteDriversIdle(const CanDriversList&, float) noexcept {}

GCodeResult SetRemoteStandstillCurrentPercent(const CanDriversData<float>&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult SetRemoteDriverCurrents(const CanDriversData<float>&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult SetRemotePressureAdvance(const CanDriversData<float>&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult SetRemoteDriverStepsPerMmAndMicrostepping(const CanDriversData<StepsPerUnitAndMicrostepping>&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult ConfigureRemoteDriver(DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult GetSetRemoteDriverStallParameters(const CanDriversList&, GCodeBuffer&, const StringRef& reply, OutputBuffer *&buf) THROWS(GCodeException)
{
	return ReturnOk(reply, buf);
}

void EnableRemoteStallEndstop(DriverId, float) THROWS(GCodeException) {}

void DisableRemoteStallEndstops(CanAddress) noexcept {}

GCodeResult CreateHandle(CanAddress, RemoteInputHandle, const char*, uint16_t, uint16_t, bool*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult DeleteHandle(CanAddress, RemoteInputHandle, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult GetHandlePinName(CanAddress, RemoteInputHandle, bool*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult EnableHandle(CanAddress, RemoteInputHandle, bool, bool*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult ChangeHandleResponseTime(CanAddress, RemoteInputHandle, uint32_t, bool*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult ChangeHandleThreshold(CanAddress, RemoteInputHandle, uint32_t, bool*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult ChangeHandleSetTouchMode(CanAddress, RemoteInputHandle, uint32_t, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult SetHandleDriveLevel(CanAddress, RemoteInputHandle, uint32_t, uint8_t &returnedDriveLevel, const StringRef& reply) noexcept
{
	returnedDriveLevel = 0;
	return ReturnOk(reply);
}

GCodeResult ReadRemoteHandles(CanAddress, RemoteInputHandle, RemoteInputHandle, ReadHandlesCallbackFunction, CallbackParameter, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult CreateFilamentMonitor(DriverId, uint8_t, const GCodeBuffer&, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult ConfigureFilamentMonitor(DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult DeleteFilamentMonitor(DriverId, GCodeBuffer*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult WriteGpio(CanAddress, uint8_t, float, bool, const GCodeBuffer*, const StringRef& reply) noexcept
{
	return ReturnOk(reply);
}

GCodeResult ChangeAddressAndNormalTiming(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult ChangeFastTiming(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult StartClosedLoopDataCollection(DriverId, uint16_t, uint16_t, uint16_t, uint8_t, uint8_t, const GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

GCodeResult ProcessM655(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
	return ReturnOk(reply);
}

} // namespace CanInterface

#endif
