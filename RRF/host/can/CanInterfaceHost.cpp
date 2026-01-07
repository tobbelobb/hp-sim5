#include <CAN/CanInterface.h>

#include <CanMessageBuffer.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <RepRapFirmware.h>
#include <Platform/RepRap.h>
#include "CanCapture.h"
#include <HostTorqueMode.h>
#if SUPPORT_HANGPRINTER
#include <Movement/Kinematics/HangprinterKinematics.h>
#endif

#if SUPPORT_CAN_EXPANSION

namespace CanInterface
{
namespace
{
inline void FreeBuffer(CanMessageBuffer*& buf) noexcept
{
    if (buf != nullptr)
    {
        CanMessageBuffer::Free(buf);
        buf = nullptr;
    }
}

inline GCodeResult ReturnOk(const StringRef& reply,
                            const char* message = nullptr) noexcept
{
    if (message != nullptr)
    {
        reply.copy(message);
    }
    return GCodeResult::ok;
}

inline GCodeResult ReturnOk(const StringRef& reply, OutputBuffer*& buffer,
                            const char* message = nullptr) noexcept
{
    buffer = nullptr;
    return ReturnOk(reply, message);
}
}  // namespace

void Init() noexcept
{
}

void Shutdown() noexcept
{
}

CanAddress GetCanAddress() noexcept
{
    return 0;
}

void SendMotion(CanMessageBuffer* buf) noexcept
{
    if (buf != nullptr)
    {
        HostCanCapture::LogMotion(*buf);
        CanMessageBuffer::Free(buf);
    }
}

void WakeAsyncSender() noexcept
{
}

void WakeAsyncSenderFromIsr() noexcept
{
}

CanRequestId AllocateRequestId(CanAddress, CanMessageBuffer*) noexcept
{
    return 0;
}

GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer* buf, CanRequestId,
                                           const StringRef& reply, uint8_t*) noexcept
{
    FreeBuffer(buf);
    return ReturnOk(reply);
}

GCodeResult SendRequestAndGetCustomReply(
    CanMessageBuffer* buf, CanRequestId, const StringRef& reply, uint8_t*, CanMessageType,
    function_ref_noexcept<void(const CanMessageBuffer*) noexcept>) noexcept
{
    FreeBuffer(buf);
    return ReturnOk(reply);
}

void SendResponseNoFree(CanMessageBuffer* buf) noexcept
{
    FreeBuffer(buf);
}

void SendBroadcastNoFree(CanMessageBuffer* buf) noexcept
{
    FreeBuffer(buf);
}

void SendMessageNoReplyNoFree(CanMessageBuffer* buf) noexcept
{
    FreeBuffer(buf);
}

void Diagnostics(const StringRef& reply) noexcept
{
    ReturnOk(reply, "CAN diagnostics unavailable on host");
}

CanMessageBuffer* AllocateBuffer(const GCodeBuffer*) THROWS(GCodeException)
{
    return CanMessageBuffer::Allocate();
}

void CheckCanAddress(uint32_t, const GCodeBuffer&) THROWS(GCodeException)
{
}

uint16_t GetTimeStampCounter() noexcept
{
    return 0;
}

GCodeResult GetRemoteFirmwareDetails(uint32_t, GCodeBuffer&, const StringRef& reply)
    THROWS(GCodeException)
{
    return ReturnOk(reply, "No remote boards present (host build)");
}

GCodeResult RemoteDiagnostics(MessageType, uint32_t, unsigned int, GCodeBuffer&,
                              const StringRef& reply) THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult RemoteM408(uint32_t, unsigned int, GCodeBuffer&, const StringRef& reply)
    THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult HandleM111(uint32_t, GCodeBuffer&, const StringRef& reply)
    THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult EnableRemoteDrivers(const CanDriversList&, const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

void EnableRemoteDrivers(const CanDriversList&) noexcept
{
}

GCodeResult DisableRemoteDrivers(const CanDriversList&, const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

void DisableRemoteDrivers(const CanDriversList&) noexcept
{
}

void SetRemoteDriversIdle(const CanDriversList&, float) noexcept
{
}

GCodeResult SetRemoteStandstillCurrentPercent(const CanDriversData<float>&,
                                              const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult SetRemoteDriverCurrents(const CanDriversData<float>&,
                                    const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult SetRemotePressureAdvance(const CanDriversData<float>&,
                                     const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult SetRemoteDriverStepsPerMmAndMicrostepping(
    const CanDriversData<StepsPerUnitAndMicrostepping>&, const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply)
    THROWS(GCodeException)
{
    if (gb.GetCommandFraction() == 4)
    {
        if (!gb.Seen('T'))
        {
            reply.copy("Error: M569.4 missing parameter 'T'");
            return GCodeResult::error;
        }

        const float forceNewtons = gb.GetFValue();
#if SUPPORT_HANGPRINTER
        Kinematics& kin = reprap.GetMove().GetKinematics();
        if (kin.GetLegacyType() == KinematicsType::hangprinter)
        {
            float motorTorqueNm = 0.0F;
            bool positionMode = false;
            const GCodeResult conversion =
                static_cast<HangprinterKinematics&>(kin).ComputeODrive3TorqueFromForce(
                    driver, forceNewtons, motorTorqueNm, positionMode, reply);
            if (conversion != GCodeResult::ok)
            {
                return conversion;
            }
            const float appliedTorque = positionMode ? 0.0F : motorTorqueNm;
            const char* response =
                HostTorqueMode::Instance().SetTorqueModeExplicit(
                    driver.boardAddress, appliedTorque, positionMode);
            reply.cat(response);
            return GCodeResult::ok;
        }
#endif
        const char* response =
            HostTorqueMode::Instance().SetTorqueMode(driver.boardAddress, forceNewtons);
        reply.cat(response);
        return GCodeResult::ok;
    }

    return ReturnOk(reply);
}

GCodeResult GetSetRemoteDriverStallParameters(const CanDriversList&, GCodeBuffer&,
                                              const StringRef& reply, OutputBuffer*& buf)
    THROWS(GCodeException)
{
    return ReturnOk(reply, buf);
}

void EnableRemoteStallEndstop(DriverId, float) THROWS(GCodeException)
{
}

void DisableRemoteStallEndstops(CanAddress) noexcept
{
}

GCodeResult CreateHandle(CanAddress, RemoteInputHandle, const char*, uint16_t, uint16_t,
                         bool*, const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult DeleteHandle(CanAddress, RemoteInputHandle, const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult GetHandlePinName(CanAddress, RemoteInputHandle, bool*,
                             const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult EnableHandle(CanAddress, RemoteInputHandle, bool, bool*,
                         const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult ChangeHandleResponseTime(CanAddress, RemoteInputHandle, uint32_t, bool*,
                                     const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult ChangeHandleThreshold(CanAddress, RemoteInputHandle, uint32_t, bool*,
                                  const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult ChangeHandleSetTouchMode(CanAddress, RemoteInputHandle, uint32_t,
                                     const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult SetHandleDriveLevel(CanAddress, RemoteInputHandle, uint32_t,
                                uint8_t& returnedDriveLevel,
                                const StringRef& reply) noexcept
{
    returnedDriveLevel = 0;
    return ReturnOk(reply);
}

GCodeResult ReadRemoteHandles(CanAddress, RemoteInputHandle, RemoteInputHandle,
                              ReadHandlesCallbackFunction, CallbackParameter,
                              const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult CreateFilamentMonitor(DriverId, uint8_t, const GCodeBuffer&,
                                  const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult ConfigureFilamentMonitor(DriverId, GCodeBuffer&, const StringRef& reply)
    THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult DeleteFilamentMonitor(DriverId, GCodeBuffer*, const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult WriteGpio(CanAddress, uint8_t, float, bool, const GCodeBuffer*,
                      const StringRef& reply) noexcept
{
    return ReturnOk(reply);
}

GCodeResult ChangeAddressAndNormalTiming(GCodeBuffer&, const StringRef& reply)
    THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult ChangeFastTiming(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult StartClosedLoopDataCollection(DriverId, uint16_t, uint16_t, uint16_t, uint8_t,
                                          uint8_t, const GCodeBuffer&,
                                          const StringRef& reply) THROWS(GCodeException)
{
    return ReturnOk(reply);
}

GCodeResult ProcessM655(GCodeBuffer&, const StringRef& reply) THROWS(GCodeException)
{
    return ReturnOk(reply);
}

#if DUAL_CAN
uint32_t SendPlainMessageNoFree(CanMessageBuffer* buf, uint32_t) noexcept
{
    (void)buf;
    return 0;
}

bool ReceivePlainMessage(CanMessageBuffer*, uint32_t) noexcept
{
    return false;
}

namespace ODrive
{
CanId ArbitrationId(DriverId, uint8_t) noexcept
{
    CanId id;
    id.SetReceivedId(0);
    return id;
}

CanMessageBuffer* PrepareSimpleMessage(DriverId const, const StringRef& reply) noexcept
{
    CanMessageBuffer* buf = CanMessageBuffer::Allocate();
    if (buf == nullptr)
    {
        reply.copy("no CAN buffer");
    }
    return buf;
}

void FlushCanReceiveHardware() noexcept
{
}

bool GetExpectedSimpleMessage(CanMessageBuffer* buf, DriverId const, uint8_t const,
                              const StringRef& reply) noexcept
{
    (void)buf;
    reply.copy("Message not received");
    return false;
}
} // namespace ODrive
#endif

}  // namespace CanInterface

#endif
