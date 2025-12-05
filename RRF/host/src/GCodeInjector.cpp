#include "GCodeInjector.h"

#include <chrono>
#include <cstdio>

#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <Platform/RepRap.h>
#include <can/CanCapture.h>

GCodeInjector& GCodeInjector::Instance()
{
    static GCodeInjector instance;
    return instance;
}

std::string GCodeInjector::ExecuteBlocking(const std::string& gcode, uint32_t timeoutMs)
{
    auto cmd = std::make_shared<PendingCommand>();
    cmd->gcode = gcode;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pendingCommands_.push(cmd);
    }

    std::unique_lock<std::mutex> lock(mutex_);
    const bool completed = cmd->cv.wait_for(lock,
        std::chrono::milliseconds(timeoutMs),
        [&cmd]() { return cmd->completed; });

    if (!completed)
    {
        return "Error: Command timed out";
    }

    return cmd->response;
}

void GCodeInjector::ProcessPending()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (activeCommand_ && activeCommand_->completed)
    {
        activeCommand_.reset();
    }

    if (!activeCommand_ && !pendingCommands_.empty())
    {
        activeCommand_ = pendingCommands_.front();
        pendingCommands_.pop();
        fprintf(stderr, "[GCodeInjector] activating '%s'\n", activeCommand_->gcode.c_str());
    }

    GCodeBuffer* httpBuffer = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::HTTP);
    if (!activeCommand_)
    {
        return;
    }

    if (!activeCommand_->started)
    {
        if (httpBuffer != nullptr && httpBuffer->IsCompletelyIdle())
        {
            fprintf(stderr, "[GCodeInjector] starting '%s'\n", activeCommand_->gcode.c_str());
            HostCanCapture::StartCapture();
            httpBuffer->PutAndDecode(activeCommand_->gcode.c_str());
            activeCommand_->started = true;
        }
        return;
    }
    else if (!activeCommand_->responseReady && httpBuffer != nullptr)
    {
        static auto lastPendingLog = std::chrono::steady_clock::time_point{};
        const auto now = std::chrono::steady_clock::now();
        if (lastPendingLog.time_since_epoch().count() == 0 ||
            now - lastPendingLog > std::chrono::seconds(1))
        {
            lastPendingLog = now;
            const MovementState& ms = reprap.GetGCodes().GetConstMovementState(*httpBuffer);
            fprintf(stderr,
                    "[GCodeInjector] pending response: state=%u segLeft=%u totalSeg=%u "
                    "scheduled=%u completed=%u\n",
                    static_cast<unsigned>(httpBuffer->GetState()), ms.segmentsLeft,
                    ms.totalSegments, reprap.GetMove().GetScheduledMoves(),
                    reprap.GetMove().GetCompletedMoves());
        }
    }

    if (activeCommand_->responseReady && !activeCommand_->completed)
    {
        static auto lastLog = std::chrono::steady_clock::time_point{};
        const bool httpIdle =
            (httpBuffer == nullptr) ? true : httpBuffer->IsCompletelyIdle();
        const bool moveIdle = reprap.GetMove().NoLiveMovement();

        if (!(httpIdle && moveIdle))
        {
            const auto now = std::chrono::steady_clock::now();
            if (lastLog.time_since_epoch().count() == 0 ||
                now - lastLog > std::chrono::seconds(1))
            {
                lastLog = now;
                const MovementState& ms = reprap.GetGCodes().GetConstMovementState(*httpBuffer);
                fprintf(stderr,
                        "[GCodeInjector] waiting: httpIdle=%d moveIdle=%d segLeft=%u "
                        "totalSeg=%u scheduled=%u completed=%u\n",
                        httpIdle, moveIdle, ms.segmentsLeft, ms.totalSegments,
                        reprap.GetMove().GetScheduledMoves(),
                        reprap.GetMove().GetCompletedMoves());
            }
        }

        if (httpIdle && moveIdle)
        {
            HostCanCapture::StopCapture();
            std::string motionData = HostCanCapture::FlushCapture();

            if (!motionData.empty())
            {
                activeCommand_->response += "\n---MOTION---\n";
                activeCommand_->response += "{\"capture_version\":1}\n";
                activeCommand_->response += motionData;
            }

            activeCommand_->completed = true;
            activeCommand_->cv.notify_all();
        }
    }
}

void GCodeInjector::OnResponse(const char* response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!activeCommand_)
    {
        return;
    }

    fprintf(stderr, "[GCodeInjector] got response for '%s': '%s'\n",
            activeCommand_->gcode.c_str(), (response != nullptr) ? response : "");
    activeCommand_->response = (response != nullptr) ? response : "";
    activeCommand_->responseReady = true;
}
