#include "GCodeInjector.h"

#include <chrono>

#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>

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
    }

    if (!activeCommand_ || activeCommand_->started)
    {
        return;
    }

    GCodeBuffer* httpBuffer = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::HTTP);
    if (httpBuffer != nullptr && httpBuffer->IsCompletelyIdle())
    {
        httpBuffer->Put(activeCommand_->gcode.c_str());
        activeCommand_->started = true;
    }
}

void GCodeInjector::OnResponse(const char* response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (activeCommand_)
    {
        activeCommand_->response = (response != nullptr) ? response : "";
        activeCommand_->completed = true;
        activeCommand_->cv.notify_all();
    }
}
