#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

class GCodeInjector
{
public:
    static GCodeInjector& Instance();

    // Submit G-code and block until response is ready (or timeout)
    std::string ExecuteBlocking(const std::string& gcode, uint32_t timeoutMs = 30000);

    // Called from the Spin loop to process pending commands
    void ProcessPending();

    // Called when a response is generated (hook into GCodes reply system)
    void OnResponse(const char* response);

private:
    GCodeInjector() = default;

    struct PendingCommand
    {
        std::string gcode;
        std::string response;
        bool completed{false};
        bool started{false};
        std::condition_variable cv;
    };

    std::mutex mutex_;
    std::queue<std::shared_ptr<PendingCommand>> pendingCommands_;
    std::shared_ptr<PendingCommand> activeCommand_;
};
