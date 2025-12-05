#include <Movement/HeightControl/HeightController.h>

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/GCodeException.h>
#include <GCodeResult.h>
#include <General/StringRef.h>

#include <chrono>
#include <thread>

HeightController::HeightController() noexcept = default;

GCodeResult HeightController::Configure(GCodeBuffer&, const StringRef&) THROWS(GCodeException)
{
    return GCodeResult::ok;
}

GCodeResult HeightController::StartHeightFollowing(GCodeBuffer&, const StringRef&)
    THROWS(GCodeException)
{
    return GCodeResult::ok;
}

void HeightController::Stop() noexcept
{
}

[[noreturn]] void HeightController::RunTask() noexcept
{
    for (;;)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
