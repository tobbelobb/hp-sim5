#pragma once

#include <cstdint>

class TaskBase;

namespace HostRTOS
{
void EnterCritical() noexcept;
void ExitCritical() noexcept;
void Yield() noexcept;
TaskBase* GetCurrentTaskBase() noexcept;
}  // namespace HostRTOS
