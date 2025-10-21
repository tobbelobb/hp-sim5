#pragma once

#include <cstddef>
#include <cstdint>

using BaseType_t = int32_t;
using UBaseType_t = uint32_t;
using TickType_t = uint32_t;
using StackType_t = uint32_t;

struct StaticTask_t
{
	void* hostContext{nullptr};
};

struct StaticQueue_t
{
	void* hostContext{nullptr};
};

struct StaticSemaphore_t
{
	void* hostContext{nullptr};
};

struct StaticEventGroup_t
{
	void* hostContext{nullptr};
};

#define pdMS_TO_TICKS(x) static_cast<TickType_t>(x)
#define portMAX_DELAY 0xFFFFFFFFu
#define pdPASS 1
#define pdFAIL 0
#define pdTRUE 1
#define pdFALSE 0

#ifndef configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 0
#endif

#define configMINIMAL_STACK_SIZE 256

#include <rtos/host_rtos.h>

#define taskENTER_CRITICAL() ::HostRTOS::EnterCritical()
#define taskEXIT_CRITICAL() ::HostRTOS::ExitCritical()

#define portYIELD() ::HostRTOS::Yield()

using EventBits_t = uint32_t;
