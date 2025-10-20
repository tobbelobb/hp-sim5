#pragma once

#include "FreeRTOS.h"

using TaskHandle_t = void*;
using TaskFunction_t = void (*)(void*);

inline TickType_t xTaskGetTickCount() noexcept
{
	return 0;
}

inline TickType_t xTaskGetTickCountFromISR() noexcept
{
	return 0;
}

inline UBaseType_t uxTaskGetNumberOfTasks() noexcept
{
	return 0;
}

inline void vTaskDelay(const TickType_t) noexcept {}
inline void vTaskDelayUntil(TickType_t*, TickType_t) noexcept {}

inline TaskHandle_t xTaskCreateStatic(TaskFunction_t,
									  const char*,
									  uint32_t,
									  void*,
									  UBaseType_t,
									  StackType_t*,
									  StaticTask_t*) noexcept
{
	return nullptr;
}

inline void vTaskDelete(TaskHandle_t) noexcept {}
inline void vTaskSuspend(TaskHandle_t) noexcept {}
inline void vTaskResume(TaskHandle_t) noexcept {}

inline BaseType_t xTaskNotify(TaskHandle_t, uint32_t, uint32_t) noexcept
{
	return pdPASS;
}

inline BaseType_t xTaskNotifyFromISR(TaskHandle_t, uint32_t, uint32_t, BaseType_t*) noexcept
{
	return pdPASS;
}

inline BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t*, TickType_t) noexcept
{
	return pdPASS;
}

inline void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*) noexcept {}

inline void vTaskStartScheduler() noexcept {}

inline const StackType_t* pxTaskGetLastStackTop(TaskHandle_t) noexcept
{
	return nullptr;
}
