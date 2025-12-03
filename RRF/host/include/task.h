#pragma once

#include "FreeRTOS.h"

#include <cstddef>

using TaskHandle_t = void*;
using TaskFunction_t = void (*)(void*);

constexpr UBaseType_t tskDEFAULT_INDEX_TO_NOTIFY = 0;

enum eNotifyAction
{
    eNoAction = 0,
    eSetBits,
    eIncrement,
    eSetValueWithOverwrite,
    eSetValueWithoutOverwrite
};

TickType_t xTaskGetTickCount() noexcept;
TickType_t xTaskGetTickCountFromISR() noexcept;
UBaseType_t uxTaskGetNumberOfTasks() noexcept;
void vTaskDelay(TickType_t ticksToDelay) noexcept;
void vTaskDelayUntil(TickType_t* lastWakeTime, TickType_t ticksToWait) noexcept;
TaskHandle_t xTaskCreateStatic(TaskFunction_t function, const char* name,
                               uint32_t stackDepth, void* parameters,
                               UBaseType_t priority, StackType_t* stackBuffer,
                               StaticTask_t* taskBuffer) noexcept;
void vTaskDelete(TaskHandle_t handle) noexcept;
void vTaskSuspend(TaskHandle_t handle) noexcept;
void vTaskResume(TaskHandle_t handle) noexcept;
BaseType_t xTaskNotify(TaskHandle_t handle, uint32_t value, uint32_t action) noexcept;
BaseType_t xTaskNotifyFromISR(TaskHandle_t handle, uint32_t value, uint32_t action,
                              BaseType_t* higherPriorityTaskWoken) noexcept;
BaseType_t xTaskNotifyWait(uint32_t bitsToClearOnEntry, uint32_t bitsToClearOnExit,
                           uint32_t* receivedValue, TickType_t timeout) noexcept;
BaseType_t xTaskGenericNotify(TaskHandle_t handle, UBaseType_t index, uint32_t value,
                              eNotifyAction action, uint32_t* previousValue) noexcept;
BaseType_t xTaskGenericNotifyFromISR(TaskHandle_t handle, UBaseType_t index, uint32_t value,
                                     eNotifyAction action, uint32_t* previousValue,
                                     BaseType_t* higherPriorityTaskWoken) noexcept;
uint32_t ulTaskGenericNotifyTake(UBaseType_t index, BaseType_t clearCountOnExit,
                                 TickType_t ticksToWait) noexcept;
void vTaskNotifyGiveFromISR(TaskHandle_t handle,
                            BaseType_t* higherPriorityTaskWoken) noexcept;
void vTaskStartScheduler() noexcept;
const StackType_t* pxTaskGetLastStackTop(TaskHandle_t handle) noexcept;
TaskHandle_t xTaskGetCurrentTaskHandle() noexcept;
BaseType_t xTaskGetSchedulerState() noexcept;

constexpr BaseType_t taskSCHEDULER_NOT_STARTED = 0;
constexpr BaseType_t taskSCHEDULER_RUNNING = 1;
constexpr BaseType_t taskSCHEDULER_SUSPENDED = 2;

#define xTaskNotifyGiveIndexed(handle, index) \
    xTaskGenericNotify((handle), (index), 0, eIncrement, nullptr)
#define vTaskNotifyGiveIndexedFromISR(handle, index, higherPriorityTaskWoken) \
    xTaskGenericNotifyFromISR((handle), (index), 0, eIncrement, nullptr, (higherPriorityTaskWoken))
#define ulTaskNotifyTakeIndexed(index, clearCountOnExit, ticksToWait) \
    ulTaskGenericNotifyTake((index), (clearCountOnExit), (ticksToWait))
