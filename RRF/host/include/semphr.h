#pragma once

#include "FreeRTOS.h"

using SemaphoreHandle_t = void*;

SemaphoreHandle_t xSemaphoreCreateMutex() noexcept;
SemaphoreHandle_t xSemaphoreCreateBinary() noexcept;
BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) noexcept;
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t handle, BaseType_t* higherPriorityTaskWoken) noexcept;
BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout) noexcept;
void vSemaphoreDelete(SemaphoreHandle_t handle) noexcept;
