#pragma once

#include "FreeRTOS.h"

using SemaphoreHandle_t = void*;

inline SemaphoreHandle_t xSemaphoreCreateMutex() noexcept
{
    return nullptr;
}

inline SemaphoreHandle_t xSemaphoreCreateBinary() noexcept
{
    return nullptr;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) noexcept
{
    return pdPASS;
}

inline BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t, BaseType_t*) noexcept
{
    return pdPASS;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) noexcept
{
    return pdPASS;
}

inline void vSemaphoreDelete(SemaphoreHandle_t) noexcept {}
