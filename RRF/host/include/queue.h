#pragma once

#include "FreeRTOS.h"

using QueueHandle_t = void*;

inline QueueHandle_t xQueueCreateStatic(UBaseType_t,
										UBaseType_t,
										uint8_t*,
										StaticQueue_t*) noexcept
{
	return nullptr;
}

inline BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t) noexcept
{
	return pdPASS;
}

inline BaseType_t xQueueSendFromISR(QueueHandle_t, const void*, BaseType_t*) noexcept
{
	return pdPASS;
}

inline BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t) noexcept
{
	return pdFAIL;
}

inline UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t) noexcept
{
	return 0;
}

inline void vQueueDelete(QueueHandle_t) noexcept {}
