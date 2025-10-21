#pragma once

#include "FreeRTOS.h"

using QueueHandle_t = void*;

QueueHandle_t xQueueCreateStatic(UBaseType_t queueLength,
								 UBaseType_t itemSize,
								 uint8_t* queueStorage,
								 StaticQueue_t* queueBuffer) noexcept;
BaseType_t xQueueSend(QueueHandle_t queue, const void* item, TickType_t timeout) noexcept;
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void* item, BaseType_t* higherPriorityTaskWoken) noexcept;
BaseType_t xQueueReceive(QueueHandle_t queue, void* buffer, TickType_t timeout) noexcept;
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue) noexcept;
void vQueueDelete(QueueHandle_t queue) noexcept;
