#pragma once

#include <cstddef>
#include <cstdint>

using BaseType_t = int32_t;
using UBaseType_t = uint32_t;
using TickType_t = uint32_t;
using StackType_t = uint32_t;

struct StaticTask_t {};
struct StaticQueue_t {};
struct StaticSemaphore_t {};
struct StaticEventGroup_t {};

#define pdMS_TO_TICKS(x) static_cast<TickType_t>(x)
#define portMAX_DELAY 0xFFFFFFFFu
#define pdPASS 1
#define pdFAIL 0
#define pdTRUE 1
#define pdFALSE 0

#define configMINIMAL_STACK_SIZE 256

#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()

#define portYIELD()

using EventBits_t = uint32_t;
