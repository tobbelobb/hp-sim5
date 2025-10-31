#pragma once

// Minimal FreeRTOS configuration values for the host build. The shim runtime in
// host/rtos/freertos_shim.cpp does not consult most of these values but a
// number of firmware headers expect them to exist.

#define configCPU_CLOCK_HZ                          (100000000UL)
#define configTICK_RATE_HZ                          (1000)
#define configUSE_PREEMPTION                        1
#define configUSE_TIME_SLICING                      1
#define configUSE_TICKLESS_IDLE                     0
#define configMAX_PRIORITIES                        8
#define configMINIMAL_STACK_SIZE                    256
#define configMAX_TASK_NAME_LEN                     16
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       8
#define configQUEUE_REGISTRY_SIZE                   0
#define configENABLE_BACKWARD_COMPATIBILITY         0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS     0
#define configSUPPORT_STATIC_ALLOCATION             1
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0
#define configCHECK_FOR_STACK_OVERFLOW              0
#define configUSE_MALLOC_FAILED_HOOK                0
#define configUSE_TRACE_FACILITY                    0
#define configUSE_MUTEXES                           1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_TIMERS                            1
#define configTIMER_TASK_PRIORITY                   3
#define configTIMER_QUEUE_LENGTH                    10
#define configTIMER_TASK_STACK_DEPTH                256
#define configTOTAL_HEAP_SIZE                       (2 * 1024 * 1024)

// Interrupt priority related definitions are effectively unused on host.
#define configKERNEL_INTERRUPT_PRIORITY             0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        0
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY     0

// Optional API inclusion switches expected by upstream headers.
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskCleanUpResources               0
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_xTaskGetIdleTaskHandle              1
#define INCLUDE_xSemaphoreGetMutexHolder            1
#define INCLUDE_xTimerPendFunctionCall              1
#define INCLUDE_uxTaskGetStackHighWaterMark2        1
