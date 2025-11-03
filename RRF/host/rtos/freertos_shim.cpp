#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <RTOSIface/RTOSIface.h>
#include "host_rtos.h"
#include "HostTiming.h"

namespace
{
using Clock = std::chrono::steady_clock;
constexpr TickType_t TicksPerSecond = 1000;  // 1 tick == 1ms on host

struct TaskControlBlock
{
    TaskFunction_t entry{nullptr};
    void* parameters{nullptr};
    std::string name;
    UBaseType_t priority{0};
    TaskBase* taskBase{nullptr};
    StackType_t* stackBase{nullptr};
    uint32_t stackWords{0};
    std::thread nativeThread;
    std::mutex notifyMutex;
    std::condition_variable notifyCv;
    uint32_t notifyValue{0};
    bool notified{false};
    bool deleteRequested{false};
};

struct QueueControlBlock
{
    size_t capacity{0};
    size_t itemSize{0};
    std::deque<std::vector<uint8_t>> items;
    std::mutex mutex;
    std::condition_variable notEmpty;
    std::condition_variable notFull;
};

enum class SemaphoreKind : uint8_t
{
    Mutex,
    Binary
};

struct SemaphoreControlBlock
{
    explicit SemaphoreControlBlock(SemaphoreKind kindIn) noexcept : kind(kindIn)
    {
        if (kind == SemaphoreKind::Binary)
        {
            available = false;
        }
    }

    SemaphoreKind kind;
    std::recursive_timed_mutex mutex;  // used when kind == Mutex
    std::mutex binaryMutex;
    std::condition_variable binaryCv;
    bool available{true};
};

thread_local TaskControlBlock* currentTask = nullptr;
std::mutex tasksMutex;
std::vector<std::unique_ptr<TaskControlBlock>> allTasks;
std::recursive_mutex globalCriticalMutex;
std::atomic<BaseType_t> schedulerState{taskSCHEDULER_RUNNING};

TaskControlBlock* TaskFromHandle(TaskHandle_t handle) noexcept
{
    return static_cast<TaskControlBlock*>(handle);
}

QueueControlBlock* QueueFromHandle(QueueHandle_t handle) noexcept
{
    return static_cast<QueueControlBlock*>(handle);
}

SemaphoreControlBlock* SemaphoreFromHandle(SemaphoreHandle_t handle) noexcept
{
    return static_cast<SemaphoreControlBlock*>(handle);
}

std::chrono::milliseconds ToDuration(TickType_t ticks) noexcept
{
    if (ticks == portMAX_DELAY)
    {
        return std::chrono::milliseconds::max();
    }
    return std::chrono::milliseconds(
        static_cast<int64_t>(ticks * 1000ull / TicksPerSecond));
}

void RemoveTask(TaskControlBlock* tcb) noexcept
{
    std::lock_guard<std::mutex> lock(tasksMutex);
    auto it = std::find_if(allTasks.begin(), allTasks.end(),
                           [tcb](const std::unique_ptr<TaskControlBlock>& ptr) noexcept
                           { return ptr.get() == tcb; });
    if (it != allTasks.end())
    {
        allTasks.erase(it);
    }
}
}  // anonymous namespace

TickType_t xTaskGetTickCount() noexcept
{
    // Use virtual clock for deterministic simulation
    return static_cast<TickType_t>(HostTiming::Millis());
}

TickType_t xTaskGetTickCountFromISR() noexcept
{
    return xTaskGetTickCount();
}

UBaseType_t uxTaskGetNumberOfTasks() noexcept
{
    std::lock_guard<std::mutex> lock(tasksMutex);
    return static_cast<UBaseType_t>(allTasks.size());
}

void vTaskDelay(const TickType_t ticksToDelay) noexcept
{
    if (ticksToDelay == 0)
    {
        std::this_thread::yield();
        return;
    }
    // Use virtual delay for deterministic simulation
    // Convert ticks to milliseconds (1 tick = 1ms)
    uint32_t delayMs = static_cast<uint32_t>(ticksToDelay);
    HostTiming::DelayMilliseconds(delayMs);

    // Still yield to allow other threads to run
    std::this_thread::yield();
}

void vTaskDelayUntil(TickType_t* const lastWakeTime,
                     const TickType_t ticksToWait) noexcept
{
    if (lastWakeTime == nullptr)
    {
        vTaskDelay(ticksToWait);
        return;
    }

    const TickType_t currentTicks = xTaskGetTickCount();
    TickType_t target = *lastWakeTime + ticksToWait;
    if (target <= currentTicks)
    {
        target = currentTicks + ticksToWait;
    }

    const TickType_t waitTicks = target > currentTicks ? (target - currentTicks) : 0;
    vTaskDelay(waitTicks);
    *lastWakeTime = target;
}

TaskHandle_t xTaskCreateStatic(TaskFunction_t function, const char* name,
                               uint32_t stackDepth, void* parameters,
                               UBaseType_t priority, StackType_t* stackBuffer,
                               StaticTask_t* taskBuffer) noexcept
{
    if (function == nullptr)
    {
        return nullptr;
    }

    auto task = std::make_unique<TaskControlBlock>();
    TaskControlBlock* raw = task.get();
    raw->entry = function;
    raw->parameters = parameters;
    raw->priority = priority;
    raw->name = (name != nullptr) ? name : "";
    raw->stackBase = stackBuffer;
    raw->stackWords = stackDepth;
    if (taskBuffer != nullptr)
    {
        raw->taskBase = reinterpret_cast<TaskBase*>(taskBuffer->hostContext);
    }

    {
        std::lock_guard<std::mutex> lock(tasksMutex);
        allTasks.emplace_back(std::move(task));
    }

    if (raw->taskBase != nullptr)
    {
        raw->taskBase->AttachHostHandle(static_cast<TaskHandle_t>(raw), raw->stackBase,
                                        stackDepth);
    }

    raw->nativeThread = std::thread(
        [raw]()
        {
            currentTask = raw;
            raw->entry(raw->parameters);
            raw->deleteRequested = true;
            currentTask = nullptr;
            RemoveTask(raw);
        });
    raw->nativeThread.detach();
    return static_cast<TaskHandle_t>(raw);
}

void vTaskDelete(TaskHandle_t handle) noexcept
{
    TaskControlBlock* tcb = TaskFromHandle(handle);
    if (tcb == nullptr)
    {
        return;
    }
    tcb->deleteRequested = true;
}

void vTaskSuspend(TaskHandle_t) noexcept
{
    // Not required for cooperative host shim
}

void vTaskResume(TaskHandle_t) noexcept
{
    // Not required for cooperative host shim
}

BaseType_t xTaskNotify(TaskHandle_t handle, uint32_t value, uint32_t) noexcept
{
    TaskControlBlock* tcb = TaskFromHandle(handle);
    if (tcb == nullptr)
    {
        return pdFAIL;
    }

    {
        std::lock_guard<std::mutex> lock(tcb->notifyMutex);
        tcb->notifyValue = value;
        tcb->notified = true;
    }
    tcb->notifyCv.notify_one();
    return pdPASS;
}

BaseType_t xTaskNotifyFromISR(TaskHandle_t handle, uint32_t value, uint32_t action,
                              BaseType_t* higherPriorityTaskWoken) noexcept
{
    if (higherPriorityTaskWoken != nullptr)
    {
        *higherPriorityTaskWoken = pdFALSE;
    }
    return xTaskNotify(handle, value, action);
}

BaseType_t xTaskNotifyWait(uint32_t bitsToClearOnEntry, uint32_t bitsToClearOnExit,
                           uint32_t* receivedValue, TickType_t timeout) noexcept
{
    TaskControlBlock* tcb = currentTask;
    if (tcb == nullptr)
    {
        return pdFAIL;
    }

    std::unique_lock<std::mutex> lock(tcb->notifyMutex);
    tcb->notifyValue &= ~bitsToClearOnEntry;

    const auto duration = ToDuration(timeout);
    if (!tcb->notified)
    {
        if (timeout == 0)
        {
            return pdFAIL;
        }
        if (duration == std::chrono::milliseconds::max())
        {
            tcb->notifyCv.wait(lock, [tcb]() noexcept { return tcb->notified; });
        }
        else if (!tcb->notifyCv.wait_for(lock, duration,
                                         [tcb]() noexcept { return tcb->notified; }))
        {
            return pdFAIL;
        }
    }

    if (receivedValue != nullptr)
    {
        *receivedValue = tcb->notifyValue;
    }
    tcb->notifyValue &= ~bitsToClearOnExit;
    tcb->notified = false;
    return pdPASS;
}

void vTaskNotifyGiveFromISR(TaskHandle_t handle,
                            BaseType_t* higherPriorityTaskWoken) noexcept
{
    (void)xTaskNotify(handle, 1, 0);
    if (higherPriorityTaskWoken != nullptr)
    {
        *higherPriorityTaskWoken = pdFALSE;
    }
}

void vTaskStartScheduler() noexcept
{
    schedulerState.store(taskSCHEDULER_RUNNING, std::memory_order_release);
}

const StackType_t* pxTaskGetLastStackTop(TaskHandle_t handle) noexcept
{
    TaskControlBlock* tcb = TaskFromHandle(handle);
    if (tcb == nullptr || tcb->stackBase == nullptr)
    {
        return nullptr;
    }
    return tcb->stackBase + tcb->stackWords;
}

TaskHandle_t xTaskGetCurrentTaskHandle() noexcept
{
    return static_cast<TaskHandle_t>(currentTask);
}

BaseType_t xTaskGetSchedulerState() noexcept
{
    return schedulerState.load(std::memory_order_acquire);
}

QueueHandle_t xQueueCreateStatic(UBaseType_t queueLength, UBaseType_t itemSize, uint8_t*,
                                 StaticQueue_t* queueBuffer) noexcept
{
    auto queue = std::make_unique<QueueControlBlock>();
    QueueControlBlock* raw = queue.get();
    raw->capacity = queueLength;
    raw->itemSize = itemSize;

    if (queueBuffer != nullptr)
    {
        queueBuffer->hostContext = raw;
    }

    return static_cast<QueueHandle_t>(queue.release());
}

BaseType_t xQueueSend(QueueHandle_t handle, const void* item, TickType_t timeout) noexcept
{
    QueueControlBlock* queue = QueueFromHandle(handle);
    if (queue == nullptr || item == nullptr)
    {
        return pdFAIL;
    }

    std::unique_lock<std::mutex> lock(queue->mutex);
    auto canWrite = [queue]() noexcept { return queue->items.size() < queue->capacity; };
    if (!canWrite())
    {
        if (timeout == 0)
        {
            return pdFAIL;
        }
        const auto duration = ToDuration(timeout);
        if (duration == std::chrono::milliseconds::max())
        {
            queue->notFull.wait(lock, canWrite);
        }
        else if (!queue->notFull.wait_for(lock, duration, canWrite))
        {
            return pdFAIL;
        }
    }

    std::vector<uint8_t> payload(queue->itemSize);
    std::memcpy(payload.data(), item, queue->itemSize);
    queue->items.push_back(std::move(payload));
    lock.unlock();
    queue->notEmpty.notify_one();
    return pdPASS;
}

BaseType_t xQueueSendFromISR(QueueHandle_t handle, const void* item,
                             BaseType_t* higherPriorityTaskWoken) noexcept
{
    if (higherPriorityTaskWoken != nullptr)
    {
        *higherPriorityTaskWoken = pdFALSE;
    }
    return xQueueSend(handle, item, portMAX_DELAY);
}

BaseType_t xQueueReceive(QueueHandle_t handle, void* buffer, TickType_t timeout) noexcept
{
    QueueControlBlock* queue = QueueFromHandle(handle);
    if (queue == nullptr || buffer == nullptr)
    {
        return pdFAIL;
    }

    std::unique_lock<std::mutex> lock(queue->mutex);
    auto hasItem = [queue]() noexcept { return !queue->items.empty(); };
    if (!hasItem())
    {
        if (timeout == 0)
        {
            return pdFAIL;
        }
        const auto duration = ToDuration(timeout);
        if (duration == std::chrono::milliseconds::max())
        {
            queue->notEmpty.wait(lock, hasItem);
        }
        else if (!queue->notEmpty.wait_for(lock, duration, hasItem))
        {
            return pdFAIL;
        }
    }

    std::vector<uint8_t> payload = std::move(queue->items.front());
    queue->items.pop_front();
    std::memcpy(buffer, payload.data(), queue->itemSize);
    lock.unlock();
    queue->notFull.notify_one();
    return pdPASS;
}

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t handle) noexcept
{
    QueueControlBlock* queue = QueueFromHandle(handle);
    if (queue == nullptr)
    {
        return 0;
    }
    std::lock_guard<std::mutex> lock(queue->mutex);
    return static_cast<UBaseType_t>(queue->items.size());
}

void vQueueDelete(QueueHandle_t handle) noexcept
{
    auto* queue = QueueFromHandle(handle);
    delete queue;
}

SemaphoreHandle_t xSemaphoreCreateMutex() noexcept
{
    return static_cast<SemaphoreHandle_t>(
        new SemaphoreControlBlock(SemaphoreKind::Mutex));
}

SemaphoreHandle_t xSemaphoreCreateBinary() noexcept
{
    return static_cast<SemaphoreHandle_t>(
        new SemaphoreControlBlock(SemaphoreKind::Binary));
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) noexcept
{
    SemaphoreControlBlock* sem = SemaphoreFromHandle(handle);
    if (sem == nullptr)
    {
        return pdFAIL;
    }

    if (sem->kind == SemaphoreKind::Mutex)
    {
        sem->mutex.unlock();
        return pdPASS;
    }

    {
        std::lock_guard<std::mutex> lock(sem->binaryMutex);
        sem->available = true;
    }
    sem->binaryCv.notify_one();
    return pdPASS;
}

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t handle,
                                 BaseType_t* higherPriorityTaskWoken) noexcept
{
    if (higherPriorityTaskWoken != nullptr)
    {
        *higherPriorityTaskWoken = pdFALSE;
    }
    return xSemaphoreGive(handle);
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout) noexcept
{
    SemaphoreControlBlock* sem = SemaphoreFromHandle(handle);
    if (sem == nullptr)
    {
        return pdFAIL;
    }

    if (sem->kind == SemaphoreKind::Mutex)
    {
        if (timeout == 0)
        {
            return sem->mutex.try_lock() ? pdPASS : pdFAIL;
        }
        if (timeout == portMAX_DELAY)
        {
            sem->mutex.lock();
            return pdPASS;
        }
        const auto duration = ToDuration(timeout);
        return sem->mutex.try_lock_for(duration) ? pdPASS : pdFAIL;
    }

    std::unique_lock<std::mutex> lock(sem->binaryMutex);
    auto available = [sem]() noexcept { return sem->available; };
    if (!available())
    {
        if (timeout == 0)
        {
            return pdFAIL;
        }
        const auto duration = ToDuration(timeout);
        if (duration == std::chrono::milliseconds::max())
        {
            sem->binaryCv.wait(lock, available);
        }
        else if (!sem->binaryCv.wait_for(lock, duration, available))
        {
            return pdFAIL;
        }
    }
    sem->available = false;
    return pdPASS;
}

void vSemaphoreDelete(SemaphoreHandle_t handle) noexcept
{
    auto* sem = SemaphoreFromHandle(handle);
    delete sem;
}

namespace HostRTOS
{
void EnterCritical() noexcept
{
    globalCriticalMutex.lock();
}

void ExitCritical() noexcept
{
    globalCriticalMutex.unlock();
}

void Yield() noexcept
{
    std::this_thread::yield();
}

TaskBase* GetCurrentTaskBase() noexcept
{
    return (currentTask != nullptr) ? currentTask->taskBase : nullptr;
}
}  // namespace HostRTOS
