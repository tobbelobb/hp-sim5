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

enum class TaskState : uint8_t
{
    Ready,
    Running,
    Blocked,
    Deleted
};

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
    std::condition_variable schedulerCv;
    uint32_t notifyValue{0};
    bool notified{false};
    bool deleteRequested{false};
    TaskState state{TaskState::Ready};
    TickType_t wakeTime{0};
    size_t creationOrder{0};  // For deterministic scheduling
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

// Cooperative scheduler state for deterministic execution
std::mutex executionMutex;  // Only one task can hold this at a time
std::condition_variable schedulerCondition;
TaskControlBlock* runningTask = nullptr;
size_t nextCreationOrder = 0;

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

// Cooperative yield: release execution and let scheduler pick next task
void CooperativeYield() noexcept
{
    TaskControlBlock* previousTask = currentTask;
    currentTask = nullptr;

    // Critical section: atomically update state and find next task
    std::lock_guard<std::mutex> lock(tasksMutex);

    if (previousTask != nullptr)
    {
        // Mark previous task as ready (unless it was explicitly blocked)
        if (previousTask->state == TaskState::Running)
        {
            previousTask->state = TaskState::Ready;
        }
    }

    // Find next ready task (already holding tasksMutex)
    // Check for tasks that should wake up based on time
    TickType_t now = static_cast<TickType_t>(HostTiming::Millis());
    for (const auto& task : allTasks)
    {
        if (task->state == TaskState::Blocked && task->wakeTime <= now)
        {
            task->state = TaskState::Ready;
        }
    }

    // Find the ready task with lowest creation order (FIFO)
    TaskControlBlock* nextTask = nullptr;
    size_t lowestOrder = SIZE_MAX;

    for (const auto& task : allTasks)
    {
        if (task->state == TaskState::Ready && task->creationOrder < lowestOrder)
        {
            nextTask = task.get();
            lowestOrder = task->creationOrder;
        }
    }

    if (nextTask != nullptr)
    {
        nextTask->state = TaskState::Running;
        runningTask = nextTask;

        // Wake only the specific next task
        nextTask->schedulerCv.notify_one();
    }

    // Release execution lock after setting up next task
    executionMutex.unlock();
}

// Wait for our turn to execute
void WaitForExecutionToken(TaskControlBlock* task) noexcept
{
    std::unique_lock<std::mutex> lock(tasksMutex);

    // Wait until it's our turn
    while (runningTask != task)
    {
        task->schedulerCv.wait(lock);
    }

    // Now acquire the execution lock
    executionMutex.lock();
    currentTask = task;
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
    TaskControlBlock* thisTask = currentTask;
    if (thisTask == nullptr)
    {
        return;
    }

    if (ticksToDelay == 0)
    {
        // Just yield to next task
        CooperativeYield();
        WaitForExecutionToken(thisTask);
        return;
    }

    // Advance virtual time
    uint32_t delayMs = static_cast<uint32_t>(ticksToDelay);
    HostTiming::DelayMilliseconds(delayMs);

    // Block task until wake time
    thisTask->state = TaskState::Blocked;
    thisTask->wakeTime = xTaskGetTickCount() + ticksToDelay;

    // Yield to next ready task
    CooperativeYield();
    WaitForExecutionToken(thisTask);
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
    raw->state = TaskState::Ready;
    raw->creationOrder = nextCreationOrder++;

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
            // Wait for our turn in the cooperative scheduler
            WaitForExecutionToken(raw);

            // Run the task
            raw->entry(raw->parameters);

            // Task finished, mark as deleted and yield
            raw->deleteRequested = true;
            raw->state = TaskState::Deleted;
            CooperativeYield();
            RemoveTask(raw);
        });
    raw->nativeThread.detach();

    // If this is the first task, give it the execution token
    bool isFirstTask = false;
    {
        std::lock_guard<std::mutex> lock(tasksMutex);
        isFirstTask = (allTasks.size() == 1);
    }
    if (isFirstTask && runningTask == nullptr)
    {
        runningTask = raw;
        raw->state = TaskState::Running;
        raw->schedulerCv.notify_one();
    }

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

        // If task was blocked on notification, make it ready
        if (tcb->state == TaskState::Blocked)
        {
            tcb->state = TaskState::Ready;
        }
    }
    tcb->notifyCv.notify_one();
    // Note: task will be scheduled deterministically when current task yields
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
    TaskControlBlock* thisTask = currentTask;
    if (thisTask == nullptr)
    {
        return pdFAIL;
    }

    {
        std::lock_guard<std::mutex> lock(thisTask->notifyMutex);
        thisTask->notifyValue &= ~bitsToClearOnEntry;
    }

    // If not already notified, block and yield
    while (!thisTask->notified)
    {
        if (timeout == 0)
        {
            return pdFAIL;
        }

        // Block task and yield to others
        thisTask->state = TaskState::Blocked;
        if (timeout != portMAX_DELAY)
        {
            thisTask->wakeTime = xTaskGetTickCount() + timeout;
        }

        CooperativeYield();
        WaitForExecutionToken(thisTask);

        // Check if we timed out
        if (timeout != portMAX_DELAY && !thisTask->notified)
        {
            if (xTaskGetTickCount() >= thisTask->wakeTime)
            {
                return pdFAIL;
            }
        }
    }

    std::lock_guard<std::mutex> lock(thisTask->notifyMutex);
    if (receivedValue != nullptr)
    {
        *receivedValue = thisTask->notifyValue;
    }
    thisTask->notifyValue &= ~bitsToClearOnExit;
    thisTask->notified = false;
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

    TickType_t startTime = xTaskGetTickCount();

    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(queue->mutex);
            if (queue->items.size() < queue->capacity)
            {
                // Can write now
                std::vector<uint8_t> payload(queue->itemSize);
                std::memcpy(payload.data(), item, queue->itemSize);
                queue->items.push_back(std::move(payload));
                queue->notEmpty.notify_one();
                return pdPASS;
            }
        }

        // Queue full, check timeout
        if (timeout == 0)
        {
            return pdFAIL;
        }

        if (timeout != portMAX_DELAY)
        {
            TickType_t elapsed = xTaskGetTickCount() - startTime;
            if (elapsed >= timeout)
            {
                return pdFAIL;
            }
        }

        // Yield and try again
        TaskControlBlock* thisTask = currentTask;
        CooperativeYield();
        WaitForExecutionToken(thisTask);
    }
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

    TickType_t startTime = xTaskGetTickCount();

    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(queue->mutex);
            if (!queue->items.empty())
            {
                // Can read now
                std::vector<uint8_t> payload = std::move(queue->items.front());
                queue->items.pop_front();
                std::memcpy(buffer, payload.data(), queue->itemSize);
                queue->notFull.notify_one();
                return pdPASS;
            }
        }

        // Queue empty, check timeout
        if (timeout == 0)
        {
            return pdFAIL;
        }

        if (timeout != portMAX_DELAY)
        {
            TickType_t elapsed = xTaskGetTickCount() - startTime;
            if (elapsed >= timeout)
            {
                return pdFAIL;
            }
        }

        // Yield and try again
        TaskControlBlock* thisTask = currentTask;
        CooperativeYield();
        WaitForExecutionToken(thisTask);
    }
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
