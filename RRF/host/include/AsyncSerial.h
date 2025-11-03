#pragma once

#include "CoreIO.h"
#include "Stream.h"

#include <cstddef>
#include <cstdint>

class AsyncSerial : public Stream
{
public:
    using InterruptCallbackFn = void (*)(AsyncSerial*) noexcept;
    using OnBeginFn = void (*)(AsyncSerial*) noexcept;
    using OnEndFn = void (*)(AsyncSerial*) noexcept;
    using OnTransmissionEndedFn = void (*)(CallbackParameter) noexcept;

    enum UARTModes
    {
        Mode_8N1,
        Mode_8E1,
        Mode_8O1,
        Mode_8M1,
        Mode_8S1,
    };

    struct Errors
    {
        uint32_t all{0};
    };

    AsyncSerial(void* = nullptr, int = 0, uint32_t = 0, size_t = 0, size_t = 0,
                OnBeginFn = nullptr, OnEndFn = nullptr) noexcept
    {
    }

    void begin(uint32_t, UARTModes = Mode_8N1) noexcept
    {
    }
    void end() noexcept
    {
    }
    int available() noexcept override
    {
        return 0;
    }
    int read() noexcept override
    {
        return -1;
    }
    void flush() noexcept override
    {
    }
    size_t write(uint8_t) noexcept override
    {
        return 1;
    }
    size_t write(const uint8_t*, size_t len) noexcept override
    {
        return len;
    }
    size_t canWrite() noexcept override
    {
        return 0;
    }

    void ClearTransmitBuffer() noexcept
    {
    }
    void ClearReceiveBuffer() noexcept
    {
    }
    void DisableTransmit() noexcept
    {
    }
    void EnableTransmit() noexcept
    {
    }

    void setInterruptPriority(uint32_t) noexcept
    {
    }
    uint32_t getInterruptPriority() noexcept
    {
        return 0;
    }

    InterruptCallbackFn SetInterruptCallback(InterruptCallbackFn f) noexcept
    {
        return f;
    }
    OnTransmissionEndedFn SetOnTxEndedCallback(OnTransmissionEndedFn fn,
                                               CallbackParameter) noexcept
    {
        return fn;
    }

    Errors GetAndClearErrors() noexcept
    {
        return Errors{};
    }
    void IrqHandler() noexcept
    {
    }
};
