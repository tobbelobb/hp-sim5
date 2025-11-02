The way the real firmware uses time is fundamentally asynchronous and interrupt-driven, which is the opposite of a sequential, batch-mode program. Faking this layer correctly is the key to the entire project.

Let's do a deep dive into the firmware's timekeeping and then lay out the optimal simulation strategy.

### Part 1: How Timers Work in the Real Firmware

ReprapFirmware uses two distinct types of timers for different purposes:

1.  **The High-Frequency Hardware Step Timer (`StepTimer.h`)**
    *   **Purpose:** This timer's one and only job is to generate the extremely precise, high-frequency electrical pulses that are sent to the stepper motor drivers. The timing of these pulses directly controls the speed and acceleration of the motors. It is the heart of all physical movement.
    *   **Mechanism:**
        *   It commandeers a hardware timer peripheral on the microcontroller (e.g., a Timer/Counter on SAM E70 processors).
        *   This timer is configured to tick at a very high frequency, typically several megahertz (e.g., every 0.25 microseconds).
        *   An **Interrupt Service Routine (ISR)**, the `StepTimer::ISR()`, is attached to this timer. This is a special, high-priority function that the CPU executes every time the hardware timer ticks.
        *   Inside the ISR, a tight loop checks if it's time to create a "step" pulse for any of the motors currently moving. It also handles the "direction" pin logic.
    *   **Key Concept:** All physical motion is driven by this high-priority interrupt, running completely in the background and asynchronously from the main G-code processing tasks.

2.  **The Low-Frequency System Clock (`Platform.h`)**
    *   **Purpose:** This provides general-purpose timekeeping for everything that *isn't* stepper pulse generation. This includes heater PID loops, checking for timeouts, logging, and handling delays.
    *   **Mechanism:** It's based on the Real-Time Operating System (FreeRTOS) "tick". FreeRTOS is typically configured to have a system tick every millisecond.
    *   **`Platform::millis()`**: This function simply returns a value derived from the FreeRTOS `xTaskGetTickCount()`. It has a resolution of 1 millisecond.
    *   **`Platform::micros()`**: This provides higher-resolution timing, often by reading the current count of the same high-frequency hardware timer used by the `StepTimer`, but without the interrupt context.

### Part 2: How G-Codes Depend on These Timers

1.  **`G0`/`G1` (Movement Commands)**
    *   The `GCodes` module parses the command and extracts the target coordinates and feedrate.
    *   It calls into the `Move` module, which prepares the move.
    *   The `Move` module creates a `DDA` (Digital Differential Analyzer) object for this specific move. The `DDA` is the crucial part: it performs all the physics calculations, determining the acceleration, constant velocity, and deceleration phases. **Crucially, it calculates the exact number of microseconds the entire move should take.**
    *   The prepared `DDA` is handed off to the `StepTimer`.
    *   The `GCodes` task then returns immediately, reporting that the command has been processed. It does **not** wait for the move to finish.
    *   The `StepTimer` ISR, ticking away in the background, now executes the `DDA`'s plan, generating step pulses at the correct intervals until the move is complete.

2.  **`G4` (Dwell/Wait Command)**
    *   The `GCodes` module parses the `P` (milliseconds) or `S` (seconds) parameter.
    *   It then calls a function like `Platform::Delay(milliseconds)`.
    *   Internally, `Platform::Delay` uses the FreeRTOS function `vTaskDelay()`. This **blocks** the G-code processing task. The operating system puts the task to sleep and will not wake it up to process the next G-code until the specified time has elapsed.

### Part 3: The Optimal Approach for the Host Simulator

The goal is to eliminate all real-time waiting while preserving the logical passage of time. The best way to do this is with a **Virtual Master Clock**.

**The Core Concept:** Create a single `uint64_t` variable in your host environment that represents the total elapsed time of the simulation in microseconds. All "timing" operations will simply manipulate this variable instead of actually waiting.

```cpp
// In your main simulation environment
uint64_t virtual_clock_micros = 0;
```

Here’s how you stub, shim, and fake the timer-dependent modules:

#### 1. Faking `Platform.h` (The System Clock)

Your fake `Platform.h` becomes an interface to the virtual clock. This is fast and maintains correctness.

*   **`uint32_t Platform::millis()`**:
    *   **Implementation:** `return virtual_clock_micros / 1000;`
*   **`uint64_t Platform::micros()`**:
    *   **Implementation:** `return virtual_clock_micros;`
*   **`void Platform::Delay(uint32_t ms)`**:
    *   **Implementation:** `virtual_clock_micros += (uint64_t)ms * 1000;`
    *   **This is the crucial change for `G4`**. Instead of sleeping for `ms` milliseconds, you instantly advance the virtual clock. The simulation of the wait is instantaneous, but the timeline is correctly updated.

#### 2. Faking `StepTimer.h` (The Motion)

This is the "batch mode" part. Since there is no hardware to pulse, the `StepTimer`'s job changes from "execute a move over time" to "calculate the time a move would have taken".

*   **Your `StepTimerHost::StartMove(DDA& dda)` function:**
    1.  **Instantly "Execute" the Move:** The `DDA` contains all the information about the move. You can now iterate through all the steps that *would* have been generated. This is where you would put your CAN capture logic. You can get the total number of steps for each axis directly from the `DDA` object.
    2.  **Calculate Move Duration:** The `DDA` has already calculated the total time the move will take. You can retrieve this with a function (you may need to expose it in the `DDA` class) like `dda.GetMoveDurationMicros()`.
    3.  **Advance the Virtual Clock:** This is the most important step. You advance the master clock by the entire duration of the move.
        ```cpp
        // Inside your fake StepTimer when a move is commanded
        uint32_t move_duration_micros = dda.GetCalculatedMoveTime();
        virtual_clock_micros += move_duration_micros;

        // Mark the move as "complete" immediately
        dda.MoveComplete();
        ```

### Summary: The Simulation Loop

Your simulator's main loop will now look like this:

1.  Initialize `virtual_clock_micros = 0;`.
2.  Initialize all the real RRF modules (`RepRap`, `GCodes`, `Move`, etc.) using your fake hardware interfaces.
3.  Read a line of G-code from a file.
4.  Feed it to the `GCodes` interpreter.
5.  The interpreter runs:
    *   If it's a **`G1` command**, it will eventually call your fake `StepTimer`, which will instantly "perform" the steps and add, for example, `150,000` microseconds to `virtual_clock_micros`.
    *   If it's a **`G4 P2000` command**, it will call your fake `Platform::Delay`, which will instantly add `2,000,000` microseconds to `virtual_clock_micros`.
    *   If it's an instantaneous command (e.g., `M114` Get Position), no time passes, and the clock does not advance.
6.  Loop to the next G-code line.

This approach gives you the best of both worlds:
*   **Speed:** The simulation runs as fast as your CPU can process the G-code logic, with no `sleep()` calls or real-time delays. A multi-hour print can be simulated in seconds.
*   **Correctness:** The virtual clock maintains a perfect, logical timeline. If you were to log events, their timestamps would be accurate relative to each other, reflecting exactly how long the real firmware would have taken to execute the job.
