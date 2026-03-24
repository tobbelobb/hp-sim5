```mermaid
graph LR
    subgraph Terminal Session
        term["Terminal interface (scripts/klipper_gcode_bridge.mjs)
State: readline prompt, currentGcode, sequential stdin queue"]
        apiBridge["KlipperApiBridge
State: API socket /tmp/klippy_uds, request ids, pending responses"]
    end

    subgraph Klipper Stack
        klippy["klippy.py
State: host command queue, gcode/script executor, webhooks readiness"]
        mcuBridge["klipper_linux_mcu_bridge.py
State: PTY forwarder, msgproto parser, parsed-line mirror, clock sync"]
        klipperMcu["klipper_mcu
State: step generation, host->MCU packets, queue_step output"]
        hostPty["/tmp/klipper_host_mcu
State: Klippy-facing PTY symlink"]
        rawPty["/tmp/klipper_host_mcu_raw-<pid>
State: klipper_mcu raw PTY"]
    end

    wsInternal["WebSocket ws://127.0.0.1:8770
State: klipper_parsed packets, klipper_clock samples"]
    motionRelay["createKlipperMotionRelay + KlipperCommander
State: axis mapping, bucketized queue_step -> Move commands, ASAP mode"]
    wsExternal["WebSocket ws://localhost:8790
State: {type:'command', command, gcode} payloads"]

    subgraph Browser http://localhost:5173/hp-sim5/hp-sim-3d/?gcode_ws=ws://localhost:8790
        hpSim["hp-sim/assets/hp-sim.js or hp-sim-3d/assets/hp-sim.js
State: externalCommandSocket, externalCommandQueue"]
        remote["RemoteSpoolSystem
State: queue of Move commands, playback state, ASAP toggle"]
        stepper["StepperMotorSystem + StepperMotorComponent
State per axis: commandedAngle, torqueMode, targetTorque"]
        ui["Canvas/UI
State: machine view, HUD, speed controls"]
    end

    term -->|G-code via stdin or --cmd| apiBridge
    apiBridge -->|gcode/script| klippy
    klippy -->|host traffic| hostPty --> mcuBridge
    mcuBridge -->|bytes| rawPty --> klipperMcu
    klipperMcu -->|parsed host->MCU packets| mcuBridge
    mcuBridge -->|klipper_parsed JSON| wsInternal --> motionRelay
    mcuBridge -->|klipper_clock JSON| wsInternal
    motionRelay -->|Move commands| wsExternal --> hpSim
    hpSim -->|pushExternalCommands/addCommand| remote --> stepper --> ui
```
