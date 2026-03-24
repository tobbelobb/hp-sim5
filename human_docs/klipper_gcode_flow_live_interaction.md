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
State: klipper_parsed mirror, klipper_clock samples"]
    wsExternal["WebSocket ws://localhost:8790
State: klipper_parsed / klipper_clock raw stream"]
    klipperRaw["klipperPacer worker via connectKlipperRaw
State: MCU clock model, queue_step pacing, Move command emission"]

    subgraph Browser http://localhost:5173/hp-sim5/hp-sim-3d/?gcode_ws=ws://localhost:8790
        hpSim["hp-sim-3d/assets/hp-sim.js
State: raw Klipper websocket, pacing worker hookup"]
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
    mcuBridge -->|klipper_parsed JSON| wsInternal --> wsExternal --> klipperRaw
    mcuBridge -->|klipper_clock JSON| wsInternal --> wsExternal --> klipperRaw
    klipperRaw -->|timed Move commands| hpSim -->|addCommand| remote --> stepper --> ui
    hpSim -->|updates view + HUD| ui
```
