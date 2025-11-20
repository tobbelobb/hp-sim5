Queue Fill Path

  - The DDA ring (“queue”) is filled inside Move::MoveLoop when the Move task pulls RawMoves from GCodes and immediately pushes them into the ring (rings[0].AddStandardMove). That loop lives in RRF/ReprapFirmware/src/Movement/Move.cpp:684-770.
  gcodes.ReadMove(...) is called in the Move task thread, so as soon as GCodes has parsed another block the Move task copies the transformed coordinates and enqueues them.
  - The Move task itself is a FreeRTOS task backed by a host thread created in Move::Init() via moveTask.Create(MoveStart, ...) (RRF/ReprapFirmware/src/Movement/Move.cpp:600-616). On the host build the FreeRTOS shim turns that into a detached std::thread
  (RRF/host/rtos/freertos_shim.cpp:212-240), so Move::MoveLoop (and therefore every DDARing::Spin invocation) always executes on that dedicated Move thread.

  Commit Conditions

  - DDARing::Spin calls PrepareMoves whenever the current move is committed but more moves are only provisionally planned (cdda->IsProvisional()). The inner loop in PrepareMoves keeps preparing until:
    firstUnpreparedMove->IsProvisional() AND moveTimeLeft < prepareAdvanceTime AND “already prepared” < half the ring size (plus CAN checks) (RRF/ReprapFirmware/src/Movement/DDARing.cpp:468-505). That “time left” test is what keeps the queue from
  draining; when virtual time advances enough that moveTimeLeft drops under the requested guard band (prepareAdvanceTime), another move gets committed.
  - The actual state flip happens during DDA::Prepare. After the move’s segments are generated and any CAN payload is queued, the code writes state = committed (RRF/ReprapFirmware/src/Movement/DDA.cpp:1124-1392, see especially lines 1339-1389). So the
  Move task thread both decides when to prepare and performs the commit.

  Timeline & Clocks

  - When a commit is about to happen, DDA::Prepare reads the current movement timer (now = StepTimer::GetMovementTimerTicks()) and schedules the move start time either right after the previous committed move or now + prepareAdvanceTime (RRF/
  ReprapFirmware/src/Movement/DDA.cpp:1151-1175). Those timestamps are in “step clocks”.
  - In the host build StepTimer::GetMovementTimerTicks() is just HostTiming::StepClocks64() (RRF/host/include/Movement/StepTimer.h:52-63). The virtual counter advances only when the firmware calls HostTiming::AdvanceStepClocks, which the Move
  thread does opportunistically while spinning (RRF/ReprapFirmware/src/Movement/DDARing.cpp:334-345,417-422). Each call bumps the counter by the requested number of ticks (StepClockRate = 48 MHz / 64 = 750 kHz, defined in RRF/ReprapFirmware/src/
  RepRapFirmware.h:646), i.e. 1 tick ≈ 1.333 µs of simulated time.
  - “Real time” is just whatever pace the host threads happen to run. Because the Move task owns both queue filling (AddStandardMove) and committing (PrepareMoves/DDA::Prepare), the only wall-clock dependency is how quickly that thread runs.
  “Simulation time” (the step-clock timeline that determines moveStartTime, underruns, etc.) advances strictly through HostTiming::AdvanceStepClocks calls, so any pause that prevents the Move task from calling it allows the virtual clock to lag while
  prepareAdvanceTime keeps shrinking—eventually triggering the queue-drained condition you’re seeing.

