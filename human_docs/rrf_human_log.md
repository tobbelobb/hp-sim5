# Log Oct 25, 2025

## Tooling
I made RRF/host/rrg.sh to search for definitions in ReprapFirmware and host code.

I also removed `-Wall -Wextra -pedantic` from CXXFLAGS in the Makefile, just to have less
noise when working on this.

I used the following command to try and compile the real Move.cpp:
```bash
cd RRF/host
g++ -fdiagnostics-color=always -std=gnu++17 -pthread -I. -Iinclude -Irtos -I../ReprapFirmware/src -I../ReprapFirmware/src/Hardware/SAME70 -I../RRFLibraries/src -I../CANlib/src -D__SAME70Q20B__ -DDUET3_MB6HC=1 -DDUET3=1 -DSAME70=1 -DRRF_HOST_BUILD=1 -DSRC_MOVEMENT_STEPTIMER_H_=1 -DUSE_SBC=0 -c ../ReprapFirmware/src/Movement/Move.cpp -o build/rrf_Movement/Move.o |& head -n 20
```

I used the following to get a rough idea if anything changed, or if errors exploded or shrinked after a change:

```bash
cd RRF/host
g++ -fdiagnostics-color=always -std=gnu++17 -pthread -I. -Iinclude -Irtos -I../ReprapFirmware/src -I../ReprapFirmware/src/Hardware/SAME70 -I../RRFLibraries/src -I../CANlib/src -D__SAME70Q20B__ -DDUET3_MB6HC=1 -DDUET3=1 -DSAME70=1 -DRRF_HOST_BUILD=1 -DSRC_MOVEMENT_STEPTIMER_H_=1 -DUSE_SBC=0 -c ../ReprapFirmware/src/Movement/Move.cpp -o build/rrf_Movement/Move.o |& wc -l
```

To look for definitions in ../ReprapFirmware I did a lot of
```
git -C ../ReprapFirmware/ grep SomeVariableOrClass
```

## Code Changes
I first added a forward declaration for ZProbe in host/include/Endstops/EndstopDefs.h
But later I renamed it to host/include/Endstops/EndstopDefsHost.h so that nothing includes it.
That way, the real EndstopDefs.h gets included instead, since it's not shadowed anymore.

I actually "unshadowed" four real headed files by renaming my shadowing headers:
```
 rename RRF/host/include/Endstops/{EndstopDefs.h => EndstopDefsHost.h} (97%)
 rename RRF/host/include/GCodes/{GCodes.h => GCodesHost.h} (100%)
 rename RRF/host/include/Movement/{DDA.h => DDAHost_previouslyDDA.h} (100%)
 rename RRF/host/include/Movement/{Move.h => MoveHost.h} (100%)
```

In my host/include/Platform/RepRap.h I then got a circular include, which I fixed by
no longer including <GCodes/GCodes.h>, and instead doing a forward declaration and
using only pointers to GCodes, not GCodes objects by value inside my RepRap.h.
I also used pointers to neighboring member variables:
```cpp
       Platform* platform;
       Move* move;
       GCodes* gCodes;
       Heat* heat;
       PrintMonitor* printMonitor;
       FansManager* fansManager;
       GlobalVariables* globalVariables;
```
I kept my RepRap.h instead of renaming it because it's part of the Platform part of the code.
My thinking is that I should probably shadow the whole Platform, since I'm on x86_64?
I don't know for sure that it's the right approach but it's my first guess.

I created a new file RRF/host/include/Platform/TaskPriorities.h which makes one define
that's supposed to come from FreeRTOSConfig.h but I couldn't figure out how that was supposed to
happen, because the real TaskPriorities.h did not include FreeRTOSConfig.h itself.
It relied on its includer to first include FreeRTOSConfig.h I guess.
But FreeRTOS details are of no interest to us on x86_64, so I just stuck something in there
to silence the warnings:
```File: RRF/host/include/Platform/TaskPriorities.h
#pragma once

#define configMAX_PRIORITIES ( 8 )

#include_next <Platform/TaskPriorities.h>
```


