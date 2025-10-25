I want to build a x86_64 version of ReprapFirmware (here: https://github.com/tobbelobb/ReprapFirmware/tree/6e292a37fec641a9d2de68bde3cdccb7ee407788)

I want to do it in the RRF/host directory of the hp-sim5 repo (here: https://github.com/tobbelobb/hp-sim5/tree/main/RRF/host)

Right now there's a web of stubs and shims that I want to untangle.

Take for example hp-sim5/RRF/host/src/main.cpp (here: https://raw.githubusercontent.com/tobbelobb/hp-sim5/refs/heads/main/RRF/host/src/main.cpp).
It does a
```cpp
#include <Movement/Move.h>
```

Which Move.h is even included then. Is it the
RRF/host/include/Movement/Move.h or the RRF/ReprapFirmware/src/Movement/Move.h that takes presedence?

Tracing this is complicated because the Makefile (here: https://raw.githubusercontent.com/tobbelobb/hp-sim5/refs/heads/main/RRF/host/Makefile)
does a `-I` in the CXXFLAGS of both directories (the host/include one is first though). Like this:
```
CXXFLAGS := -std=gnu++17 -Wall -Wextra -pedantic -pthread \
    -I. \
    -Iinclude \
    -Irtos \
    -I../ReprapFirmware/src \
    -I../ReprapFirmware/src/Hardware/SAME70 \
    -I../RRFLibraries/src \
    -I../CANlib/src \
	-D__SAME70Q20B__ \
	-DDUET3_MB6HC=1 \
	-DDUET3=1 \
	-DSAME70=1 \
	-DRRF_HOST_BUILD=1 \
    -DSRC_MOVEMENT_STEPTIMER_H_=1 \
    -DUSE_SBC=0
```

I tried renaming the local Movement dir (RRF/host/include/Movement) to just movement, with a lowercase m, to avoid naming confusion.
However, I didn't know which #include statements inside main.cpp (here: https://raw.githubusercontent.com/tobbelobb/hp-sim5/refs/heads/main/RRF/host/src/main.cpp) to change from "Movement" to "movement",
and my attempts at trial and error created an explosion of error messages so I just reverted it.

I want to work towards including and compiling the real ReprapFirmware Move.h (here:
https://raw.githubusercontent.com/tobbelobb/RepRapFirmware/6e292a37fec641a9d2de68bde3cdccb7ee407788/src/Movement/Move.h) and Move.cpp (here:
https://raw.githubusercontent.com/tobbelobb/RepRapFirmware/6e292a37fec641a9d2de68bde3cdccb7ee407788/src/Movement/Move.cpp).

My strategy is to first try and compile an object file from main.cpp, like this:
g++ -std=gnu++17 -pthread -I. -Iinclude -Irtos -I../ReprapFirmware/src -I../ReprapFirmware/src/Hardware/SAME70 -I../RRFLibraries/src -I../CANlib/src -D__SAME70Q20B__ -DDUET3_MB6HC=1 -DDUET3=1 -DSAME70=1 -DRRF_HOST_BUILD=1 -DSRC_MOVEMENT_STEPTIMER_H_=1 -DUSE_SBC=0 -c src/main.cpp -o build/src/main.o

I need advice on how the current tangled system works.
I basically need a map of the different .h and .cpp files, and their dependency tree.
Can you suggest any tools that will make this job easier for me, so I can overview my build?
