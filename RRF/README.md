## ReprapFirmware Host Build `rrf_simulator`

This is a host (x86_64) version of ReprapFirmware with:
 - No FreeRTOS
 - No reason to be synchronous, we can fake clock ticks and such. We will only run in "batch mode" on x86_64, so no reason to wait for anything or stay synchronous, as long as everything happens in the right order.
 - No connected boards, but we want to capture packets that would have been sent to external CAN boards, so we need to fake something there
 - No networking
 - No connections to other programs such as DuetWebInterface or DSW or anything like that.
 - No fans or heaters or anything like that. No physical pins or connections to anything at all actually.

The binary, `rrf_simulator`, is supposed to mirror the behavior of Klippers "batch mode", also known as "debug mode".
In batch mode, Klipper simply reads a gcode and a config file, and dumps the stepper move commands, as well as a handful of other commands.
This is very useful for hp-sim5 because we can run our simulation based on the commands read from the batch dump, without needing any special hardware.
Batch mode basically stubs the USB-interface of Klipper.

ReprapFirmware previously had no such mode, and not even a x86_64 build at all.
The closest analog to Klipper's USB interface is ReprapFirmware's CAN interface.
Our approach is to stub this and capture the move messages, so we can drive the hp-sim5 simulation with them.

We had to:
 1. Decide which real source files to compile, and which to stub or shim.
 2. Get the chosen real files to compile.
 3. Write the stub and shim code.
 4. Write the actual batch mode logic.

On point 1, the files are listed in CMakeLists.txt.
To control which files get used in our build, we populate a physically isolated build directory called `build/generated_sources`.

## How To Build and Run

```
# cd RRF # Assumed start directory
# Possibly rm -rf build if a corrupted one already exists, then cmake -B build to make a new one
cmake --build build --target rrf_simulator -j
./build/rrf_simulator --vsd run/vsd --gcode gcodes/test_cartesian.gcode --can-log run/vsd/logs/first.csv -c sys/config_hangprinter.g
```

## Instructions for Developers and AI Assistants

The ReprapFirmware code is in ./ReprapFirmware/src, ./RRFLibraries/src and ./CANlib/src.

Our code is in ./host.

main.cpp is the entry point.
It is supposed to exercises all the real ReprapFirmware logic that generates movement commands and that are sent out via the CAN interface.
The CAN messages are supposed to get captured by our logic and written to disk.
These captured files will be used in a physics simulator to check how well the ReprapFirmware planner, kinematics logic, DDA, movement system etc works.

Don't change real source files in ReprapFirmware source tree.
Some changes in ReprapFirmware has been neccessary to build cleanly, but it's mainly been about fixing format warnings.

If you implement any custom host logic (any code in RRF/host) then look up how the logic is implemented in the original code (RRF/ReprapFirmware),
and get as close as you can to the original logic, while preserving the host build's goal,
which is to create a binary that can consume gcode, plan moves, emit CAN packages, capture the CAN packages and write them to disk.
The movements are the most important part.
Fans, object model, and such are nice-to-haves.

Don't make any changes directly in the `RRF/build/generated_sources` directory.
If you need to make a change there go via the RRF/CMakeLists.txt file:
`cd RRF; cmake --build build --target rrf_simulator -j`


### Instructions For Adding New Pieces of ReprapFirmware Logic and Compiling It

Start off by changing the CMakeLists.txt file so your header and source file is available to the compiler and linker.
Then do `make prepare_sources && make help` in the build directory, and find an object (.o) file corresponding to your newly added implementation file (.cpp) if any.
Get this object file to compile cleanly first, with something like `make generated_sources/some/file.o`.
If you know the logic of many object files were touched you can build them all in one go with:

```
./build-all-objs.sh -C build
```

Once you've identified which object files give warnings and errors, I recommend building them individually, for less terminal noise.

The simplest error you can get is when the compiler can't find some header file.
Find the header file and make a decision if we should include the original header, or if we should make our own stubbed or shimmed version.
When you've made your decision on that, include the header file name in the appropriate list in CMakeLists.txt and re-run `make prepare_sources`.

When you get another error while compiling, first check the source file if the executing code should have been hidden by an `#if defined(SOME_IRRELEVANT_FEATURE)` or something.
If the code should have been not compiled, consider changing the host's #defines, for example in host/include/Config/Features_Host.h.
Don't change real source files in ReprapFirmware source tree, unless it's a simple `%u` -> `%zu` change to silence a `-Wformat` error, or something really minor like that.

If the code was really meant to be included (it most often is), then try to find out where the functionality is implemented, using your tools to find files and grep patterns.
If two files implement the functionality, for example one in the ReprapFirmware tree and one in the host tree, then the CMakeLists.txt should be inspected, in order to find out if the correct file is included in the source tree (unless that was
already evident).

If the functionality was only implemented in tre ReprapFirmware tree, then consider if we should make it behave acceptably by either:
 - Shadowing the impementation with either
   1. A shadowing header (.h-file) + a shadowing implementation (.cpp file), or
   2. A shadowing implementation file only, or
   3. By shadowing or fixing functionality that's one layer down (eg make the function compile by stubbing or shimming all the problematic function calls within it).
 - Accepting the original ReprapFirmware logic and making it compile properly on host directly from ReprapFirmware source files
   1. One tool you can utilize to make this work is by effectively injecting things into the top of the real header by including it in the RRF_HEADERS_WE_NEED_WITH_orig_POSTFIX list
      and creating your own version of the original header, with the same path under the host directory, and ending that header with `#include <ORIGINAL/PATH/name.h.orig>`

Either way, you need to decide if a given functionality makes sense on a host (x86_64) version of a 3d printer firmware like ReprapFirmware.
Reviewing CMakeLists.txt should give you a really good idea about what pieces of functionality we've aimed to preserve, and what we've aimed to stub or shim thus far.
It's generally a good idea to keep your decisions in line with what's already there.
