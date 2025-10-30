Hey, I'm building a host (x86_64) version of ReprapFirmware. My code is in ./host/, the original code is in ./ReprapFirmware/.
I'm currently working on compiling the object file from ReprapFirmware/src/PrintMonitor/PrintMonitor.o.
The way I work is that I have a CMakeLists.txt in here (absolute path ~/repos/hp-sim5/RRF/CMakeLists.txt).

You can read directly in the CMakeLists.txt which original files, versus which shadowing/fake files I intend currently to use in the final build.

I use my CMakeLists.txt like this:
```
cmake -B build
cd build
make prepare_sources
make generated_sources/src/PrintMonitor/PrintMonitor.o # Or whatever target we're working on
```

Right now I'm working on some errors related to `error: call of overloaded ‘ExpressionValue(long int)’ is ambiguous`.

As you can see in CMakeLists.txt I use the real header ObjecModel/ObjectModel.h.
I also compile the original ObjectModel/ObjectModel.cpp.

I guess some differences in int type sizes on host (x86_64) compared to the original microcontroller target makes some overloads ambiguous.
It has an implementation file here: host/platform/PlatformHost.cpp

Please make the generated_sources/src/PrintMonitor/PrintMonitor.o target build cleanly.
Remember to do `make prepare_sources` whenever the lists of source files have changed in CMakeLists.txt.
Don't edit files in the generated_sources directory directly, since they are meant to be transient, temporary files only used for compiling.
Instead, edit the files in their normal location and re-run `make prepare_sources`.

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
Keep your decisions in line with what's already there.
