#pragma once

#if !defined(configMAX_PRIORITIES)
#define configMAX_PRIORITIES					( 8 )	// each priority level used 20 bytes of RAM, so don't make this too large
#endif

#include <Platform/TaskPriorities.h.orig>
