// Host-side DDA implementation for Step 9.1

#ifdef RRF_HOST_BUILD

#include <Movement/DDA.h>

DDA::DDA() noexcept
	: checkEndstops(false)
	, isPrintingMove(false)
	, moveStartTime(0)
	, clocksNeeded(0)
{
}

#endif // RRF_HOST_BUILD
