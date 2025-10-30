#pragma once

#ifdef RRF_HOST_BUILD

#include <Movement/DDA.h>

namespace host::planner
{
	// Queue a segmented RawMove together with its start machine coordinates.
	void QueueSegment(const RawMove& move, const float startMachineCoords[MaxAxesPlusExtruders]) noexcept;

	// Flush all queued segments, performing lookahead scheduling and emitting CAN movements.
	// Returns true on success, false if any segment failed to initialise or prepare.
	bool FlushQueuedSegments() noexcept;

	// Reset queued segments (clears without emitting). Primarily for tests.
	void Reset() noexcept;
}

#endif // RRF_HOST_BUILD
