#pragma once

#ifdef RRF_HOST_BUILD

#include <UniqueIdBase.h>
#include <Platform/OutputMemory.h>
#include <Networking/NetworkDefs.h>

class UniqueId : public UniqueIdBase
{
public:
	void AppendCharsToBuffer(OutputBuffer *buf) const noexcept
	{
		if (buf == nullptr)
		{
			return;
		}

		AppendCharsTo([buf](char c) noexcept { buf->cat(c); });
	}

	void GenerateMacAddress(MacAddress& addr) const noexcept
	{
		addr.SetDefault();
		addr.bytes[0] |= 0x02;    // mark as locally administered
	}
};

#endif
