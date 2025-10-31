#include <CAN/CanInterface.h>

#include "CanCapture.h"
#include <CanMessageBuffer.h>

#if SUPPORT_CAN_EXPANSION

namespace CanInterface
{
void Init() noexcept
{
}

void Shutdown() noexcept
{
}

CanAddress GetCanAddress() noexcept
{
	return 0;
}

void SendMotion(CanMessageBuffer *buf) noexcept
{
	if (buf == nullptr)
	{
		return;
	}

	HostCanCapture::LogMotion(*buf);
	CanMessageBuffer::Free(buf);
}

void WakeAsyncSender() noexcept
{
}

void WakeAsyncSenderFromIsr() noexcept
{
}

} // namespace CanInterface

#endif
