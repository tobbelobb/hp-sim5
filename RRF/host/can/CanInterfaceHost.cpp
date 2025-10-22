#include <CAN/CanInterface.h>

#include <CAN/CanCapture.h>
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

