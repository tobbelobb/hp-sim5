#include <Platform/MessageBox.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

// ----------------------------------------------------------------------------
// Minimal message box support for host builds

constexpr ObjectModelTableEntry MessageBox::objectModelTable[] = { };
constexpr uint8_t MessageBox::objectModelTableDescriptor[] = { 0 };

DEFINE_GET_OBJECT_MODEL_TABLE(MessageBox)

uint32_t MessageBox::nextSeq = 0;
MessageBox *_ecv_null MessageBox::mboxList = nullptr;
ReadWriteLock MessageBox::mboxLock;
uint32_t MessageBox::startTime = 0;
unsigned int MessageBox::numMessages = 0;
unsigned int MessageBox::numAutoCancelledMessages = 0;

/*static*/ uint32_t MessageBox::Create(const char *_ecv_array msg,
                                       const char *_ecv_array title,
                                       int mode,
                                       float timeout,
                                       AxesBitmap controls,
                                       MessageBoxLimits *_ecv_null limits) noexcept
{
	(void)msg;
	(void)title;
	(void)mode;
	(void)timeout;
	(void)controls;
	(void)limits;

	WriteLocker locker(mboxLock);
	const uint32_t seq = ++nextSeq;
	startTime = 0;
	reprap.StateUpdated();
	return seq;
}

float MessageBox::GetTimeLeft() const noexcept
{
	return 0.0f;
}

/*static*/ bool MessageBox::Acknowledge(uint32_t,
                                        bool& wasBlocking,
                                        bool& shouldAbort) noexcept
{
	wasBlocking = false;
	shouldAbort = false;
	return false;
}

/*static*/ bool MessageBox::CheckTimeout() noexcept
{
	return false;
}

/*static*/ ReadLockedPointer<const MessageBox> MessageBox::GetLockedCurrent() noexcept
{
	return ReadLockedPointer<const MessageBox>(nullptr, nullptr);
}

void MessageBox::TimeOut() noexcept
{
	// Host build never queues blocking message boxes.
}

void MessageBoxLimits::GetIntegerLimits(GCodeBuffer&,
                                        bool defaultIsString) THROWS(GCodeException)
{
	minVal.SetInt(0);
	maxVal.SetInt(0);
	if (defaultIsString)
	{
		defaultVal.Release();
	}
	else
	{
		defaultVal.SetInt(0);
	}
	choices.Release();
}

void MessageBoxLimits::GetFloatLimits(GCodeBuffer&) THROWS(GCodeException)
{
	minVal.SetFloat(0.0f, 1);
	maxVal.SetFloat(0.0f, 1);
	defaultVal.SetFloat(0.0f, 1);
	choices.Release();
}
