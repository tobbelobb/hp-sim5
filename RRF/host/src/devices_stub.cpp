#include <AsyncSerial.h>
#include <Core.h>
#include <SerialCDC.h>
#include <USARTClass.h>

AsyncSerial serialUart1;
USARTClass serialUart2;
#if defined(DUET3_MB6HC)
AsyncSerial serialWiFi;
#endif
SerialCDC serialUSB;

void DeviceInit() noexcept
{
}
void StopAnalogTask() noexcept
{
}
void StopUsbTask() noexcept
{
}
