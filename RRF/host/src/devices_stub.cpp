#include <Core.h>

#ifdef SUPPORT_USB
#undef SUPPORT_USB
#endif

#include <Hardware/SAME70/Devices.h>

AsyncSerial serialUart1;
USARTClass serialUart2;
#if defined(DUET3_MB6HC)
AsyncSerial serialWiFi;
#endif
SerialCDC serialUSB;

void DeviceInit() noexcept {}
void StopAnalogTask() noexcept {}
void StopUsbTask() noexcept {}
