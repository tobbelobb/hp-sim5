#include <Hardware/SAM4E/Devices.h>

AsyncSerial serialUart;
AsyncSerial serialWiFi;
SerialCDC serialUSB;
TwoWire Wire;

void DeviceInit() noexcept {}
void StopAnalogTask() noexcept {}
void StopUsbTask() noexcept {}
