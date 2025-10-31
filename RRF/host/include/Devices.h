#pragma once
#define SUPPORT_USB		0

//#include <AsyncSerial.h>
//#include <USARTClass.h>
//#include <SerialCDC.h>
//
//extern AsyncSerial serialUart1;
//extern USARTClass serialUart2;
//#if defined(DUET3_MB6HC)
//extern AsyncSerial serialWiFi;
//#endif
//extern SerialCDC serialUSB;

void DeviceInit() noexcept {}
void StopAnalogTask() noexcept {}
void StopUsbTask() noexcept {}
