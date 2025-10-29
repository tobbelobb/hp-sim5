#pragma once
#define SUPPORT_USB		0

//#include <AsyncSerial.h>
//#include <USARTClass.h>
//
//extern AsyncSerial serialUart1;
//extern USARTClass serialUart2;
//
//#include "SerialCDC.h"
//
//extern SerialCDC serialUSB;

void DeviceInit() noexcept {};
void StopAnalogTask() noexcept {};
void StopUsbTask() noexcept {};
