#pragma once

#include "AsyncSerial.h"

class USARTClass : public AsyncSerial
{
public:
    using AsyncSerial::AsyncSerial;
};
