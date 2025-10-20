#pragma once

#include <cstdint>

using DmaChannel = uint8_t;
using DmaPriority = uint8_t;
using Pin = uint32_t;
using PwmFrequency = uint16_t;
using CanAddress = uint8_t;
using NvicPriority = uint32_t;
using ExintNumber = uint8_t;
using EventNumber = uint8_t;

#ifndef __FP16_TYPE_DEFINED
#define __FP16_TYPE_DEFINED
using __fp16 = float;
#endif

#ifndef FLOAT16_T_DEFINED
# define FLOAT16_T_DEFINED
using float16_t = __fp16;
#endif

constexpr Pin NoPin = 0xFF;
constexpr Pin Nx = 0xFF;
