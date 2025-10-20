#pragma once

#include <cstdint>

using DmaChannel = uint8_t;
using DmaPriority = uint8_t;
using Pin = uint8_t;
using PwmFrequency = uint16_t;
using CanAddress = uint8_t;
using NvicPriority = uint32_t;
using ExintNumber = uint8_t;
using EventNumber = uint8_t;

#ifndef FLOAT16_T_DEFINED
# define FLOAT16_T_DEFINED
using float16_t = uint16_t;
#endif

constexpr Pin NoPin = 0xFF;
constexpr Pin Nx = 0xFF;

