#include <Tools/Spindle.h>

// Host build: provide minimal object model plumbing so the class is concrete.

constexpr ObjectModelTableEntry Spindle::objectModelTable[] = { };
constexpr uint8_t Spindle::objectModelTableDescriptor[] = { 0 };

DEFINE_GET_OBJECT_MODEL_TABLE(Spindle)
