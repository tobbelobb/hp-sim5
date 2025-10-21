#include <ObjectModel/GlobalVariables.h>

#include <Platform/OutputMemory.h>

const ObjectModelTableEntry GlobalVariables::objectModelTable[1] = {};
const uint8_t GlobalVariables::objectModelTableDescriptor[1] = { 0 };
const ObjectModelClassDescriptor GlobalVariables::objectModelClassDescriptor = { objectModelTable, objectModelTableDescriptor, nullptr };

const ObjectModelClassDescriptor* GlobalVariables::GetObjectModelClassDescriptor() const noexcept
{
	return &objectModelClassDescriptor;
}

ReadLockedPointer<const VariableSet> GlobalVariables::GetForReading() noexcept
{
	return ReadLockedPointer<const VariableSet>(nullptr, &vars);
}

WriteLockedPointer<VariableSet> GlobalVariables::GetForWriting() noexcept
{
	return WriteLockedPointer<VariableSet>(nullptr, &vars);
}

void GlobalVariables::ReportAllAsJson(OutputBuffer* buf) const THROWS(GCodeException)
{
	if (buf != nullptr)
	{
		buf->cat("{}");
	}
}

void GlobalVariables::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext&, const ObjectModelClassDescriptor*, uint8_t, const char*) const THROWS(GCodeException)
{
	if (buf != nullptr)
	{
		buf->cat("{}");
	}
}
