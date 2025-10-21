#include <ObjectModel/GlobalVariables.h>

#include <Platform/OutputMemory.h>

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
