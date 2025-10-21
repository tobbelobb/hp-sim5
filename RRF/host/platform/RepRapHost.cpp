#include <Platform/RepRap.h>

RepRap reprap;

RepRap::RepRap() noexcept
	: platform(),
	  gCodes(),
	  heat(),
	  printMonitor(platform, gCodes),
	  fansManager(),
	  globalVariables()
{
	platform.Init();
	heat.Init();
	fansManager.Init();
	printMonitor.Init();
}

ExpressionValue RepRap::GetObjectValueUsingTableNumber(ObjectExplorationContext& context,
													   const ObjectModelClassDescriptor*,
													   const char* idString,
													   uint8_t) THROWS(GCodeException)
{
	ExpressionValue result;

	if (context.WantExists())
	{
		result.SetBool(false);
		return result;
	}

	if (context.WantArrayLength())
	{
		result.SetUnsigned(0);
		return result;
	}

	result.SetNull(nullptr);

	if (idString != nullptr && *idString != '\0')
	{
		platform.MessageF(WarningMessage, "object model path '%s' not supported in host build\n", idString);
	}
	else
	{
		platform.Message(GenericMessage, "object model path lookup not supported in host build\n");
	}

	return result;
}
