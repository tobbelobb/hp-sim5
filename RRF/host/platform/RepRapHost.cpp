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

