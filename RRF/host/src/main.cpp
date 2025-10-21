#include <Version.h>
#include <Storage/MassStorage.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace
{
	void PrintUsage()
	{
		std::cout << "Usage: host_rrf_bootstrap [--vsd <path>] [--run <file.gcode>]\n";
	}
}

int main(int argc, char** argv)
{
	std::string vsdPath = "host/vsd";
	std::string runPath;
	bool showHelp = false;

	for (int i = 1; i < argc; ++i)
	{
		const std::string arg(argv[i]);
		if (arg == "--vsd")
		{
			if (i + 1 >= argc)
			{
				std::cerr << "--vsd requires a path argument\n";
				return 1;
			}
			vsdPath = argv[++i];
		}
		else if (arg == "--run")
		{
			if (i + 1 >= argc)
			{
				std::cerr << "--run requires a filename\n";
				return 1;
			}
			runPath = argv[++i];
		}
		else if (arg == "--help" || arg == "-h")
		{
			showHelp = true;
		}
		else
		{
			std::cerr << "Unknown argument: " << arg << "\n";
			return 1;
		}
	}

	if (showHelp)
	{
		PrintUsage();
		return 0;
	}

	try
	{
		std::filesystem::path absRoot = std::filesystem::absolute(vsdPath);
		MassStorage::SetHostRoot(absRoot.string());
		MassStorage::Init();
	}
	catch (const std::exception& ex)
	{
		std::cerr << "Failed to initialise virtual SD: " << ex.what() << "\n";
		return 1;
	}

	std::cout << "RRF host bootstrap build\n";
	std::cout << "Version: " << VERSION << "\n";
	std::cout << "Build date: " << DateText << TimeSuffix << "\n";
	std::cout << "Virtual SD root: " << MassStorage::GetHostRoot() << "\n";

	if (!runPath.empty())
	{
		std::cout << "G-code execution not yet implemented. Requested file: " << runPath << "\n";
	}

	return 0;
}
