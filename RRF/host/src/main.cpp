#include <Version.h>
#include <Storage/MassStorage.h>

#include <Platform/RepRap.h>
#include <Platform/MessageType.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/GCodeInput.h>
#include <GCodes/GCodeException.h>
#include <GCodes/GCodeMachineState.h>
#include <GCodes/GCodes.h>
#include <Storage/FileData.h>
#include <Storage/FileStore.h>
#include <General/String.h>

#include <array>
#include <cctype>
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

	std::string NormaliseRunPath(const std::string& rawPath)
	{
		if (rawPath.empty())
		{
			return rawPath;
		}

		if (rawPath.size() > 1 && rawPath[1] == ':')
		{
			return rawPath;
		}

		std::string trimmed = rawPath;
		while (!trimmed.empty() && (trimmed.front() == '/' || trimmed.front() == '\\'))
		{
			trimmed.erase(trimmed.begin());
		}
		return "0:/" + trimmed;
	}

	bool ProcessLinearMove(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','A','B','C','D','U','V','W' };
		const bool axesRelative = reprap.GetGCodes().GetAxesRelative(0);

		for (char letter : axisLetters)
		{
			if (!gb.Seen(letter))
			{
				continue;
			}

			const float value = gb.GetFValue();
			size_t index = 0;
			if (!reprap.GetGCodes().TryGetAxisIndex(letter, index))
			{
				continue;
			}

			const float current = reprap.GetGCodes().GetUserPosition(index);
			const float updated = axesRelative ? current + value : value;
			reprap.GetGCodes().SetUserPosition(index, updated);
		}

		return true;
	}

	bool ProcessSetPosition(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','A','B','C','D','U','V','W' };

		for (char letter : axisLetters)
		{
			if (!gb.Seen(letter))
			{
				continue;
			}

			const float value = gb.GetFValue();
			size_t index = 0;
			if (!reprap.GetGCodes().TryGetAxisIndex(letter, index))
			{
				continue;
			}
			reprap.GetGCodes().SetUserPosition(index, value);
		}
		return true;
	}

	bool ProcessGCode(GCodeBuffer& gb)
	{
		try
		{
			const char commandLetter = gb.GetCommandLetter();
			if (commandLetter == 0)
			{
				return true;
			}

			if (commandLetter != 'G' || !gb.HasCommandNumber())
			{
				return true;
			}

			const int commandNumber = gb.GetCommandNumber();
			switch (commandNumber)
			{
			case 0:
			case 1:
				return ProcessLinearMove(gb);

			case 90:
				reprap.GetGCodes().SetAxesRelative(0, false);
				gb.LatestMachineState().axesRelative = false;
				reprap.GetPlatform().Message(GenericMessage, "Using absolute positioning");
				return true;

			case 91:
				reprap.GetGCodes().SetAxesRelative(0, true);
				gb.LatestMachineState().axesRelative = true;
				reprap.GetPlatform().Message(GenericMessage, "Using relative positioning");
				return true;

			case 92:
				return ProcessSetPosition(gb);

			default:
				return true;
			}
		}
		catch (const GCodeException& exc)
		{
			String<StringLength100> message;
			exc.GetMessage(message.GetRef(), &gb);
			reprap.GetPlatform().MessageF(ErrorMessage, "G-code error: %s", message.c_str());
			return false;
		}
	}

	bool FlushPending(GCodeBuffer& gb)
	{
		if (!gb.FileEnded())
		{
			return true;
		}

		gb.DecodeCommand();
		const bool ok = ProcessGCode(gb);
		gb.SetFinished(true);
		gb.Init();
		return ok;
	}

	bool ExecuteFile(const std::string& rrfPath)
	{
		if (!MassStorage::FileExists(rrfPath.c_str()))
		{
			std::cerr << "G-code file not found: " << rrfPath << "\n";
			return false;
		}

		FileStore* store = MassStorage::OpenFile(rrfPath.c_str(), OpenMode::read, 0);
		if (store == nullptr)
		{
			std::cerr << "Failed to open G-code file: " << rrfPath << "\n";
			return false;
		}

		FileData fileData;
		fileData.Set(store);

	RegularGCodeInput normalInput;

	GCodeBuffer buffer(GCodeChannel::File, &normalInput, nullptr, FileMessage, Compatibility::RepRapFirmware);
		buffer.StartNewFile();
		buffer.Init();
		buffer.LatestMachineState().axesRelative = false;

		reprap.GetGCodes().ClearInputs();
		reprap.GetGCodes().RegisterInput(buffer);
		reprap.GetGCodes().SetAxisCount(7);
		reprap.GetGCodes().SetAxesRelative(0, false);
		reprap.GetGCodes().ResetUserPositions();

		bool running = true;

		constexpr size_t chunkSize = 512;
		char chunk[chunkSize];

		while (running)
		{
			const int bytesRead = fileData.Read(chunk, chunkSize);
			if (bytesRead < 0)
			{
				std::cerr << "Error while reading G-code data\n";
				running = false;
				break;
			}

			if (bytesRead == 0)
			{
				break;
			}

			for (int i = 0; i < bytesRead && running; ++i)
			{
				if (buffer.Put(chunk[i]))
				{
					buffer.DecodeCommand();
					running = ProcessGCode(buffer);
					buffer.SetFinished(true);
					buffer.Init();
				}
			}
		}

		if (running)
		{
			running = FlushPending(buffer);
		}

		fileData.Close();

		if (running)
		{
			const char* letters = reprap.GetGCodes().GetAxisLetters();
			const size_t axisCount = reprap.GetGCodes().GetAxisCount();
			std::cout << "Final user positions:";
			for (size_t i = 0; i < axisCount; ++i)
			{
				const float pos = reprap.GetGCodes().GetUserPosition(i);
				std::cout << ' ' << letters[i] << '=' << pos;
			}
			std::cout << '\n';
		}

		return running;
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
		const std::string canonicalRun = NormaliseRunPath(runPath);
		std::cout << "Executing G-code file: " << canonicalRun << "\n";
		if (!ExecuteFile(canonicalRun))
		{
			return 1;
		}
	}

	return 0;
}
