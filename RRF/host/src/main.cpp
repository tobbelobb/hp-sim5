#include <Version.h>
#include <Storage/MassStorage.h>

#include <CAN/CanCapture.h>
#include <CAN/CanMotion.h>
#include <CanMessageBuffer.h>
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
#include <Movement/DDA.h>
#include <Movement/Move.h>

#include <array>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace
{
	// Step 9.2.1: Deterministic simulated time in step-timer ticks (48 MHz)
	constexpr uint32_t StepClockFrequency = 48000000;  // 48 MHz
	uint64_t currentSimulatedTicks = 0;

	void PrintUsage()
	{
		std::cout << "Usage: host_rrf_bootstrap [--vsd <path>] [--run <file.gcode>] [--can-log <path|disable>]\n";
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

		// Build RawMove from G-code
		RawMove move;
		bool hasMovement = false;

		// Get current position as starting point
		float currentCoords[MaxAxesPlusExtruders] = {0};  // Initialize all to zero
		for (size_t i = 0; i < MaxAxes; ++i)
		{
			currentCoords[i] = reprap.GetGCodes().GetUserPosition(i);
		}

		// Copy current coords as target (will be updated by seen axes)
		for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
		{
			move.coords[i] = currentCoords[i];
		}

		// Process axis parameters
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
			move.coords[index] = updated;
			reprap.GetGCodes().SetUserPosition(index, updated);
			hasMovement = true;
		}

		// Handle extruder E parameter
		// TODO: For now, skip E parameter until we debug basic axis movement
		if (gb.Seen('E'))
		{
			gb.GetFValue();  // Consume but ignore for now
		}

		// Handle feedrate F parameter (in mm/min, convert to mm/sec)
		if (gb.Seen('F'))
		{
			const float feedRateMMperMin = gb.GetFValue();
			move.feedRate = feedRateMMperMin / 60.0f;  // Convert to mm/sec
		}
		else
		{
			// Use default feedrate if not specified
			move.feedRate = 100.0f;  // 100 mm/s default
		}

		// If there's actual movement, create and execute a DDA
		if (hasMovement)
		{
			std::cout << "[DEBUG] Creating DDA...\n" << std::flush;
			DDA dda;
			std::cout << "[DEBUG] Calling Init...\n" << std::flush;
			if (dda.Init(move, currentCoords))
			{
				std::cout << "[DEBUG] Init succeeded, calling Prepare...\n" << std::flush;
				if (dda.Prepare())
				{
					std::cout << "[DEBUG] Prepare succeeded, finishing movement...\n" << std::flush;
					// Set move start time to current simulated ticks
					dda.SetMoveStartTime(static_cast<uint32_t>(currentSimulatedTicks & 0xFFFFFFFF));

					std::cout << "[DEBUG] About to call FinishMovement...\n" << std::flush;
					// Call FinishMovement to actually emit CAN packets
					const uint32_t clocksUsed = CanMotion::FinishMovement(dda, dda.GetMoveStartTime(), false);
					std::cout << "[DEBUG] FinishMovement returned, clocks=" << clocksUsed << "\n" << std::flush;

					// Advance simulated time deterministically
					currentSimulatedTicks += clocksUsed;

					std::cout << "Move executed: distance=" << dda.GetPrepParams().totalDistance
							  << "mm, speed=" << dda.GetPrepParams().topSpeed
							  << "mm/s, ticks=" << clocksUsed << "\n";
				}
			}
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
	std::string canLogPath;
	bool showHelp = false;
	bool canLogProvided = false;
	bool canLogDisabled = false;

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
		else if (arg == "--can-log")
		{
			if (i + 1 >= argc)
			{
				std::cerr << "--can-log requires a path or the value 'disable'\n";
				return 1;
			}
			canLogPath = argv[++i];
			canLogProvided = true;
			const std::string lower = [] (std::string value) {
				for (char& ch : value)
				{
					ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
				}
				return value;
			}(canLogPath);
			if (lower == "disable" || lower == "none")
			{
				canLogDisabled = true;
			}
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

	std::filesystem::path absRoot;
	try
	{
		absRoot = std::filesystem::absolute(vsdPath);
		MassStorage::SetHostRoot(absRoot.string());
		MassStorage::Init();
	}
	catch (const std::exception& ex)
	{
		std::cerr << "Failed to initialise virtual SD: " << ex.what() << "\n";
		return 1;
	}

	std::filesystem::path resolvedCanLog;
	if (!canLogDisabled)
	{
		if (canLogProvided)
		{
			std::filesystem::path candidate(canLogPath);
			if (candidate.is_absolute())
			{
				resolvedCanLog = candidate;
			}
			else
			{
				resolvedCanLog = absRoot / candidate;
			}
		}
		else
		{
			resolvedCanLog = absRoot / "logs" / "can_capture.jsonl";
		}
	}

	if (!canLogDisabled)
	{
		if (!HostCanCapture::Configure(resolvedCanLog))
		{
			std::cerr << "Failed to initialise CAN capture sink at " << resolvedCanLog << "\n";
			return 1;
		}
	}
	else
	{
		if (!HostCanCapture::Configure({}))
		{
			std::cerr << "Failed to disable CAN capture sink\n";
			return 1;
		}
	}

	const bool captureActive = !canLogDisabled && !resolvedCanLog.empty();

	struct CaptureGuard
	{
		~CaptureGuard()
		{
			HostCanCapture::Shutdown();
		}
	} captureGuard;

	// Initialize CAN subsystem
	CanMessageBuffer::Init(40);  // Allocate 40 CAN message buffers (enough for multiple boards)
	CanMotion::Init();

	std::cout << "RRF host bootstrap build\n";
	std::cout << "Version: " << VERSION << "\n";
	std::cout << "Build date: " << DateText << TimeSuffix << "\n";
	std::cout << "Virtual SD root: " << MassStorage::GetHostRoot() << "\n";
	if (captureActive)
	{
		std::cout << "CAN capture log: " << resolvedCanLog << "\n";
	}
	else
	{
		std::cout << "CAN capture disabled\n";
	}

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
