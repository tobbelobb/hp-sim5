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
#include <Movement/Kinematics/Kinematics.h>

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
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };
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
		// Note: Extruder uses separate relative/absolute mode (typically M83/M82)
		// For now, assume relative extruder mode (M83) as configured in typical config.g
		if (gb.Seen('E'))
		{
			const float eValue = gb.GetFValue();
			const size_t extruderIndex = MaxAxes;  // First extruder drive

			// For relative extrusion, add to current position
			// For absolute, would just set to eValue
			// For now, treat as relative (M83 mode is common)
			const float currentE = currentCoords[extruderIndex];
			const float targetE = currentE + eValue;  // Relative extrusion

			move.coords[extruderIndex] = targetE;
			move.hasE = true;
			hasMovement = true;  // E-only moves are valid
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
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };

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

	// M92: Set steps per mm
	// Example: M92 E415 sets extruder steps/mm to 415
	bool ProcessM92(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };
		Move& move = reprap.GetMove();

		// Handle axis parameters
		for (char letter : axisLetters)
		{
			if (gb.Seen(letter))
			{
				size_t axis = 0;
				if (reprap.GetGCodes().TryGetAxisIndex(letter, axis))
				{
					const float value = gb.GetFValue();
					move.SetDriveStepsPerMm(axis, value, 0);
					std::cout << "Set " << letter << " steps/mm to " << value << "\n";
				}
			}
		}

		// Handle extruder E parameter
		if (gb.Seen('E'))
		{
			const float value = gb.GetFValue();
			const size_t extruderDrive = MaxAxes;  // First extruder
			move.SetDriveStepsPerMm(extruderDrive, value, 0);
			std::cout << "Set E steps/mm to " << value << "\n";
		}

		return true;
	}

	// M201: Set max accelerations (mm/s²)
	// Example: M201 X10000 Y10000 Z10000 U10000 E1000
	bool ProcessM201(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };
		Move& move = reprap.GetMove();

		for (char letter : axisLetters)
		{
			if (gb.Seen(letter))
			{
				size_t axis = 0;
				if (reprap.GetGCodes().TryGetAxisIndex(letter, axis))
				{
					const float value = gb.GetFValue();
					move.SetAcceleration(axis, value);
					std::cout << "Set " << letter << " acceleration to " << value << " mm/s²\n";
				}
			}
		}

		if (gb.Seen('E'))
		{
			const float value = gb.GetFValue();
			const size_t extruderDrive = MaxAxes;
			move.SetAcceleration(extruderDrive, value);
			std::cout << "Set E acceleration to " << value << " mm/s²\n";
		}

		return true;
	}

	// M203: Set max speeds (mm/min - will convert to mm/s)
	// Example: M203 X36000 Y36000 Z36000 E3600
	bool ProcessM203(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };
		Move& move = reprap.GetMove();

		for (char letter : axisLetters)
		{
			if (gb.Seen(letter))
			{
				size_t axis = 0;
				if (reprap.GetGCodes().TryGetAxisIndex(letter, axis))
				{
					const float mmPerMin = gb.GetFValue();
					const float mmPerSec = mmPerMin / 60.0f;
					move.SetMaxFeedrate(axis, mmPerSec);
					std::cout << "Set " << letter << " max speed to " << mmPerSec << " mm/s\n";
				}
			}
		}

		if (gb.Seen('E'))
		{
			const float mmPerMin = gb.GetFValue();
			const float mmPerSec = mmPerMin / 60.0f;
			const size_t extruderDrive = MaxAxes;
			move.SetMaxFeedrate(extruderDrive, mmPerSec);
			std::cout << "Set E max speed to " << mmPerSec << " mm/s\n";
		}

		return true;
	}

	// M566: Set jerk (maximum instant speed change, mm/min)
	// Example: M566 X240 Y240 Z1200 E1200
	bool ProcessM566(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };
		Move& move = reprap.GetMove();

		for (char letter : axisLetters)
		{
			if (gb.Seen(letter))
			{
				size_t axis = 0;
				if (reprap.GetGCodes().TryGetAxisIndex(letter, axis))
				{
					const float value = gb.GetFValue();
					move.SetJerk(axis, value);
					std::cout << "Set " << letter << " jerk to " << value << " mm/min\n";
				}
			}
		}

		if (gb.Seen('E'))
		{
			const float value = gb.GetFValue();
			const size_t extruderDrive = MaxAxes;
			move.SetJerk(extruderDrive, value);
			std::cout << "Set E jerk to " << value << " mm/min\n";
		}

		return true;
	}

	// M584: Set axis to driver mapping
	// Example: M584 X40.0 Y41.0 Z42.0 U43.0 P4
	// Format: axis letter followed by board.driver (40.0 = board 40, driver 0)
	bool ProcessM584(GCodeBuffer& gb)
	{
		static constexpr std::array<char, 10> axisLetters{ 'X','Y','Z','U','V','W','A','B','C','D' };
		Move& move = reprap.GetMove();

		// Handle P parameter (number of visible axes)
		if (gb.Seen('P'))
		{
			const int visibleAxes = gb.GetIValue();
			reprap.GetGCodes().SetAxisCount(static_cast<size_t>(visibleAxes));
			std::cout << "Set visible axes to " << visibleAxes << "\n";
		}

		for (char letter : axisLetters)
		{
			if (gb.Seen(letter))
			{
				size_t axis = 0;
				if (reprap.GetGCodes().TryGetAxisIndex(letter, axis))
				{
					const float driverSpec = gb.GetFValue();
					const uint8_t board = static_cast<uint8_t>(driverSpec);
					const uint8_t localDriver = static_cast<uint8_t>((driverSpec - board) * 10.0f + 0.5f);

					DriverId driver;
					if (board == 0)
					{
						driver.SetLocal(localDriver);
					}
					else
					{
						driver = DriverId(board, localDriver);
					}

					move.SetAxisDriverId(axis, driver);
					std::cout << "Mapped " << letter << " axis to driver " << static_cast<int>(board)
							  << "." << static_cast<int>(localDriver) << "\n";
				}
			}
		}

		// Handle E parameter (extruder driver mapping)
		// Example: M584 E0:1:2:3:4:5
		// For now, just parse the first extruder
		if (gb.Seen('E'))
		{
			const int extruderDriver = gb.GetIValue();
			std::cout << "Mapped extruder to driver " << extruderDriver << "\n";
		}

		return true;
	}

	// M569: Set driver direction
	// Example: M569 P40.0 S1 (driver 40.0 goes forward)
	bool ProcessM569(GCodeBuffer& gb)
	{
		if (!gb.Seen('P'))
		{
			return true;  // No driver specified
		}

		Move& move = reprap.GetMove();
		const float driverSpec = gb.GetFValue();
		const uint8_t board = static_cast<uint8_t>(driverSpec);
		const uint8_t localDriver = static_cast<uint8_t>((driverSpec - board) * 10.0f + 0.5f);

		DriverId driver;
		if (board == 0)
		{
			driver.SetLocal(localDriver);
		}
		else
		{
			driver = DriverId(board, localDriver);
		}

		if (gb.Seen('S'))
		{
			const int direction = gb.GetIValue();
			const bool forward = (direction == 1);
			move.SetDriverDirection(driver, forward);
			std::cout << "Set driver " << static_cast<int>(board) << "." << static_cast<int>(localDriver)
					  << " direction to " << (forward ? "forward" : "backward") << "\n";
		}

		return true;
	}

	// M669: Set kinematics type
	// Example: M669 K6 (Hangprinter), M669 K1 (Cartesian)
	// Step 9.3.2: Now actually creates the real kinematics instance
	bool ProcessM669(GCodeBuffer& gb)
	{
		if (gb.Seen('K'))
		{
			const int kinematicsType = gb.GetIValue();
			std::cout << "Set kinematics type to " << kinematicsType;

			Move& move = reprap.GetMove();
			KinematicsType kType = KinematicsType::cartesian;  // Default

			switch (kinematicsType)
			{
			case 1:
				std::cout << " (Cartesian)";
				kType = KinematicsType::cartesian;
				break;
			case 6:
				std::cout << " (Hangprinter)";
				kType = KinematicsType::hangprinter;
				break;
			default:
				std::cout << " (unknown, defaulting to Cartesian)";
				kType = KinematicsType::cartesian;
				break;
			}
			std::cout << "\n";

			// Actually change the kinematics
			move.SetKinematics(kType);
		}

		// Other M669 parameters (for Hangprinter: anchor positions, etc.)
		// For now, just consume and log them
		static constexpr std::array<char, 4> hangprinterParams{ 'A', 'B', 'C', 'D' };
		for (char letter : hangprinterParams)
		{
			if (gb.Seen(letter))
			{
				gb.GetFValue();  // Consume the value
				// std::cout << "  " << letter << " parameter seen\n";
			}
		}

		return true;
	}

	// M666: Set Hangprinter mechanical parameters
	// Many sub-parameters: Q, R, U, O, L, H, W, S, I, X, T, Y, C, J
	// For now, just consume them - actual Hangprinter kinematics comes later
	bool ProcessM666(GCodeBuffer& gb)
	{
		// Just consume all the parameters for now
		static constexpr std::array<char, 14> params{ 'Q', 'R', 'U', 'O', 'L', 'H', 'W', 'S', 'I', 'X', 'T', 'Y', 'C', 'J' };

		for (char letter : params)
		{
			if (gb.Seen(letter))
			{
				gb.GetFValue();  // Consume the value
			}
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

			if (!gb.HasCommandNumber())
			{
				return true;
			}

			const int commandNumber = gb.GetCommandNumber();

			// Handle G-codes
			if (commandLetter == 'G')
			{
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

			// Handle M-codes
			if (commandLetter == 'M')
			{
				switch (commandNumber)
				{
				case 92:
					return ProcessM92(gb);
				case 201:
					return ProcessM201(gb);
				case 203:
					return ProcessM203(gb);
				case 566:
					return ProcessM566(gb);
				case 584:
					return ProcessM584(gb);
				case 569:
					return ProcessM569(gb);
				case 669:
					return ProcessM669(gb);
				case 666:
					return ProcessM666(gb);
				default:
					// Silently ignore unknown M-codes (many config commands we don't need yet)
					return true;
				}
			}

			return true;
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

	// Step 9.2.2: Execute config.g before running any user G-code
	std::cout << "\n=== Executing config.g ===\n";
	const std::string configPath = "0:/sys/config.g";
	if (MassStorage::FileExists(configPath.c_str()))
	{
		if (!ExecuteFile(configPath))
		{
			std::cerr << "Warning: config.g execution failed or was incomplete\n";
		}
		std::cout << "=== Config.g complete ===\n\n";
	}
	else
	{
		std::cout << "Note: No config.g found at " << configPath << ", using defaults\n\n";
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
