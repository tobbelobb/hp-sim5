#include <RepRapFirmware.h>
#include <Storage/MassStorage.h>
#include <can/CanCapture.h>
#include <CAN/CanMotion.h>
#include <CanMessageBuffer.h>
#include <Platform/RepRap.h>
#include <PrintMonitor/PrintMonitor.h>
#define private public
#include <GCodes/GCodes.h>
#undef private
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <General/String.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

namespace
{
	enum class CaptureSelection : uint8_t
	{
		notProvided,
		disabled,
		enabled
	};

	struct CommandLineOptions
	{
		std::filesystem::path vsdRoot{"run/vsd"};
		std::string runArgument;
		CaptureSelection capture{CaptureSelection::notProvided};
		std::filesystem::path capturePath;
		bool showHelp{false};
	};

	constexpr unsigned int kDefaultCanBuffers = 64;
	constexpr std::chrono::minutes kPrintTimeout{30};
	constexpr std::chrono::milliseconds kSpinSleep{1};
	constexpr unsigned int kIdleSettlingCycles = 25;

	void PrintUsage() noexcept
	{
		std::cout << "Usage: rrf_simulator [--vsd <path>] [--gcode <file.gcode>] [--can-log <path|disable>]\n";
	}

	std::string ToLower(std::string value) noexcept
	{
		std::transform(value.begin(), value.end(), value.begin(),
					   [] (unsigned char ch) noexcept { return static_cast<char>(std::tolower(ch)); });
		return value;
	}

	bool ParseCommandLine(int argc, char** argv, CommandLineOptions& options, std::string& error) noexcept
	{
		for (int i = 1; i < argc; ++i)
		{
			const std::string arg(argv[i]);
			if (arg == "--help" || arg == "-h")
			{
				options.showHelp = true;
				return true;
			}
			else if (arg == "--vsd")
			{
				if (i + 1 >= argc)
				{
					error = "--vsd requires a path argument";
					return false;
				}
				options.vsdRoot = argv[++i];
			}
			else if (arg == "--gcode")
			{
				if (i + 1 >= argc)
				{
					error = "--gcode requires a filename";
					return false;
				}
				options.runArgument = argv[++i];
			}
			else if (arg == "--can-log")
			{
				if (i + 1 >= argc)
				{
					error = "--can-log requires a path or the value 'disable'";
					return false;
				}
				const std::string value = ToLower(argv[++i]);
				if (value == "disable" || value == "none" || value == "off")
				{
					options.capture = CaptureSelection::disabled;
					options.capturePath.clear();
				}
				else
				{
					options.capture = CaptureSelection::enabled;
					options.capturePath = argv[i];
				}
			}
			else
			{
				error = "Unknown option '" + arg + "'";
				return false;
			}
		}
		return true;
	}

	std::optional<std::string> ResolveRunFile(const std::filesystem::path& vsdRoot,
											  const std::string& runArg,
											  std::string& error) noexcept
	{
		if (runArg.empty())
		{
			return std::nullopt;
		}

		const std::filesystem::path gcodeRoot = vsdRoot / "gcodes";
		if (!std::filesystem::exists(gcodeRoot))
		{
			error = "G-code directory '" + gcodeRoot.string() + "' does not exist";
			return std::nullopt;
		}

		auto resolveRelative = [&gcodeRoot, &error] (const std::filesystem::path& hostPath) -> std::optional<std::string>
		{
			if (!std::filesystem::exists(hostPath))
			{
				error = "G-code file '" + hostPath.string() + "' does not exist";
				return std::nullopt;
			}

			std::error_code ec;
			std::filesystem::path relative = std::filesystem::relative(hostPath, gcodeRoot, ec);
			if (ec)
			{
				error = "File '" + hostPath.string() + "' is outside '" + gcodeRoot.string() + "'";
				return std::nullopt;
			}

			for (const auto& component : relative)
			{
				if (component == "..")
				{
					error = "File '" + hostPath.string() + "' escapes the gcodes directory";
					return std::nullopt;
				}
			}

			if (relative.empty())
			{
				error = "Run target refers to a directory";
				return std::nullopt;
			}

			return relative.generic_string();
		};

		// RRF-style path?
		if (runArg.size() > 2 && runArg[1] == ':')
		{
			std::string remainder = runArg.substr(2);
			while (!remainder.empty() && (remainder.front() == '/' || remainder.front() == '\\'))
			{
				remainder.erase(remainder.begin());
			}

			std::filesystem::path rrfPath(remainder);
			if (rrfPath.empty())
			{
				error = "gcode path '" + runArg + "' is not valid";
				return std::nullopt;
			}

			auto iter = rrfPath.begin();
			if (iter == rrfPath.end() || iter->string() != "gcodes")
			{
				error = "gcode path '" + runArg + "' must target 0:/gcodes";
				return std::nullopt;
			}

			std::filesystem::path relative;
			for (++iter; iter != rrfPath.end(); ++iter)
			{
				relative /= *iter;
			}

			if (relative.empty())
			{
				error = "gcode path '" + runArg + "' is incomplete";
				return std::nullopt;
			}

			return resolveRelative(gcodeRoot / relative);
		}

		// Host-style path. Prefer path relative to gcodes, fall back to vsdRoot.
		std::filesystem::path candidate(runArg);
		if (candidate.is_absolute())
		{
			return resolveRelative(candidate);
		}

		const std::filesystem::path gcodesCandidate = gcodeRoot / candidate;
		if (std::filesystem::exists(gcodesCandidate))
		{
			return resolveRelative(gcodesCandidate);
		}

		const std::filesystem::path vsdCandidate = vsdRoot / candidate;
		return resolveRelative(vsdCandidate);
	}

	bool ConfigureCapture(const CommandLineOptions& options) noexcept
	{
		switch (options.capture)
		{
			case CaptureSelection::disabled:
				return HostCanCapture::Configure({});

			case CaptureSelection::enabled:
			{
				std::filesystem::path capturePath = options.capturePath;
				if (!capturePath.is_absolute())
				{
					capturePath = std::filesystem::absolute(capturePath);
				}

				const std::filesystem::path parent = capturePath.parent_path();
				if (!parent.empty())
				{
					std::error_code ec;
					std::filesystem::create_directories(parent, ec);
					if (ec)
					{
						std::cerr << "Failed to create CAN log directory '" << parent << "': " << ec.message() << '\n';
						return false;
					}
				}

				if (!HostCanCapture::Configure(capturePath))
				{
					std::cerr << "Failed to open CAN log file '" << capturePath << "'\n";
					return false;
				}
				std::cout << "CAN capture enabled: " << capturePath << '\n';
				return true;
			}

			case CaptureSelection::notProvided:
			default:
				return HostCanCapture::Configure({});
		}
	}

	bool WaitForPrintCompletion() noexcept
	{
		GCodeBuffer* const fileBuffer = reprap.GetGCodes().FileGCode();
		if (fileBuffer == nullptr)
		{
			std::cerr << "No file G-code buffer available\n";
			return false;
		}

		const auto start = std::chrono::steady_clock::now();
		unsigned int idleCycles = 0;

		for (;;)
		{
			reprap.Spin();

			const bool printing = reprap.GetPrintMonitor().IsPrinting();
			const bool fileBusy = fileBuffer->IsDoingFile() || !fileBuffer->IsCompletelyIdle();
			const bool moveActive = !reprap.GetMove().NoLiveMovement();

			if (!printing && !fileBusy && !moveActive)
			{
				if (++idleCycles > kIdleSettlingCycles)
				{
					return true;
				}
			}
			else
			{
				idleCycles = 0;
			}

			if (reprap.IsStopped())
			{
				std::cerr << "Firmware entered stopped state\n";
				return false;
			}

			if (std::chrono::steady_clock::now() - start > kPrintTimeout)
			{
				std::cerr << "Timed out waiting for print to finish\n";
				return false;
			}

			std::this_thread::sleep_for(kSpinSleep);
		}
	}

	void ReportFinalPosition() noexcept
	{
		float machine[MaxAxes] = { 0.0f };
		reprap.GetMove().GetCurrentMachinePosition(machine, 0);

		const char* letters = reprap.GetGCodes().GetAxisLetters();
		const size_t totalAxes = reprap.GetGCodes().GetTotalAxes();

		std::cout << "Final machine position:";
		for (size_t axis = 0; axis < totalAxes && letters[axis] != '\0'; ++axis)
		{
			std::cout << ' ' << letters[axis] << '=' << machine[axis];
		}
		std::cout << '\n';
	}

	bool StartPrint(const std::string& relativePath) noexcept
	{
		String<GCodeReplyLength> reply;
		if (!reprap.GetGCodes().QueueFileToPrint(relativePath.c_str(), reply.GetRef()))
		{
			std::cerr << reply.c_str();
			return false;
		}

		reprap.GetPrintMonitor().StartingPrint(relativePath.c_str());
		reprap.GetGCodes().StartPrinting(true);

		const auto start = std::chrono::steady_clock::now();
		const bool ok = WaitForPrintCompletion();
		const auto stop = std::chrono::steady_clock::now();

		const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		if (ok)
		{
			std::cout << "Completed G-code '" << relativePath << "' in " << elapsed.count() << " ms\n";
		}
		else
		{
			std::cerr << "Failed to complete G-code '" << relativePath << "'\n";
		}
		return ok;
	}
} // namespace

int main(int argc, char** argv)
{
	CommandLineOptions options;
	std::string parseError;
	if (!ParseCommandLine(argc, argv, options, parseError))
	{
		std::cerr << parseError << '\n';
		PrintUsage();
		return 1;
	}

	if (options.showHelp)
	{
		PrintUsage();
		return 0;
	}

	bool reprapInitialised = false;
	const auto cleanup = [&]() noexcept
	{
		if (reprapInitialised)
		{
			reprap.Exit();
			reprapInitialised = false;
		}
		HostCanCapture::Shutdown();
		MassStorage::CloseAllFiles();
	};

	try
	{
		std::filesystem::path vsdRoot = options.vsdRoot;
		if (!vsdRoot.is_absolute())
		{
			vsdRoot = std::filesystem::absolute(vsdRoot);
		}

		std::error_code dirError;
		std::filesystem::create_directories(vsdRoot, dirError);
		if (dirError && !std::filesystem::exists(vsdRoot))
		{
			std::cerr << "Failed to prepare VSD directory '" << vsdRoot << "': " << dirError.message() << '\n';
			return 1;
		}

		MassStorage::SetHostRoot(vsdRoot.string());
		MassStorage::Init();

		if (!ConfigureCapture(options))
		{
			cleanup();
			return 1;
		}

		CanMessageBuffer::Init(kDefaultCanBuffers);
		CanMotion::Init();

		reprap.Init();
		reprapInitialised = true;

		bool success = true;
		if (!options.runArgument.empty())
		{
			std::string resolveError;
			auto relative = ResolveRunFile(vsdRoot, options.runArgument, resolveError);
			if (!relative)
			{
				std::cerr << resolveError << '\n';
				success = false;
			}
			else
			{
				std::cout << "Starting G-code '" << *relative << "'\n";
				success = StartPrint(*relative);
			}
		}
		else
		{
			std::cout << "Firmware initialised. No --gcode file supplied, exiting.\n";
		}

		if (success)
		{
			ReportFinalPosition();
		}

		cleanup();
		return success ? 0 : 1;
	}
	catch (const std::exception& ex)
	{
		std::cerr << "Fatal error: " << ex.what() << '\n';
		cleanup();
		return 1;
	}
	catch (...)
	{
		std::cerr << "Fatal error: unknown exception\n";
		cleanup();
		return 1;
	}
}
