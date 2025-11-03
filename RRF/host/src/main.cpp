#include <CanMessageBuffer.h>
#include <RepRapFirmware.h>
#include <Storage/MassStorage.h>
#include <can/CanCapture.h>
#define private public
#define protected public
#include <CAN/CanMotion.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
#include <Platform/RepRap.h>
#include <PrintMonitor/PrintMonitor.h>
#undef protected
#undef private
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/SimulationMode.h>
#include <General/String.h>
#include <HostTiming.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

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
    std::optional<std::filesystem::path> gcodeArgument;
    CaptureSelection capture{CaptureSelection::notProvided};
    std::optional<std::filesystem::path> captureArgument;
    std::optional<std::filesystem::path> configArgument;
    bool showHelp{false};
};

constexpr unsigned int kDefaultCanBuffers = 64;
constexpr std::chrono::minutes kPrintTimeout{30};
constexpr unsigned int kIdleSettlingCycles = 25;
constexpr bool kTraceCompletion = false;

void PrintUsage() noexcept
{
    std::cout << "Usage: rrf_simulator [--vsd|-s <path>] [--gcode|-g <file.gcode>] "
                 "[--config|-c <file>]"
                 " [--can-log|-l <path|disable>]\n";
}

std::string ToLower(std::string value) noexcept
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) noexcept
                   { return static_cast<char>(std::tolower(ch)); });
    return value;
}

std::filesystem::path GetCurrentPath() noexcept
{
    std::error_code ec;
    const auto cwd = std::filesystem::current_path(ec);
    return ec ? std::filesystem::path{} : cwd;
}

std::filesystem::path NormalisePath(const std::filesystem::path& value) noexcept
{
    if (value.empty())
    {
        return value;
    }

    std::filesystem::path result;
    if (value.is_absolute())
    {
        result = value;
    }
    else
    {
        std::error_code absoluteError;
        result = std::filesystem::absolute(value, absoluteError);
        if (absoluteError)
        {
            const auto cwd = GetCurrentPath();
            if (!cwd.empty())
            {
                result = cwd / value;
            }
            else
            {
                result = value;
            }
        }
    }

    return result.lexically_normal();
}

bool IsSubPathOf(const std::filesystem::path& candidate,
                 const std::filesystem::path& base) noexcept
{
    auto candidateIt = candidate.begin();
    for (const auto& component : base)
    {
        if (candidateIt == candidate.end() || *candidateIt != component)
        {
            return false;
        }
        ++candidateIt;
    }
    return true;
}

std::optional<std::filesystem::path> ResolveExistingPath(
    const std::filesystem::path& vsdRoot, const std::filesystem::path& userPath,
    const char* description, bool requireRegularFile, std::string& error) noexcept
{
    error.clear();

    if (userPath.empty())
    {
        error = std::string(description) + " path is empty";
        return std::nullopt;
    }

    const auto normalizedVsd = NormalisePath(vsdRoot);

    const auto validate = [&](const std::filesystem::path& candidate)
        -> std::optional<std::filesystem::path>
    {
        std::error_code existsError;
        if (!std::filesystem::exists(candidate, existsError))
        {
            return std::nullopt;
        }

        if (requireRegularFile)
        {
            std::error_code fileError;
            if (!std::filesystem::is_regular_file(candidate, fileError))
            {
                error = std::string(description) + " '" + candidate.string() +
                        "' is not a regular file";
                return std::nullopt;
            }
        }

        return NormalisePath(candidate);
    };

    if (userPath.is_absolute())
    {
        const auto normalizedUser = NormalisePath(userPath);
        auto result = validate(normalizedUser);
        if (!result && error.empty())
        {
            error = std::string(description) + " '" + normalizedUser.string() +
                    "' does not exist";
        }
        return result;
    }

    const auto vsdCandidate = NormalisePath(normalizedVsd / userPath);
    if (auto result = validate(vsdCandidate))
    {
        return result;
    }
    if (!error.empty())
    {
        return std::nullopt;
    }

    const auto cwd = GetCurrentPath();
    const auto cwdCandidate =
        cwd.empty() ? NormalisePath(userPath) : NormalisePath(cwd / userPath);
    if (auto result = validate(cwdCandidate))
    {
        return result;
    }
    if (!error.empty())
    {
        return std::nullopt;
    }

    error = std::string(description) + " '" + userPath.string() +
            "' was not found relative to '" + normalizedVsd.string() +
            "' or the current working directory";
    return std::nullopt;
}

std::filesystem::path ResolveOutputPath(const std::filesystem::path& vsdRoot,
                                        const std::filesystem::path& userPath) noexcept
{
    if (userPath.empty())
    {
        return {};
    }

    if (userPath.is_absolute())
    {
        return NormalisePath(userPath);
    }

    const auto normalizedVsd = NormalisePath(vsdRoot);
    const auto vsdCandidate = NormalisePath(normalizedVsd / userPath);

    const auto cwd = GetCurrentPath();
    const auto cwdCandidate =
        cwd.empty() ? NormalisePath(userPath) : NormalisePath(cwd / userPath);

    if (cwdCandidate.is_absolute() && IsSubPathOf(cwdCandidate, normalizedVsd))
    {
        return cwdCandidate;
    }

    return vsdCandidate;
}

bool PrepareConfigFile(const std::filesystem::path& vsdRoot,
                       const std::filesystem::path& configSource) noexcept
{
    const auto normalizedVsd = NormalisePath(vsdRoot);
    const auto normalizedSource = NormalisePath(configSource);

    std::filesystem::path sysDir = (normalizedVsd / "sys").lexically_normal();

    std::error_code dirError;
    std::filesystem::create_directories(sysDir, dirError);
    if (dirError)
    {
        std::cerr << "Failed to prepare sys directory '" << sysDir
                  << "': " << dirError.message() << '\n';
        return false;
    }

    const std::filesystem::path defaultConfig = (sysDir / "config.g").lexically_normal();

    std::string displayPath = normalizedSource.string();
    displayPath.erase(std::remove(displayPath.begin(), displayPath.end(), '"'),
                      displayPath.end());
    std::cout << "Using configuration file: " << displayPath << '\n';

    std::error_code eqError;
    bool sameAsDefault = false;
    if (std::filesystem::exists(defaultConfig))
    {
        sameAsDefault =
            std::filesystem::equivalent(defaultConfig, normalizedSource, eqError);
    }

    if (eqError)
    {
        // Treat errors comparing paths as not being the same path
        sameAsDefault = false;
    }

    if (sameAsDefault)
    {
        MassStorage::ClearSysConfigOverride();
        return true;
    }

    MassStorage::SetSysConfigOverride(normalizedSource.string());
    return true;
}

bool ParseCommandLine(int argc, char** argv, CommandLineOptions& options,
                      std::string& error) noexcept
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            options.showHelp = true;
            return true;
        }
        else if (arg == "--vsd" || arg == "-s")
        {
            if (i + 1 >= argc)
            {
                error = "--vsd requires a path argument";
                return false;
            }
            options.vsdRoot = argv[++i];
        }
        else if (arg == "--gcode" || arg == "-g")
        {
            if (i + 1 >= argc)
            {
                error = "--gcode requires a filename";
                return false;
            }
            options.gcodeArgument = std::filesystem::path(argv[++i]);
        }
        else if (arg == "--config" || arg == "-c")
        {
            if (i + 1 >= argc)
            {
                error = "--config requires a filename";
                return false;
            }
            options.configArgument = std::filesystem::path(argv[++i]);
        }
        else if (arg == "--can-log" || arg == "-l")
        {
            if (i + 1 >= argc)
            {
                error = "--can-log requires a path or the value 'disable'";
                return false;
            }
            std::string rawValue(argv[++i]);
            const std::string value = ToLower(rawValue);
            if (value == "disable" || value == "none" || value == "off")
            {
                options.capture = CaptureSelection::disabled;
                options.captureArgument.reset();
            }
            else
            {
                options.capture = CaptureSelection::enabled;
                options.captureArgument = std::filesystem::path(rawValue);
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

std::optional<std::string> ResolveRunFile(
    const std::filesystem::path& vsdRoot,
    const std::optional<std::filesystem::path>& runArg, std::string& error) noexcept
{
    if (!runArg)
    {
        return std::nullopt;
    }

    const auto gcodeRoot = NormalisePath(vsdRoot / "gcodes");

    std::error_code existsError;
    if (!std::filesystem::exists(gcodeRoot, existsError))
    {
        error = "G-code directory '" + gcodeRoot.string() + "' does not exist";
        return std::nullopt;
    }

    const auto toRelative =
        [&gcodeRoot,
         &error](const std::filesystem::path& hostPath) -> std::optional<std::string>
    {
        std::error_code existsErr;
        if (!std::filesystem::exists(hostPath, existsErr))
        {
            error = "G-code file '" + hostPath.string() + "' does not exist";
            return std::nullopt;
        }

        const auto normalizedHost = NormalisePath(hostPath);
        if (!IsSubPathOf(normalizedHost, gcodeRoot))
        {
            error = "G-code file '" + normalizedHost.string() + "' is outside '" +
                    gcodeRoot.string() + "'";
            return std::nullopt;
        }

        std::error_code ec;
        std::filesystem::path relative =
            std::filesystem::relative(normalizedHost, gcodeRoot, ec);
        if (ec)
        {
            error =
                "Failed to resolve relative path for '" + normalizedHost.string() + "'";
            return std::nullopt;
        }

        for (const auto& component : relative)
        {
            if (component == "..")
            {
                error = "G-code file '" + normalizedHost.string() +
                        "' escapes the gcodes directory";
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
    const std::string runString = runArg->generic_string();
    if (runString.size() > 2 && runString[1] == ':')
    {
        if (runString[0] != '0')
        {
            error = "gcode path '" + runString + "' must target 0:/gcodes";
            return std::nullopt;
        }

        std::string remainder = runString.substr(2);
        while (!remainder.empty() &&
               (remainder.front() == '/' || remainder.front() == '\\'))
        {
            remainder.erase(remainder.begin());
        }

        std::filesystem::path rrfPath(remainder);
        if (rrfPath.empty())
        {
            error = "gcode path '" + runString + "' is not valid";
            return std::nullopt;
        }

        auto iter = rrfPath.begin();
        if (iter == rrfPath.end() || iter->string() != "gcodes")
        {
            error = "gcode path '" + runString + "' must target 0:/gcodes";
            return std::nullopt;
        }

        std::filesystem::path relative;
        for (++iter; iter != rrfPath.end(); ++iter)
        {
            relative /= *iter;
        }

        if (relative.empty())
        {
            error = "gcode path '" + runString + "' is incomplete";
            return std::nullopt;
        }

        return toRelative(gcodeRoot / relative);
    }

    auto resolved = ResolveExistingPath(vsdRoot, *runArg, "G-code file", true, error);
    if (!resolved)
    {
        return std::nullopt;
    }

    return toRelative(*resolved);
}

bool ConfigureCapture(const CommandLineOptions& options,
                      const std::filesystem::path& vsdRoot) noexcept
{
    switch (options.capture)
    {
        case CaptureSelection::disabled:
            return HostCanCapture::Configure({});

        case CaptureSelection::enabled:
        {
            if (!options.captureArgument)
            {
                std::cerr << "CAN log path not specified\n";
                return false;
            }

            std::filesystem::path capturePath =
                ResolveOutputPath(vsdRoot, *options.captureArgument);
            const std::filesystem::path parent = capturePath.parent_path();

            if (!parent.empty())
            {
                std::error_code ec;
                std::filesystem::create_directories(parent, ec);
                if (ec)
                {
                    std::cerr << "Failed to create CAN log directory '" << parent
                              << "': " << ec.message() << '\n';
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
    uint64_t lastCaptureCount = HostCanCapture::GetCaptureCount();
    unsigned int captureIdleCycles = 0;
    unsigned int fastForwardAttempts = 0;
    bool seenCapture = (lastCaptureCount != 0);

    for (;;)
    {
        reprap.Spin();

        const uint64_t currentCaptureCount = HostCanCapture::GetCaptureCount();
        if (currentCaptureCount != lastCaptureCount)
        {
            lastCaptureCount = currentCaptureCount;
            captureIdleCycles = 0;
            fastForwardAttempts = 0;
            seenCapture = true;
        }
        else if (currentCaptureCount != 0)
        {
            if (captureIdleCycles < kIdleSettlingCycles)
            {
                ++captureIdleCycles;
            }
            else
            {
                ++fastForwardAttempts;
                const uint64_t latestFinish =
                    HostCanCapture::GetLatestFinishMasterClock();
                if (latestFinish != 0)
                {
                    const uint64_t advance = static_cast<uint64_t>(std::min<unsigned int>(
                                                 fastForwardAttempts, 3600U)) *
                                             HostTiming::StepClockFrequencyHz;
                    HostTiming::EnsureMasterClockAtLeast(latestFinish + advance);
                }
            }
        }
        else
        {
            captureIdleCycles = 0;
        }

        const bool fileIdle = fileBuffer->IsCompletelyIdle();
        const bool moveIdle = reprap.GetMove().NoLiveMovement();

        if (seenCapture)
        {
            if (currentCaptureCount != 0)
            {
                if (fileIdle && moveIdle && captureIdleCycles >= kIdleSettlingCycles)
                {
                    return true;
                }
                idleCycles = 0;
            }
            else
            {
                if (fileIdle && moveIdle)
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
            }
        }
        else
        {
            if (fileIdle && moveIdle)
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
        }

        if constexpr (kTraceCompletion)
        {
            static uint64_t debugCounter = 0;
            if ((debugCounter++ % 1000ULL) == 0)
            {
                std::cout << "[wait] captures=" << currentCaptureCount
                          << " captureIdle=" << captureIdleCycles
                          << " seen=" << seenCapture << " fileIdle=" << fileIdle
                          << " moveIdle=" << moveIdle
                          << " scheduled=" << reprap.GetMove().GetScheduledMoves()
                          << " completed=" << reprap.GetMove().GetCompletedMoves()
                          << '\n';
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //std::this_thread::yield();

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
    }
}

void VirtuallyHomeAxesIfNeeded() noexcept
{
    auto& gcodes = reprap.GetGCodes();
    if (gcodes.AllAxesAreHomed())
    {
        return;
    }

    const size_t visibleAxes = gcodes.GetVisibleAxes();
    const char* axisLetters = gcodes.GetAxisLetters();

    std::vector<std::string> homedAxes;
    homedAxes.reserve(visibleAxes);

    for (size_t axis = 0; axis < visibleAxes; ++axis)
    {
        if (!gcodes.IsAxisHomed(static_cast<unsigned int>(axis)))
        {
            gcodes.SetAxisIsHomed(static_cast<unsigned int>(axis));

            if (axisLetters != nullptr && axisLetters[axis] != '\0')
            {
                homedAxes.emplace_back(1, axisLetters[axis]);
            }
            else
            {
                homedAxes.emplace_back("axis" + std::to_string(axis));
            }
        }
    }

    if (!homedAxes.empty())
    {
        std::cout << "Host marked axes as homed: ";
        for (size_t i = 0; i < homedAxes.size(); ++i)
        {
            if (i != 0)
            {
                std::cout << ", ";
            }
            std::cout << homedAxes[i];
        }
        std::cout << '\n';
    }
}

void ReportFinalPosition() noexcept
{
    float machine[MaxAxes] = {0.0f};
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

    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    if (ok)
    {
        std::cout << "Completed G-code '" << relativePath << "' in " << elapsed.count()
                  << " ms\n";
    }
    else
    {
        std::cerr << "Failed to complete G-code '" << relativePath << "'\n";
    }
    return ok;
}
}  // namespace

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
        MassStorage::ClearSysConfigOverride();
    };

    try
    {
        std::filesystem::path vsdRoot = NormalisePath(options.vsdRoot);

        std::error_code dirError;
        std::filesystem::create_directories(vsdRoot, dirError);
        if (dirError)
        {
            std::cerr << "Failed to prepare VSD directory '" << vsdRoot
                      << "': " << dirError.message() << '\n';
            cleanup();
            return 1;
        }

        MassStorage::ClearSysConfigOverride();

        if (options.configArgument)
        {
            std::string configError;
            auto resolvedConfig =
                ResolveExistingPath(vsdRoot, *options.configArgument,
                                    "Configuration file", true, configError);
            if (!resolvedConfig)
            {
                std::cerr << configError << '\n';
                cleanup();
                return 1;
            }
            if (!PrepareConfigFile(vsdRoot, *resolvedConfig))
            {
                cleanup();
                return 1;
            }
        }

        MassStorage::SetHostRoot(vsdRoot.string());
        MassStorage::Init();

        if (!ConfigureCapture(options, vsdRoot))
        {
            cleanup();
            return 1;
        }

        CanMessageBuffer::Init(kDefaultCanBuffers);
        CanMotion::Init();

        reprap.Init();
        reprapInitialised = true;
        reprap.GetGCodes().HostForceSimulationMode(SimulationMode::off);
        VirtuallyHomeAxesIfNeeded();
        HostTiming::Reset();
        HostCanCapture::Reset();

        bool success = true;
        if (options.gcodeArgument)
        {
            std::string resolveError;
            auto relative = ResolveRunFile(vsdRoot, options.gcodeArgument, resolveError);
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
