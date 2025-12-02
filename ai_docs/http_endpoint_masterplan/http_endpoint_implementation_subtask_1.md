# Subtask 1: Add HTTP Server Mode to rrf_simulator

## Overview
Add an HTTP server mode to `rrf_simulator` that keeps the firmware running and accepts G-code commands via POST requests to `/machine/code`.

## Files to Modify/Create
- `RRF/host/src/main.cpp` - Add server mode
- `RRF/host/src/HttpServer.h` - New file (optional, can inline)
- `RRF/host/CMakeLists.txt` - Add cpp-httplib dependency

## Implementation Details

### 1. Add cpp-httplib Dependency
cpp-httplib is a header-only C++ HTTP library. Add it via FetchContent or as a submodule:

```cmake
# In RRF/host/CMakeLists.txt
include(FetchContent)
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.14.1
)
FetchContent_MakeAvailable(httplib)

target_link_libraries(rrf_simulator PRIVATE httplib::httplib)
```

### 2. Add Command Line Options
Extend `CommandLineOptions` in `main.cpp`:

```cpp
struct CommandLineOptions
{
    std::filesystem::path vsdRoot{"run/vsd"};
    std::optional<std::filesystem::path> gcodeArgument;
    CaptureSelection capture{CaptureSelection::notProvided};
    std::optional<std::filesystem::path> captureArgument;
    std::optional<std::filesystem::path> configArgument;
    bool showHelp{false};
    // NEW:
    bool serverMode{false};
    uint16_t serverPort{8080};
};
```

Add parsing:
```cpp
else if (arg == "--server" || arg == "-S")
{
    options.serverMode = true;
}
else if (arg == "--port" || arg == "-p")
{
    if (i + 1 >= argc) { error = "--port requires a port number"; return false; }
    options.serverPort = static_cast<uint16_t>(std::stoi(argv[++i]));
}
```

### 3. HTTP Server Implementation
Create a minimal HTTP server that handles `/machine/code`:

```cpp
#include <httplib.h>

class RrfHttpServer
{
public:
    RrfHttpServer(uint16_t port) : port_(port) {}

    void Start()
    {
        server_.Post("/machine/code", [](const httplib::Request& req, httplib::Response& res) {
            std::string gcode = req.body;
            std::string response = ProcessGCode(gcode);  // See Subtask 2
            res.set_content(response, "text/plain");
        });

        server_.Get("/machine/status", [](const httplib::Request& req, httplib::Response& res) {
            // Return basic status JSON
            res.set_content("{\"status\":\"idle\"}", "application/json");
        });

        std::cout << "HTTP server listening on port " << port_ << std::endl;
        server_.listen("0.0.0.0", port_);
    }

    void Stop() { server_.stop(); }

private:
    httplib::Server server_;
    uint16_t port_;
};
```

### 4. Main Loop for Server Mode
Add a server mode path in `main()`:

```cpp
if (options.serverMode)
{
    std::cout << "Starting in server mode on port " << options.serverPort << std::endl;

    // Start background spin thread
    std::atomic<bool> running{true};
    std::thread spinThread([&running]() {
        while (running.load()) {
            reprap.Spin();
            // Small sleep to prevent busy-waiting when idle
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    // Start HTTP server (blocks until stopped)
    RrfHttpServer server(options.serverPort);
    server.Start();

    running.store(false);
    spinThread.join();
}
else
{
    // Existing batch processing logic...
}
```

### 5. Graceful Shutdown
Handle SIGINT/SIGTERM for clean shutdown:

```cpp
#include <csignal>

std::atomic<bool> g_shutdown{false};

void signalHandler(int signal) {
    g_shutdown.store(true);
}

// In main():
std::signal(SIGINT, signalHandler);
std::signal(SIGTERM, signalHandler);
```

## Testing

### Unit Tests
Create `RRF/host/tests/test_http_server.cpp`:

```cpp
TEST(HttpServer, StartsAndStops) {
    RrfHttpServer server(0);  // Port 0 = auto-assign
    std::thread serverThread([&server]() { server.Start(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    server.Stop();
    serverThread.join();
}
```

### Integration Test Script
Create `tests/test_http_endpoint.sh`:

```bash
#!/bin/bash
set -e

# Start server in background
./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 2  # Wait for initialization

# Test basic connectivity
RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8080/machine/status)
if [ "$RESPONSE" != "200" ]; then
    echo "FAIL: Server not responding"
    kill $SERVER_PID
    exit 1
fi

# Cleanup
kill $SERVER_PID
echo "PASS: HTTP server mode works"
```

## Validation Criteria
1. `rrf_simulator --server` starts and listens on specified port
2. Server responds to GET /machine/status with 200
3. Server accepts POST to /machine/code
4. Firmware remains initialized and responsive
5. Clean shutdown on SIGINT

## Dependencies
- Subtask 2 (G-code injection) must be complete for /machine/code to work
- Can be developed in parallel with stubs returning "ok"

## Estimated Complexity
- HTTP library integration: Simple (header-only)
- Threading model: Moderate (need to coordinate Spin loop with HTTP)
- Signal handling: Simple
