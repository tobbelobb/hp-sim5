#pragma once

#ifdef RRF_HOST_BUILD

#include <array>
#include <cstddef>
#include <cstdint>

#include <Networking/NetworkDefs.h>
#include <Networking/Socket.h>

class SocketHost final : public Socket
{
public:
    SocketHost(NetworkInterface* iface, int fd, TcpPort listenPort, IPAddress remoteIp,
               TcpPort remotePort) noexcept;
    ~SocketHost() noexcept;

    bool UsingTls() const noexcept override { return false; }
    void Poll() noexcept override;
    void Close() noexcept override;
    bool IsClosing() const noexcept override { return closed; }
    void Terminate() noexcept override;
    void TerminateAndDisable() noexcept override;
    bool ReadChar(char& c) noexcept override;
    bool ReadBuffer(const uint8_t*& buffer, size_t& len) noexcept override;
    void Taken(size_t len) noexcept override;
    bool CanRead() const noexcept override;
    bool CanSend() const noexcept override;
    size_t Send(const uint8_t* data, size_t length) noexcept override;
    void Send() noexcept override;

    bool IsClosed() const noexcept { return closed; }

private:
    bool FillBuffer() noexcept;
    void CloseFd() noexcept;

    static constexpr size_t BufferBytes = 64 * 1024;

    int fd{-1};
    bool closed{false};
    bool disabled{false};
    std::array<uint8_t, BufferBytes> readBuffer{};
    size_t readPos{0};
    size_t writePos{0};
};

#endif
