#include "SocketHost.h"

#ifdef RRF_HOST_BUILD

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketHost::SocketHost(NetworkInterface* iface, int fdIn, TcpPort listenPort,
                       IPAddress remoteIp, TcpPort remotePortIn) noexcept
    : Socket(iface), fd(fdIn)
{
    localPort = listenPort;
    remotePort = remotePortIn;
    protocol = HttpProtocol;
    remoteIPAddress = remoteIp;

    const int currentFlags = fcntl(fd, F_GETFL, 0);
    if (currentFlags != -1)
    {
        fcntl(fd, F_SETFL, currentFlags | O_NONBLOCK);
    }
}

SocketHost::~SocketHost() noexcept
{
    CloseFd();
}

void SocketHost::Poll() noexcept
{
    FillBuffer();
}

void SocketHost::Close() noexcept
{
    if (!closed)
    {
        CloseFd();
        closed = true;
    }
}

void SocketHost::Terminate() noexcept
{
    Close();
}

void SocketHost::TerminateAndDisable() noexcept
{
    disabled = true;
    Close();
}

bool SocketHost::ReadChar(char& c) noexcept
{
    const uint8_t* buffer;
    size_t len;
    if (ReadBuffer(buffer, len) && len != 0)
    {
        c = static_cast<char>(buffer[0]);
        Taken(1);
        return true;
    }
    return false;
}

bool SocketHost::ReadBuffer(const uint8_t*& buffer, size_t& len) noexcept
{
    FillBuffer();

    if (writePos > readPos)
    {
        buffer = readBuffer.data() + readPos;
        len = writePos - readPos;
        return true;
    }

    buffer = nullptr;
    len = 0;
    return false;
}

void SocketHost::Taken(size_t len) noexcept
{
    const size_t available = (writePos > readPos) ? (writePos - readPos) : 0;
    const size_t consume = (len > available) ? available : len;
    readPos += consume;

    if (readPos >= writePos)
    {
        readPos = writePos = 0;
    }
}

bool SocketHost::CanRead() const noexcept
{
    return !closed && !disabled && fd >= 0;
}

bool SocketHost::CanSend() const noexcept
{
    return !closed && !disabled && fd >= 0;
}

size_t SocketHost::Send(const uint8_t* data, size_t length) noexcept
{
    if (!CanSend() || length == 0)
    {
        return 0;
    }

    ssize_t ret = send(fd, reinterpret_cast<const char*>(data), length,
                       MSG_DONTWAIT
#ifdef MSG_NOSIGNAL
                       | MSG_NOSIGNAL
#endif
    );

    if (ret > 0)
    {
        return static_cast<size_t>(ret);
    }

    if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
    {
        return 0;
    }

    Close();
    return 0;
}

void SocketHost::Send() noexcept
{
    // No buffered send state to flush for the host socket.
}

bool SocketHost::FillBuffer() noexcept
{
    if (!CanRead())
    {
        return false;
    }

    if (readPos == writePos)
    {
        readPos = writePos = 0;
    }
    else if (writePos == readBuffer.size() && readPos != 0)
    {
        const size_t remaining = writePos - readPos;
        memmove(readBuffer.data(), readBuffer.data() + readPos, remaining);
        readPos = 0;
        writePos = remaining;
    }

    bool readAny = false;
    while (writePos < readBuffer.size())
    {
        ssize_t ret = recv(fd, reinterpret_cast<char*>(readBuffer.data() + writePos),
                           readBuffer.size() - writePos, MSG_DONTWAIT);

        if (ret > 0)
        {
            writePos += static_cast<size_t>(ret);
            readAny = true;
            continue;
        }
        else if (ret == 0)
        {
            Close();
            break;
        }
        else if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
            break;
        }
        else
        {
            Close();
            break;
        }
    }

    return readAny;
}

void SocketHost::CloseFd() noexcept
{
    if (fd >= 0)
    {
        shutdown(fd, SHUT_RDWR);
        close(fd);
        fd = -1;
    }
}

#endif
