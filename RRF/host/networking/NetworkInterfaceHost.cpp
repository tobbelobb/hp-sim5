#include "NetworkInterfaceHost.h"

#ifdef RRF_HOST_BUILD

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Platform/Platform.h>
#include <Platform/RepRap.h>

NetworkInterfaceHost::NetworkInterfaceHost(Platform& p) noexcept
    : platform(p),
      listenFd(-1),
      listenPort(static_cast<TcpPort>(HostNetworkConfig::GetHttpPort())),
      usingDhcp(true),
      enabled(false),
      ipAddress(DefaultIpAddress),
      netMask(DefaultNetMask),
      gateway(DefaultGateway)
{
    macAddress.SetDefault();
    hostname.copy("rrf-host");
}

NetworkInterfaceHost::~NetworkInterfaceHost() noexcept
{
    StopListening();
    CloseAllConnections();
}

void NetworkInterfaceHost::Init() noexcept
{
    interfaceMutex.Create("HostNet");
    listenPort = static_cast<TcpPort>(HostNetworkConfig::GetHttpPort());
    portNumbers[HttpProtocol] = listenPort;
    macAddress.SetDefault();
    hostname.copy("rrf-host");
}

void NetworkInterfaceHost::Activate() noexcept
{
    enabled = true;
    listenPort = static_cast<TcpPort>(HostNetworkConfig::GetHttpPort());
    portNumbers[HttpProtocol] = listenPort;
    SetState(NetworkState::active);
    IfaceStartProtocol(HttpProtocol);
}

void NetworkInterfaceHost::Exit() noexcept
{
    StopListening();
    CloseAllConnections();
    enabled = false;
    SetState(NetworkState::disabled);
}

void NetworkInterfaceHost::Spin() noexcept
{
    AcceptConnections();

    for (auto it = connections.begin(); it != connections.end();)
    {
        SocketHost* socket = it->socket.get();
        if (socket == nullptr || !socket->CanRead())
        {
            it = connections.erase(it);
            continue;
        }

        if (!it->attached)
        {
            if (reprap.GetNetwork().FindResponder(socket, HttpProtocol))
            {
                it->attached = true;
            }
        }
        else
        {
            socket->Poll();
        }

        if (socket->IsClosed())
        {
            it = connections.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void NetworkInterfaceHost::Diagnostics(const StringRef& reply) noexcept
{
    reply.lcatf(" HTTP(port=%u)", static_cast<unsigned int>(listenPort));
}

GCodeResult NetworkInterfaceHost::EnableInterface(int mode, const StringRef& ssid,
                                                  const StringRef& reply, bool tlsAllowed) noexcept
{
    (void)ssid;
    (void)tlsAllowed;
    if (mode == 0)
    {
        Exit();
        reply.copy("Network disabled");
        return GCodeResult::ok;
    }

    enabled = true;
    SetState(NetworkState::active);
    IfaceStartProtocol(HttpProtocol);
    reply.copy("Network enabled");
    return GCodeResult::ok;
}

GCodeResult NetworkInterfaceHost::GetNetworkState(const StringRef& reply) noexcept
{
    reply.copy(GetStateName());
    return GCodeResult::ok;
}

int NetworkInterfaceHost::EnableState() const noexcept
{
    return enabled ? 1 : 0;
}

bool NetworkInterfaceHost::IsWiFiInterface() const noexcept
{
    return false;
}

IPAddress NetworkInterfaceHost::GetIPAddress() const noexcept
{
    return ipAddress;
}

IPAddress NetworkInterfaceHost::GetNetmask() const noexcept
{
    return netMask;
}

IPAddress NetworkInterfaceHost::GetGateway() const noexcept
{
    return gateway;
}

bool NetworkInterfaceHost::UsingDhcp() const noexcept
{
    return usingDhcp;
}

void NetworkInterfaceHost::SetIPAddress(IPAddress ip, IPAddress nm,
                                         IPAddress gw) noexcept
{
    ipAddress = ip;
    netMask = nm;
    gateway = gw;
    usingDhcp = false;
}

GCodeResult NetworkInterfaceHost::SetMacAddress(const MacAddress& mac,
                                                const StringRef& reply) noexcept
{
    macAddress = mac;
    reply.copy("MAC stored locally");
    return GCodeResult::ok;
}

const MacAddress& NetworkInterfaceHost::GetMacAddress() const noexcept
{
    return macAddress;
}

void NetworkInterfaceHost::UpdateHostname(const char* name) noexcept
{
    if (name != nullptr && *name != 0)
    {
        hostname.copy(name);
    }
}

void NetworkInterfaceHost::IfaceShutdownProtocol(NetworkProtocol protocol,
                                                 bool) noexcept
{
    if (protocol == HttpProtocol)
    {
        StopListening();
        CloseAllConnections();
    }
}

void NetworkInterfaceHost::IfaceStartProtocol(NetworkProtocol protocol) noexcept
{
    if (protocol == HttpProtocol)
    {
        const TcpPort port = portNumbers[HttpProtocol];
        listenPort = port;
        StartListening(port);
    }
}

void NetworkInterfaceHost::AcceptConnections() noexcept
{
    if (listenFd < 0)
    {
        return;
    }

    for (;;)
    {
        sockaddr_in addr;
        socklen_t len = sizeof(addr);
        int client = accept(listenFd, reinterpret_cast<sockaddr*>(&addr), &len);
        if (client < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }
            break;
        }

        TcpPort remotePort = static_cast<TcpPort>(ntohs(addr.sin_port));
        IPAddress remoteIp;
        remoteIp.SetV4LittleEndian(ntohl(addr.sin_addr.s_addr));

        connections.push_back(
            {std::make_unique<SocketHost>(this, client, listenPort, remoteIp, remotePort),
             false});
    }
}

void NetworkInterfaceHost::CloseAllConnections() noexcept
{
    for (auto& conn : connections)
    {
        if (conn.socket)
        {
            conn.socket->TerminateAndDisable();
        }
    }
    connections.clear();
}

void NetworkInterfaceHost::StopListening() noexcept
{
    if (listenFd >= 0)
    {
        close(listenFd);
        listenFd = -1;
    }
}

bool NetworkInterfaceHost::StartListening(TcpPort port) noexcept
{
    StopListening();

    listenFd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd < 0)
    {
        platform.MessageF(LogWarn, "Failed to create HTTP socket: %s\n", std::strerror(errno));
        return false;
    }

    int flags = fcntl(listenFd, F_GETFL, 0);
    if (flags != -1)
    {
        fcntl(listenFd, F_SETFL, flags | O_NONBLOCK);
    }

    int reuse = 1;
    (void)setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(listenFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        platform.MessageF(LogWarn, "Failed to bind HTTP socket: %s\n", std::strerror(errno));
        StopListening();
        return false;
    }

    if (listen(listenFd, 8) < 0)
    {
        platform.MessageF(LogWarn, "Failed to listen on HTTP socket: %s\n", std::strerror(errno));
        StopListening();
        return false;
    }

    return true;
}

#endif
