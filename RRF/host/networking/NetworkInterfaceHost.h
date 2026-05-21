#pragma once

#ifdef RRF_HOST_BUILD

#include <memory>
#include <vector>

#include <General/String.h>
#include <Networking/NetworkInterface.h>
#include "HostNetworkConfig.h"
#include "SocketHost.h"

class Platform;

class NetworkInterfaceHost final : public NetworkInterface
{
public:
    explicit NetworkInterfaceHost(Platform& platform) noexcept;
    ~NetworkInterfaceHost() noexcept override;

    void Init() noexcept override;
    void Activate() noexcept override;
    void Exit() noexcept override;
    void Spin() noexcept override;
    void Diagnostics(const StringRef& reply) noexcept override;

    GCodeResult EnableInterface(int mode, const StringRef& ssid,
                                const StringRef& reply, int tlsParam = 1) noexcept override;
    GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
    int EnableState() const noexcept override;
    bool IsWiFiInterface() const noexcept override;

    IPAddress GetIPAddress() const noexcept override;
    IPAddress GetNetmask() const noexcept override;
    IPAddress GetGateway() const noexcept override;
    bool UsingDhcp() const noexcept override;
    void SetIPAddress(IPAddress ip, IPAddress nm,
                      IPAddress gw) noexcept override;
    GCodeResult SetMacAddress(const MacAddress& mac,
                              const StringRef& reply) noexcept override;
    const MacAddress& GetMacAddress() const noexcept override;
    void UpdateHostname(const char* hostname) noexcept override;
    bool OpenDataPort(TcpPort, bool useTls = false) noexcept override
    {
        (void)useTls;
        return false;
    }
    void TerminateDataPort() noexcept override {}

protected:
    void IfaceShutdownProtocol(NetworkProtocol protocol,
                               bool permanent) noexcept override;
    void IfaceStartProtocol(NetworkProtocol protocol) noexcept override;
    const ObjectModelClassDescriptor *_ecv_null
    GetObjectModelClassDescriptor() const noexcept override
    {
        return nullptr;
    }
    const ObjectModelArrayTableEntry *_ecv_null
    GetObjectModelArrayEntry(unsigned int) const noexcept override
    {
        return nullptr;
    }

private:
    struct PendingConnection
    {
        std::unique_ptr<SocketHost> socket;
        bool attached{false};
    };

    void AcceptConnections() noexcept;
    void CloseAllConnections() noexcept;
    void StopListening() noexcept;
    bool StartListening(TcpPort port) noexcept;

    Platform& platform;
    int listenFd;
    TcpPort listenPort;
    bool usingDhcp;
    bool enabled;
    IPAddress ipAddress;
    IPAddress netMask;
    IPAddress gateway;
    MacAddress macAddress;
    String<16> hostname;
    std::vector<PendingConnection> connections;
};

#endif
