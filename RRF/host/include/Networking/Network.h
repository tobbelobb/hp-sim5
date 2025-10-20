#pragma once

#ifdef RRF_HOST_BUILD

#include <cstdint>
#include <cstring>

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <Networking/NetworkDefs.h>
#include <General/StringRef.h>
#include <Platform/OutputMemory.h>
#include <GCodeResult.h>

class Platform;
class GCodeBuffer;
class OutputBuffer;
class NetworkInterface;
class NetworkResponder;
class Socket;
class WifiFirmwareUploader;

class Network final : public ObjectModel
{
public:
	explicit Network(Platform& p) noexcept;
	Network(const Network&) = delete;

	void Init() noexcept;
	void Activate() noexcept {}
	void Exit() noexcept {}

	void Diagnostics(unsigned int part, const StringRef& reply) noexcept;
	unsigned int GetNumNetworkInterfaces() const noexcept;
	bool IsWiFiInterface(unsigned int) const noexcept;

	GCodeResult EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply) noexcept;
	GCodeResult EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept;
	GCodeResult DisableProtocol(unsigned int interface, NetworkProtocol protocol, const StringRef& reply) noexcept;
	GCodeResult ReportProtocols(unsigned int interface, const StringRef& reply) const noexcept;

	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& longReply) noexcept;
	WifiFirmwareUploader *_ecv_null GetWifiUploader() const noexcept { return nullptr; }
	void ResetWiFiForUpload(bool) noexcept {}
	const char *_ecv_array GetWiFiServerVersion() const noexcept { return nullptr; }

	GCodeResult ConfigureNetworkProtocol(GCodeBuffer& gb, const StringRef& reply) noexcept;
	GCodeResult GetNetworkState(unsigned int interface, const StringRef& reply) noexcept;
	int EnableState(unsigned int) const noexcept { return 0; }

	IPAddress GetIPAddress(unsigned int) const noexcept { return ipAddress; }
	void SetReportedIPAddress(IPAddress ip) noexcept { reportedIpAddress = ip; }
	void SetEthernetIPAddress(IPAddress ip, IPAddress nm, IPAddress gw) noexcept;
	IPAddress GetNetmask(unsigned int) const noexcept { return netMask; }
	IPAddress GetGateway(unsigned int) const noexcept { return gateway; }
	bool UsingDhcp(unsigned int) const noexcept { return usingDhcp; }
	GCodeResult SetMacAddress(unsigned int, const MacAddress& mac, const StringRef& reply) noexcept;
	const MacAddress& GetMacAddress(unsigned int) const noexcept { return macAddress; }
	const char *_ecv_array GetHostname() const noexcept { return hostname; }
	void SetHostname(const char *_ecv_array name) noexcept;

	void TerminateResponders(const NetworkInterface *, NetworkProtocol) noexcept {}
	bool FindResponder(Socket *, NetworkProtocol) noexcept { return false; }

	bool StartClient(NetworkInterface *, NetworkProtocol) noexcept { return false; }
	void StopClient(NetworkInterface *, NetworkProtocol) noexcept {}

	void HandleHttpGCodeReply(const char *_ecv_array msg) noexcept;
	void HandleTelnetGCodeReply(const char *_ecv_array msg) noexcept;
	void HandleHttpGCodeReply(OutputBuffer *buf) noexcept;
	void HandleTelnetGCodeReply(OutputBuffer *buf) noexcept;

	void MqttPublish(const char *, const char *, int, bool, bool) noexcept {}
	uint32_t GetHttpReplySeq() noexcept { return ++httpReplySeq; }

	void CreateAdditionalInterface() noexcept {}

protected:
	const ObjectModelClassDescriptor *_ecv_null GetObjectModelClassDescriptor() const noexcept override { return nullptr; }
	const ObjectModelArrayTableEntry *_ecv_null GetObjectModelArrayEntry(unsigned int) const noexcept override { return nullptr; }

private:
	static constexpr const char *_ecv_array UnsupportedMessage = "Networking not implemented on host build";

	void ReleaseBuffer(OutputBuffer *buf) noexcept;

	Platform& platform;
	IPAddress ipAddress;
	IPAddress netMask;
	IPAddress gateway;
	IPAddress reportedIpAddress;
	MacAddress macAddress;
	char hostname[16];
	bool usingDhcp{true};
	uint32_t httpReplySeq{0};
};

inline Network::Network(Platform& p) noexcept
	: platform(p),
	  ipAddress(DefaultIpAddress),
	  netMask(DefaultNetMask),
	  gateway(DefaultGateway),
	  reportedIpAddress(DefaultIpAddress)
{
	macAddress.SetDefault();
	std::strncpy(hostname, "rrf-host", sizeof(hostname));
	hostname[sizeof(hostname) - 1] = 0;
}

inline void Network::Init() noexcept
{
	httpReplySeq = 0;
	usingDhcp = true;
}

inline void Network::Diagnostics(unsigned int part, const StringRef& reply) noexcept
{
	(void)part;
	reply.copy(UnsupportedMessage);
}

inline unsigned int Network::GetNumNetworkInterfaces() const noexcept
{
	return 0;
}

inline bool Network::IsWiFiInterface(unsigned int) const noexcept
{
	return false;
}

inline GCodeResult Network::EnableInterface(unsigned int, int, const StringRef&, const StringRef& reply) noexcept
{
	reply.copy(UnsupportedMessage);
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Network::EnableProtocol(unsigned int, NetworkProtocol, int, uint32_t, int, const StringRef& reply) noexcept
{
	reply.copy(UnsupportedMessage);
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Network::DisableProtocol(unsigned int, NetworkProtocol, const StringRef& reply) noexcept
{
	reply.copy(UnsupportedMessage);
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Network::ReportProtocols(unsigned int, const StringRef& reply) const noexcept
{
	reply.copy("No network protocols enabled in host build");
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Network::HandleWiFiCode(int, GCodeBuffer&, const StringRef& reply, OutputBuffer *&) noexcept
{
	reply.copy(UnsupportedMessage);
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Network::ConfigureNetworkProtocol(GCodeBuffer&, const StringRef& reply) noexcept
{
	reply.copy(UnsupportedMessage);
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Network::GetNetworkState(unsigned int, const StringRef& reply) noexcept
{
	reply.copy("Network disabled");
	return GCodeResult::warningNotSupported;
}

inline void Network::SetEthernetIPAddress(IPAddress ip, IPAddress nm, IPAddress gw) noexcept
{
	ipAddress = ip;
	netMask = nm;
	gateway = gw;
	usingDhcp = false;
}

inline GCodeResult Network::SetMacAddress(unsigned int, const MacAddress& mac, const StringRef& reply) noexcept
{
	macAddress = mac;
	reply.copy("MAC stored locally only");
	return GCodeResult::warningNotSupported;
}

inline void Network::SetHostname(const char *_ecv_array name) noexcept
{
	if (name == nullptr || *name == 0)
	{
		std::strncpy(hostname, "rrf-host", sizeof(hostname));
	}
	else
	{
		std::strncpy(hostname, name, sizeof(hostname));
	}
	hostname[sizeof(hostname) - 1] = 0;
}

inline void Network::HandleHttpGCodeReply(const char *_ecv_array) noexcept
{
	// Intentionally dropped
}

inline void Network::HandleTelnetGCodeReply(const char *_ecv_array) noexcept
{
	// Intentionally dropped
}

inline void Network::ReleaseBuffer(OutputBuffer *buf) noexcept
{
	if (buf != nullptr)
	{
		OutputBuffer *_ecv_null volatile temp = buf;
		OutputBuffer::ReleaseAll(temp);
	}
}

inline void Network::HandleHttpGCodeReply(OutputBuffer *buf) noexcept
{
	ReleaseBuffer(buf);
}

inline void Network::HandleTelnetGCodeReply(OutputBuffer *buf) noexcept
{
	ReleaseBuffer(buf);
}

#endif // RRF_HOST_BUILD

