#include "UsbDefs.h"
#include "UsbEthernetDevice.h"
#ifdef interface
#undef interface
#endif
#include "IopBios.h"
#include "Log.h"
#include "Ps2Const.h"
#include "states/RegisterState.h"
#include <algorithm>
#include <cassert>
#include <cstring>

#ifndef _WIN32
#include <fcntl.h>
#endif

using namespace Iop;

#define LOG_NAME "iop_usb_ethernet"

#define STATE_REG_DESCRIPTORMEMPTR ("descriptorMemPtr")
#define STATE_REG_ENDPOINTSCANINDEX ("endpointScanIndex")
#define STATE_REG_RXACTIVE ("rxActive")
#define STATE_REG_RXBUFFERPTR ("rxBufferPtr")
#define STATE_REG_RXSIZE ("rxSize")
#define STATE_REG_RXCB ("rxCb")
#define STATE_REG_RXCBARG ("rxCbArg")
#define STATE_REG_INTRACTIVE ("intrActive")
#define STATE_REG_INTRBUFFERPTR ("intrBufferPtr")
#define STATE_REG_INTRSIZE ("intrSize")
#define STATE_REG_INTRCB ("intrCb")
#define STATE_REG_INTRCBARG ("intrCbArg")
#define STATE_REG_INTRTICKS ("intrTicks")

namespace
{
	// Namco System246 UE PCB (ADMtek Pegasus-based) USB IDs & registers
	constexpr uint16 PEGASUS_VENDOR_ID = 0x0B9A;
	constexpr uint16 PEGASUS_PRODUCT_ID = 0x0500;

	constexpr uint8 PEGASUS_REQ_GET_REGS = 0xF0;
	constexpr uint8 PEGASUS_REQ_SET_REGS = 0xF1;
	constexpr uint8 PEGASUS_REQ_SET_REG = 0xF1;

	constexpr uint8 REG_ETH_CTRL0 = 0x00;
	constexpr uint8 REG_ETH_CTRL1 = 0x01;
	constexpr uint8 REG_ETH_CTRL2 = 0x02;
	constexpr uint8 REG_ETH_ID0 = 0x10;
	constexpr uint8 REG_EPROM_OFFSET = 0x20;
	constexpr uint8 REG_EPROM_DATA_LO = 0x21;
	constexpr uint8 REG_EPROM_DATA_HI = 0x22;
	constexpr uint8 REG_EPROM_CTRL = 0x23;
	constexpr uint8 REG_PHY_ADDR = 0x25;
	constexpr uint8 REG_PHY_DATA_LO = 0x26;
	constexpr uint8 REG_PHY_DATA_HI = 0x27;
	constexpr uint8 REG_PHY_CTRL = 0x28;
	constexpr uint8 REG_USB_STAT = 0x2A;
	constexpr uint8 REG_GPIO_0 = 0x7E;
	constexpr uint8 REG_GPIO_1 = 0x7F;
	constexpr uint8 REG_CHIP_ID = 0x83;

	constexpr uint8 EPROM_READ = 0x02;
	constexpr uint8 EPROM_DONE = 0x04;

	constexpr uint8 PHY_WRITE = 0x20;
	constexpr uint8 PHY_READ = 0x40;
	constexpr uint8 PHY_DONE = 0x80;

	constexpr uint32 DESCRIPTOR_DEVICE_OFFSET = 0x00;
	constexpr uint32 DESCRIPTOR_CONFIG_OFFSET = 0x20;
	constexpr uint32 DESCRIPTOR_IFACE_OFFSET = 0x30;
	constexpr uint32 DESCRIPTOR_EP1_OFFSET = 0x40;
	constexpr uint32 DESCRIPTOR_EP2_OFFSET = 0x50;
	constexpr uint32 DESCRIPTOR_EP3_OFFSET = 0x60;
}

CEthernetUsbDevice::CEthernetUsbDevice(CIopBios& bios, uint8* ram)
    : m_bios(bios)
    , m_ram(ram)
{
	InitSocket();

	uint8 nodeIndex = static_cast<uint8>(m_localPort >= m_basePort ? (m_localPort - m_basePort) : 0);
	uint8 mac[6] = {0x00, 0x04, 0xA3, 0x24, 0x60, static_cast<uint8>(0x10 + nodeIndex)};

	memcpy(&m_regs[REG_ETH_ID0], mac, sizeof(mac));
	m_regs[REG_ETH_CTRL1] = 0x08;
	m_regs[REG_USB_STAT] = 0x01;
	m_regs[REG_CHIP_ID] = 0x01; // ADM8513

	m_eeprom[0] = static_cast<uint16>(mac[0] | (mac[1] << 8));
	m_eeprom[1] = static_cast<uint16>(mac[2] | (mac[3] << 8));
	m_eeprom[2] = static_cast<uint16>(mac[4] | (mac[5] << 8));
	m_eeprom[3] = 0x0000;
	m_eeprom[4] = PEGASUS_VENDOR_ID;
	m_eeprom[5] = PEGASUS_PRODUCT_ID;

	// Standard MII PHY registers (link up, 100Mbps full duplex, auto-nego complete)
	m_phyRegs[0] = 0x3100; // BMCR
	m_phyRegs[1] = 0x782D; // BMSR: Link valid + Auto-nego complete
	m_phyRegs[2] = 0x001D; // PHYIDR1
	m_phyRegs[3] = 0x2411; // PHYIDR2
	m_phyRegs[4] = 0x01E1; // ANAR
	m_phyRegs[5] = 0x45E1; // ANLPAR
}

CEthernetUsbDevice::~CEthernetUsbDevice()
{
	CloseSocket();
}

void CEthernetUsbDevice::InitSocket()
{
#ifdef _WIN32
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

	m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(m_socket == INVALID_SOCKET)
	{
		CLog::GetInstance().Warn(LOG_NAME, "Failed to create UDP socket.\r\n");
		return;
	}

#ifdef _WIN32
	u_long nonBlocking = 1;
	ioctlsocket(m_socket, FIONBIO, &nonBlocking);
#else
	int flags = fcntl(m_socket, F_GETFL, 0);
	fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);
#endif

	for(uint16 offset = 0; offset < m_maxNodes; offset++)
	{
		sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(m_basePort + offset);
		if(bind(m_socket, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0)
		{
			m_localPort = m_basePort + offset;
			CLog::GetInstance().Print(LOG_NAME, "Bound arcade USB Ethernet UDP bridge to port %u.\r\n", m_localPort);
			return;
		}
	}

	CLog::GetInstance().Warn(LOG_NAME, "Failed to bind any arcade USB Ethernet UDP port in range %u-%u.\r\n", m_basePort, m_basePort + m_maxNodes - 1);
	CloseSocket();
}

void CEthernetUsbDevice::CloseSocket()
{
	if(m_socket != INVALID_SOCKET)
	{
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
	}
}

void CEthernetUsbDevice::SaveState(CRegisterState& state) const
{
	state.SetRegister32(STATE_REG_DESCRIPTORMEMPTR, m_descriptorMemPtr);
	state.SetRegister32(STATE_REG_ENDPOINTSCANINDEX, m_endpointScanIndex);
	if(!m_rxTransfers.empty())
	{
		const auto& rx = m_rxTransfers.front();
		state.SetRegister32(STATE_REG_RXACTIVE, 1);
		state.SetRegister32(STATE_REG_RXBUFFERPTR, rx.bufferPtr);
		state.SetRegister32(STATE_REG_RXSIZE, rx.size);
		state.SetRegister32(STATE_REG_RXCB, rx.doneCb);
		state.SetRegister32(STATE_REG_RXCBARG, rx.arg);
	}
	else
	{
		state.SetRegister32(STATE_REG_RXACTIVE, 0);
		state.SetRegister32(STATE_REG_RXBUFFERPTR, 0);
		state.SetRegister32(STATE_REG_RXSIZE, 0);
		state.SetRegister32(STATE_REG_RXCB, 0);
		state.SetRegister32(STATE_REG_RXCBARG, 0);
	}
	state.SetRegister32(STATE_REG_INTRACTIVE, m_intrTransfer.active ? 1 : 0);
	state.SetRegister32(STATE_REG_INTRBUFFERPTR, m_intrTransfer.bufferPtr);
	state.SetRegister32(STATE_REG_INTRSIZE, m_intrTransfer.size);
	state.SetRegister32(STATE_REG_INTRCB, m_intrTransfer.doneCb);
	state.SetRegister32(STATE_REG_INTRCBARG, m_intrTransfer.arg);
	state.SetRegister32(STATE_REG_INTRTICKS, m_intrTicks);
}

void CEthernetUsbDevice::LoadState(const CRegisterState& state)
{
	m_descriptorMemPtr = state.GetRegister32(STATE_REG_DESCRIPTORMEMPTR);
	m_endpointScanIndex = static_cast<uint8>(state.GetRegister32(STATE_REG_ENDPOINTSCANINDEX));
	m_rxTransfers.clear();
	if(state.GetRegister32(STATE_REG_RXACTIVE) != 0)
	{
		PENDING_TRANSFER rx = {};
		rx.active = true;
		rx.bufferPtr = state.GetRegister32(STATE_REG_RXBUFFERPTR);
		rx.size = state.GetRegister32(STATE_REG_RXSIZE);
		rx.doneCb = state.GetRegister32(STATE_REG_RXCB);
		rx.arg = state.GetRegister32(STATE_REG_RXCBARG);
		m_rxTransfers.push_back(rx);
	}
	m_intrTransfer.active = state.GetRegister32(STATE_REG_INTRACTIVE) != 0;
	m_intrTransfer.bufferPtr = state.GetRegister32(STATE_REG_INTRBUFFERPTR);
	m_intrTransfer.size = state.GetRegister32(STATE_REG_INTRSIZE);
	m_intrTransfer.doneCb = state.GetRegister32(STATE_REG_INTRCB);
	m_intrTransfer.arg = state.GetRegister32(STATE_REG_INTRCBARG);
	m_intrTicks = state.GetRegister32(STATE_REG_INTRTICKS);
}

uint16 CEthernetUsbDevice::GetId() const
{
	return DEVICE_ID;
}

const char* CEthernetUsbDevice::GetLldName() const
{
	return "pegasus";
}

void CEthernetUsbDevice::CountTicks(uint32 ticks)
{
	PollSocket();

	if(!m_pendingCallbacks.empty())
	{
		auto cb = m_pendingCallbacks.front();
		m_pendingCallbacks.pop_front();
		m_bios.TriggerCallback(cb.doneCb, 0, cb.size, cb.arg);
	}

	if(m_pendingCallbacks.empty() && !m_rxTransfers.empty() && !m_rxQueue.empty())
	{
		auto frame = std::move(m_rxQueue.front());
		m_rxQueue.pop_front();
		CompleteRxTransfer(frame);
	}

	if(m_intrTransfer.active)
	{
		m_intrTicks -= static_cast<int32>(ticks);
		if(m_intrTicks <= 0)
		{
			uint32 copySize = std::min<uint32>(m_intrTransfer.size, 8);
			uint8* buffer = m_ram + m_intrTransfer.bufferPtr;
			memset(buffer, 0, copySize);
			if(copySize >= 6)
			{
				buffer[5] = 0x01; // Link status up
			}
			uint32 cb = m_intrTransfer.doneCb;
			uint32 arg = m_intrTransfer.arg;
			m_intrTransfer.active = false;
			m_bios.TriggerCallback(cb, 0, copySize, arg);
		}
	}
}

void CEthernetUsbDevice::OnLldRegistered()
{
	if(m_descriptorMemPtr == 0)
	{
		m_descriptorMemPtr = m_bios.GetSysmem()->AllocateMemory(0x100, 0, 0);
	}
	m_endpointScanIndex = 0;

	auto deviceDesc = reinterpret_cast<Usb::DEVICE_DESCRIPTOR*>(m_ram + m_descriptorMemPtr + DESCRIPTOR_DEVICE_OFFSET);
	memset(deviceDesc, 0, sizeof(Usb::DEVICE_DESCRIPTOR));
	deviceDesc->base.length = sizeof(Usb::DEVICE_DESCRIPTOR);
	deviceDesc->base.descriptorType = Usb::DESCRIPTOR_TYPE_DEVICE;
	deviceDesc->usbVersion = 0x0110;
	deviceDesc->maxPacketSize = 64;
	deviceDesc->vendorId = PEGASUS_VENDOR_ID;
	deviceDesc->productId = PEGASUS_PRODUCT_ID;
	deviceDesc->deviceVersion = 0x0101;
	deviceDesc->numConfigurations = 1;

	auto configDesc = reinterpret_cast<Usb::CONFIGURATION_DESCRIPTOR*>(m_ram + m_descriptorMemPtr + DESCRIPTOR_CONFIG_OFFSET);
	memset(configDesc, 0, sizeof(Usb::CONFIGURATION_DESCRIPTOR));
	configDesc->base.length = sizeof(Usb::CONFIGURATION_DESCRIPTOR);
	configDesc->base.descriptorType = Usb::DESCRIPTOR_TYPE_CONFIGURATION;
	configDesc->totalLength = sizeof(Usb::CONFIGURATION_DESCRIPTOR) + sizeof(Usb::INTERFACE_DESCRIPTOR) + (3 * sizeof(Usb::ENDPOINT_DESCRIPTOR));
	configDesc->numInterfaces = 1;
	configDesc->configurationValue = 1;
	configDesc->attributes = 0x80;
	configDesc->maxPower = 150;

	auto ifaceDesc = reinterpret_cast<Usb::INTERFACE_DESCRIPTOR*>(m_ram + m_descriptorMemPtr + DESCRIPTOR_IFACE_OFFSET);
	memset(ifaceDesc, 0, sizeof(Usb::INTERFACE_DESCRIPTOR));
	ifaceDesc->base.length = sizeof(Usb::INTERFACE_DESCRIPTOR);
	ifaceDesc->base.descriptorType = Usb::DESCRIPTOR_TYPE_INTERFACE;
	ifaceDesc->interfaceNumber = 0;
	ifaceDesc->numEndpoints = 3;

	auto ep1Desc = reinterpret_cast<Usb::ENDPOINT_DESCRIPTOR*>(m_ram + m_descriptorMemPtr + DESCRIPTOR_EP1_OFFSET);
	memset(ep1Desc, 0, sizeof(Usb::ENDPOINT_DESCRIPTOR));
	ep1Desc->base.length = sizeof(Usb::ENDPOINT_DESCRIPTOR);
	ep1Desc->base.descriptorType = Usb::DESCRIPTOR_TYPE_ENDPOINT;
	ep1Desc->endpointAddress = 0x81; // Bulk IN
	ep1Desc->attributes = 0x02;
	ep1Desc->maxPacketSize = 64;

	auto ep2Desc = reinterpret_cast<Usb::ENDPOINT_DESCRIPTOR*>(m_ram + m_descriptorMemPtr + DESCRIPTOR_EP2_OFFSET);
	memset(ep2Desc, 0, sizeof(Usb::ENDPOINT_DESCRIPTOR));
	ep2Desc->base.length = sizeof(Usb::ENDPOINT_DESCRIPTOR);
	ep2Desc->base.descriptorType = Usb::DESCRIPTOR_TYPE_ENDPOINT;
	ep2Desc->endpointAddress = 0x02; // Bulk OUT
	ep2Desc->attributes = 0x02;
	ep2Desc->maxPacketSize = 64;

	auto ep3Desc = reinterpret_cast<Usb::ENDPOINT_DESCRIPTOR*>(m_ram + m_descriptorMemPtr + DESCRIPTOR_EP3_OFFSET);
	memset(ep3Desc, 0, sizeof(Usb::ENDPOINT_DESCRIPTOR));
	ep3Desc->base.length = sizeof(Usb::ENDPOINT_DESCRIPTOR);
	ep3Desc->base.descriptorType = Usb::DESCRIPTOR_TYPE_ENDPOINT;
	ep3Desc->endpointAddress = 0x83; // Interrupt IN
	ep3Desc->attributes = 0x03;
	ep3Desc->maxPacketSize = 8;
	ep3Desc->interval = 1;
}

uint32 CEthernetUsbDevice::ScanStaticDescriptor(uint32 deviceId, uint32 descriptorPtr, uint32 descriptorType)
{
	assert(deviceId == DEVICE_ID);

	struct DescriptorEntry
	{
		uint32 offset;
		uint32 type;
	};

	static const DescriptorEntry descriptors[] = {
	    {DESCRIPTOR_DEVICE_OFFSET, Usb::DESCRIPTOR_TYPE_DEVICE},
	    {DESCRIPTOR_CONFIG_OFFSET, Usb::DESCRIPTOR_TYPE_CONFIGURATION},
	    {DESCRIPTOR_IFACE_OFFSET, Usb::DESCRIPTOR_TYPE_INTERFACE},
	    {DESCRIPTOR_EP1_OFFSET, Usb::DESCRIPTOR_TYPE_ENDPOINT},
	    {DESCRIPTOR_EP2_OFFSET, Usb::DESCRIPTOR_TYPE_ENDPOINT},
	    {DESCRIPTOR_EP3_OFFSET, Usb::DESCRIPTOR_TYPE_ENDPOINT},
	};

	bool foundPrev = (descriptorPtr == 0);
	for(const auto& entry : descriptors)
	{
		uint32 addr = m_descriptorMemPtr + entry.offset;
		if(!foundPrev)
		{
			if(addr == descriptorPtr)
			{
				foundPrev = true;
			}
			continue;
		}
		if(descriptorType == 0 || entry.type == descriptorType)
		{
			return addr;
		}
	}
	return 0;
}

int32 CEthernetUsbDevice::OpenPipe(uint32 deviceId, uint32 descriptorPtr)
{
	assert(deviceId == DEVICE_ID);
	if(descriptorPtr == 0)
	{
		return CONTROL_PIPE_ID;
	}
	if(descriptorPtr == m_descriptorMemPtr + DESCRIPTOR_EP1_OFFSET)
	{
		return BULK_IN_PIPE_ID;
	}
	if(descriptorPtr == m_descriptorMemPtr + DESCRIPTOR_EP2_OFFSET)
	{
		return BULK_OUT_PIPE_ID;
	}
	if(descriptorPtr == m_descriptorMemPtr + DESCRIPTOR_EP3_OFFSET)
	{
		return INTR_IN_PIPE_ID;
	}
	return CONTROL_PIPE_ID;
}

int32 CEthernetUsbDevice::TransferPipe(uint32 pipeId, uint32 bufferPtr, uint32 size, uint32 optionPtr, uint32 doneCb, uint32 arg)
{
	uint16 deviceId = (pipeId & 0xFFFF);
	uint16 internalPipeId = (pipeId >> 16) & 0xFFF;
	assert(deviceId == DEVICE_ID);

	switch(internalPipeId)
	{
	case CONTROL_PIPE_ID:
		HandleControlTransfer(bufferPtr, size, optionPtr, doneCb, arg);
		return 0;

	case BULK_IN_PIPE_ID:
	{
		PENDING_TRANSFER rx = {};
		rx.active = true;
		rx.bufferPtr = bufferPtr;
		rx.size = size;
		rx.doneCb = doneCb;
		rx.arg = arg;
		m_rxTransfers.push_back(rx);
		return 0;
	}

	case BULK_OUT_PIPE_ID:
	{
		const uint8* txBuf = m_ram + bufferPtr;
		if(size > 2)
		{
			uint16 frameLen = static_cast<uint16>(txBuf[0] | (txBuf[1] << 8));
			if(frameLen > 0 && static_cast<uint32>(frameLen + 2) <= size)
			{
				SendFrameToNetwork(txBuf + 2, frameLen);
			}
			else
			{
				SendFrameToNetwork(txBuf + 2, size - 2);
			}
		}
		if(doneCb != 0)
		{
			m_pendingCallbacks.push_back({doneCb, size, arg});
		}
		return 0;
	}

	case INTR_IN_PIPE_ID:
		m_intrTransfer.active = true;
		m_intrTransfer.bufferPtr = bufferPtr;
		m_intrTransfer.size = size;
		m_intrTransfer.doneCb = doneCb;
		m_intrTransfer.arg = arg;
		m_intrTicks = PS2::IOP_CLOCK_OVER_FREQ / 60;
		return 0;

	default:
		assert(false);
		return -1;
	}
}

void CEthernetUsbDevice::HandleControlTransfer(uint32 bufferPtr, uint32 size, uint32 optionPtr, uint32 doneCb, uint32 arg)
{
	if(optionPtr != 0)
	{
		const auto* setup = reinterpret_cast<const CONTROL_SETUP*>(m_ram + optionPtr);
		uint8* data = (bufferPtr != 0) ? (m_ram + bufferPtr) : nullptr;
		uint16 regIndex = setup->index & 0xFF;
		uint16 regCount = setup->length;
		if(setup->request != PEGASUS_REQ_GET_REGS)
		{
			CLog::GetInstance().Print(LOG_NAME, "ControlTransfer: reqType=0x%02X req=0x%02X val=0x%04X idx=0x%04X len=%u\r\n",
			                          setup->requestType, setup->request, setup->value, setup->index, setup->length);
		}

		if((setup->requestType & 0x80) != 0)
		{
			// Device-to-host read
			if(setup->request == PEGASUS_REQ_GET_REGS && data != nullptr)
			{
				for(uint16 i = 0; i < regCount; i++)
				{
					uint8 idx = static_cast<uint8>(regIndex + i);
					data[i] = m_regs[idx];
				}
			}
			else if(data != nullptr && size > 0)
			{
				memset(data, 0, size);
			}
		}
		else
		{
			// Host-to-device write
			if(setup->request == PEGASUS_REQ_SET_REGS)
			{
				if(regCount == 1 && data == nullptr)
				{
					// Set single register via wValue
					m_regs[regIndex] = static_cast<uint8>(setup->value & 0xFF);
					if(regIndex == REG_ETH_CTRL1)
					{
						m_regs[REG_ETH_CTRL1] &= ~0x08; // Auto-clear RESET_MAC bit
					}
					else if(regIndex == REG_EPROM_CTRL)
					{
						UpdateEepromRead();
					}
					else if(regIndex == REG_PHY_CTRL)
					{
						UpdatePhyOperation();
					}
				}
				else if(data != nullptr)
				{
					char valBuf[64] = {};
					uint16 logLen = (regCount < 16) ? regCount : 16;
					for(uint16 i = 0; i < logLen; i++)
					{
						snprintf(valBuf + (i * 3), sizeof(valBuf) - (i * 3), "%02X ", data[i]);
					}
					CLog::GetInstance().Print(LOG_NAME, "  SET_REGS idx=0x%02X len=%u: %s\r\n", regIndex, regCount, valBuf);
					for(uint16 i = 0; i < regCount; i++)
					{
						uint8 idx = static_cast<uint8>(regIndex + i);
						m_regs[idx] = data[i];
						if(idx == REG_ETH_CTRL1)
						{
							m_regs[REG_ETH_CTRL1] &= ~0x08; // Auto-clear RESET_MAC bit
						}
						else if(idx == REG_EPROM_CTRL)
						{
							UpdateEepromRead();
						}
						else if(idx == REG_PHY_CTRL)
						{
							UpdatePhyOperation();
						}
					}
				}
			}
		}
	}

	if(doneCb != 0)
	{
		m_pendingCallbacks.push_back({doneCb, size, arg});
	}
}

void CEthernetUsbDevice::UpdateEepromRead()
{
	if((m_regs[REG_EPROM_CTRL] & EPROM_READ) != 0)
	{
		uint8 offset = m_regs[REG_EPROM_OFFSET] & 0x3F;
		uint16 word = m_eeprom[offset];
		m_regs[REG_EPROM_DATA_LO] = static_cast<uint8>(word & 0xFF);
		m_regs[REG_EPROM_DATA_HI] = static_cast<uint8>((word >> 8) & 0xFF);
		m_regs[REG_EPROM_CTRL] = (m_regs[REG_EPROM_CTRL] & ~EPROM_READ) | EPROM_DONE;
	}
}

void CEthernetUsbDevice::UpdatePhyOperation()
{
	uint8 ctrl = m_regs[REG_PHY_CTRL];
	uint8 regAddr = ctrl & 0x1F;
	if((ctrl & PHY_READ) != 0)
	{
		uint16 val = m_phyRegs[regAddr];
		m_regs[REG_PHY_DATA_LO] = static_cast<uint8>(val & 0xFF);
		m_regs[REG_PHY_DATA_HI] = static_cast<uint8>((val >> 8) & 0xFF);
		m_regs[REG_PHY_CTRL] = (ctrl & ~PHY_READ) | PHY_DONE;
	}
	else if((ctrl & PHY_WRITE) != 0)
	{
		uint16 val = static_cast<uint16>(m_regs[REG_PHY_DATA_LO] | (m_regs[REG_PHY_DATA_HI] << 8));
		m_phyRegs[regAddr] = val;
		m_regs[REG_PHY_CTRL] = (ctrl & ~PHY_WRITE) | PHY_DONE;
	}
}

void CEthernetUsbDevice::SendFrameToNetwork(const uint8* frameData, uint32 frameSize)
{
	if(m_socket == INVALID_SOCKET || frameSize == 0)
	{
		return;
	}



	for(uint16 offset = 0; offset < m_maxNodes; offset++)
	{
		uint16 targetPort = m_basePort + offset;
		if(targetPort == m_localPort)
		{
			continue;
		}

		sockaddr_in destAddr = {};
		destAddr.sin_family = AF_INET;
		destAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		destAddr.sin_port = htons(targetPort);
		sendto(m_socket, reinterpret_cast<const char*>(frameData), static_cast<int>(frameSize), 0,
		       reinterpret_cast<const sockaddr*>(&destAddr), sizeof(destAddr));
	}
}

void CEthernetUsbDevice::PollSocket()
{
	if(m_socket == INVALID_SOCKET)
	{
		return;
	}

	uint8 recvBuf[2048];
	while(m_rxQueue.size() < 32)
	{
		sockaddr_in srcAddr = {};
		socklen_t srcAddrLen = sizeof(srcAddr);
		int received = recvfrom(m_socket, reinterpret_cast<char*>(recvBuf), sizeof(recvBuf), 0,
		                        reinterpret_cast<sockaddr*>(&srcAddr), &srcAddrLen);
		if(received <= 0)
		{
			break;
		}
		if(received < 14)
		{
			continue;
		}

		// Filter frames if not in promiscuous mode (EthCtrl2 bit 2 = promiscuous, bit 1 = all multicast)
		bool promiscuous = (m_regs[REG_ETH_CTRL2] & 0x04) != 0;
		bool isBroadcast = (recvBuf[0] == 0xFF && recvBuf[1] == 0xFF && recvBuf[2] == 0xFF &&
		                    recvBuf[3] == 0xFF && recvBuf[4] == 0xFF && recvBuf[5] == 0xFF);
		bool isMulticast = (recvBuf[0] & 0x01) != 0;
		bool isUnicastMatch = (memcmp(recvBuf, &m_regs[REG_ETH_ID0], 6) == 0);

		if(!promiscuous && !isBroadcast && !isMulticast && !isUnicastMatch)
		{
			continue;
		}

		m_rxQueue.emplace_back(recvBuf, recvBuf + received);
	}
}

void CEthernetUsbDevice::CompleteRxTransfer(const std::vector<uint8>& frameData)
{
	if(m_rxTransfers.empty())
	{
		return;
	}

	PENDING_TRANSFER rx = m_rxTransfers.front();
	m_rxTransfers.pop_front();

	// Pegasus II (reg 0x81 bit 1 set): 2-byte IP-align header + Ethernet frame + 4-byte CRC + 4-byte RX status trailer
	// Per Linux pegasus.c: pkt_len = (rx_status & 0x0fff) - 8, data starts at buf + 2
	uint32 headerSize = ((m_regs[0x81] & 0x02) != 0) ? 2 : 0;
	uint32 frameSize = static_cast<uint32>(frameData.size());
	uint32 totalPacketSize = headerSize + frameSize + 8;
	if(totalPacketSize > rx.size)
	{
		m_bios.TriggerCallback(rx.doneCb, 0, 0, rx.arg);
		return;
	}

	uint8* dst = m_ram + rx.bufferPtr;
	if(headerSize > 0)
	{
		// ADM8513 (Pegasus II): first 2 bytes hold packet + CRC + status length (frameSize + 8)
		uint16 hdrLen = static_cast<uint16>((frameSize + 8) & 0x0FFF);
		dst[0] = static_cast<uint8>(hdrLen & 0xFF);
		dst[1] = static_cast<uint8>((hdrLen >> 8) & 0xFF);
	}
	memcpy(dst + headerSize, frameData.data(), frameSize);
	memset(dst + headerSize + frameSize, 0, 4); // 4-byte FCS/CRC placeholder

	// ADM8513 4-byte RX status trailer: byte 0 = status/error flags (0 = OK), bytes 2..3 = length
	uint16 rxLen = static_cast<uint16>((frameSize + 8) & 0x0FFF);
	dst[headerSize + frameSize + 4] = 0x00;
	dst[headerSize + frameSize + 5] = 0x00;
	dst[headerSize + frameSize + 6] = static_cast<uint8>(rxLen & 0xFF);
	dst[headerSize + frameSize + 7] = static_cast<uint8>((rxLen >> 8) & 0xFF);


	m_bios.TriggerCallback(rx.doneCb, 0, totalPacketSize, rx.arg);
}
