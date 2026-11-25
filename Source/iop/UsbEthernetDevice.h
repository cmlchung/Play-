#pragma once

#include "UsbDevice.h"
#include "SocketDef.h"
#include <array>
#include <deque>
#include <vector>

class CIopBios;

namespace Iop
{
	class CEthernetUsbDevice : public CUsbDevice
	{
	public:
		enum
		{
			DEVICE_ID = 2,
			CONTROL_PIPE_ID = 0x100,
			BULK_IN_PIPE_ID = 0x101,
			BULK_OUT_PIPE_ID = 0x102,
			INTR_IN_PIPE_ID = 0x103,
		};

		CEthernetUsbDevice(CIopBios&, uint8*);
		virtual ~CEthernetUsbDevice();

		uint16 GetId() const override;
		const char* GetLldName() const override;

		void SaveState(CRegisterState&) const override;
		void LoadState(const CRegisterState&) override;

		void CountTicks(uint32) override;

		void OnLldRegistered() override;
		uint32 ScanStaticDescriptor(uint32, uint32, uint32) override;
		int32 OpenPipe(uint32, uint32) override;
		int32 TransferPipe(uint32, uint32, uint32, uint32, uint32, uint32) override;

	private:
		struct CONTROL_SETUP
		{
			uint8 requestType;
			uint8 request;
			uint16 value;
			uint16 index;
			uint16 length;
		};

		struct PENDING_TRANSFER
		{
			bool active = false;
			uint32 bufferPtr = 0;
			uint32 size = 0;
			uint32 doneCb = 0;
			uint32 arg = 0;
		};

		struct PENDING_CALLBACK
		{
			uint32 doneCb = 0;
			uint32 size = 0;
			uint32 arg = 0;
		};

		void InitSocket();
		void CloseSocket();
		void PollSocket();
		void SendFrameToNetwork(const uint8* frameData, uint32 frameSize);
		void CompleteRxTransfer(const std::vector<uint8>& frameData);
		void HandleControlTransfer(uint32 bufferPtr, uint32 size, uint32 optionPtr, uint32 doneCb, uint32 arg);
		void UpdateEepromRead();
		void UpdatePhyOperation();

		CIopBios& m_bios;
		uint8* m_ram = nullptr;

		uint32 m_descriptorMemPtr = 0;
		uint8 m_endpointScanIndex = 0;

		std::array<uint8, 256> m_regs = {};
		std::array<uint16, 64> m_eeprom = {};
		std::array<uint16, 32> m_phyRegs = {};

		std::deque<PENDING_TRANSFER> m_rxTransfers;
		std::deque<PENDING_CALLBACK> m_pendingCallbacks;
		PENDING_TRANSFER m_intrTransfer;
		int32 m_intrTicks = 0;
		int32 m_rxTicks = 0;

		std::deque<std::vector<uint8>> m_rxQueue;

		struct TARGET_ENDPOINT
		{
			uint32 addr = 0;
			uint16 port = 0;
		};

		SOCKET m_socket = INVALID_SOCKET;
		uint16 m_localPort = 0;
		uint16 m_basePort = 12460;
		uint16 m_maxNodes = 4;
		std::vector<TARGET_ENDPOINT> m_targets;
	};
}
