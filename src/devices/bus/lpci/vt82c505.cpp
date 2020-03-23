// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*

    VIA VT82C505 ISA/VL PCI bridge

*/

#include "emu.h"
#include "vt82c505.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

DEFINE_DEVICE_TYPE(VT82C505, vt82c505_device, "vt82c505_device", "VIA VT82C505 PCI bridge")


vt82c505_device::vt82c505_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, VT82C505, tag, owner, clock)
	, pci_device_interface(mconfig, *this)
{
}


uint32_t vt82c505_device::pci_read(pci_bus_device *pcibus, int function, int offset, uint32_t mem_mask)
{
	uint32_t result = 0;

	if (function != 0)
		return result;

	switch (offset)
	{
	case 0x00:  // vendor/device ID
		result = 0x05051106;
		break;
	case 0x04:  // command / status
		result = 0x00000007;
		break;
	case 0x80:  // DIP switch / revision, memory size, buffer control, VLB interface timing
		result = 0x00000100;
		break;
	}
	logerror("%s: PCI read func %i, offset %02x, result %08x\n",tag(),function,offset,result);

	return result;
}

void vt82c505_device::pci_write(pci_bus_device *pcibus, int function, int offset, uint32_t data, uint32_t mem_mask)
{
	if (function != 0)
		return;

	switch(offset)
	{
	case 0x84:
		if(ACCESSING_BITS_0_7)  // RX87: memory window 1 A31:24
		{
			m_window_addr[0] = (m_window_addr[0] & 0xff00) | (data & 0x000000ff);
			logerror("%s: memory window #1 A31:24 set: %04x\n",tag(),m_window_addr[0]);
		}
		break;
	case 0x88:
		if(ACCESSING_BITS_24_31)  // RX88: memory window 1 A23:16
		{
			m_window_addr[0] = (m_window_addr[0] & 0x00ff) | ((data & 0xff000000) >> 24);
			logerror("%s: memory window #1 A23:16 set: %04x\n",tag(),m_window_addr[0]);
		}
		if(ACCESSING_BITS_16_23)  // RX89: memory window 1 attributes
		{
			m_window_attr[0] = ((data & 0x00ff0000) >> 16);
			logerror("%s: memory window #1 attributes set: %02x\n",tag(),m_window_attr[0]);
		}
		break;
	}
	logerror("%s: PCI write func %i, offset %02x, data %08x\n",tag(),function,offset,data);
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void vt82c505_device::device_start()
{
	/* setup save states */
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void vt82c505_device::device_reset()
{
}
