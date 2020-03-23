// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    NeoRAM cartridge emulation

**********************************************************************/

#include "emu.h"
#include "neoram.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(C64_NEORAM, c64_neoram_cartridge_device, "c64_neoram", "C64 NeoRAM cartridge")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  c64_neoram_cartridge_device - constructor
//-------------------------------------------------

c64_neoram_cartridge_device::c64_neoram_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, C64_NEORAM, tag, owner, clock),
	device_c64_expansion_card_interface(mconfig, *this),
	device_nvram_interface(mconfig, *this),
	m_bank(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void c64_neoram_cartridge_device::device_start()
{
	// allocate memory
	m_nvram.allocate(0x200000);

	// state saving
	save_item(NAME(m_bank));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void c64_neoram_cartridge_device::device_reset()
{
	m_bank = 0;
}


//-------------------------------------------------
//  c64_cd_r - cartridge data read
//-------------------------------------------------

uint8_t c64_neoram_cartridge_device::c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!io1)
	{
		offs_t addr = (m_bank << 8) | (offset & 0xff);
		data = m_nvram[addr];
	}

	return data;
}


//-------------------------------------------------
//  c64_cd_w - cartridge data write
//-------------------------------------------------

void c64_neoram_cartridge_device::c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!io1)
	{
		offs_t addr = (m_bank << 8) | (offset & 0xff);
		m_nvram[addr] = data;
	}
	else if (!io2)
	{
		if (BIT(offset, 0))
		{
			m_bank = ((data & 0x7f) << 6) | (m_bank & 0x3f);
		}
		else
		{
			m_bank = (m_bank & 0x1fc0) | (data & 0x3f);
		}
	}
}
