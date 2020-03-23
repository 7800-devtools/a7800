// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Ocean Software cartridge emulation

**********************************************************************/

/*

    Chase H.Q. 2: Special Criminal Investigation

    PCB Layout
    ----------

    |===========================|
    |=|                   LS02  |
    |=|                         |
    |=|                         |
    |=|    ROM0   ROM1          |
    |=|                   LS273 |
    |=|                         |
    |=|                         |
    |=|                         |
    |===========================|

    ROM0,1 - 28-pin ROM (Toshiba TC531000 pinout, markings scratched off)

*/

#include "emu.h"
#include "ocean.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(C64_OCEAN, c64_ocean_cartridge_device, "c64_ocean", "C64 Ocean cartridge")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  c64_ocean_cartridge_device - constructor
//-------------------------------------------------

c64_ocean_cartridge_device::c64_ocean_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, C64_OCEAN, tag, owner, clock),
	device_c64_expansion_card_interface(mconfig, *this),
	m_bank(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void c64_ocean_cartridge_device::device_start()
{
	// state saving
	save_item(NAME(m_bank));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void c64_ocean_cartridge_device::device_reset()
{
	m_bank = 0;
}


//-------------------------------------------------
//  c64_cd_r - cartridge data read
//-------------------------------------------------

uint8_t c64_ocean_cartridge_device::c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!roml && m_roml.bytes())
	{
		offs_t addr = (m_bank << 13) | (offset & 0x1fff);
		data = m_roml[addr & m_roml.mask()];
	}
	else if (!romh && m_romh.bytes())
	{
		offs_t addr = (m_bank << 13) | (offset & 0x1fff);
		data = m_romh[addr & m_romh.mask()];
	}
	else if (!io1)
	{
		return m_bank;
	}

	return data;
}


//-------------------------------------------------
//  c64_cd_w - cartridge data write
//-------------------------------------------------

void c64_ocean_cartridge_device::c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!io1)
	{
		m_bank = data & 0x3f;
	}
}
