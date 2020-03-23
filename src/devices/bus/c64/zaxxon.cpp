// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Zaxxon/Super Zaxxon cartridge emulation

**********************************************************************/

#include "emu.h"
#include "zaxxon.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(C64_ZAXXON, c64_zaxxon_cartridge_device, "c64_zaxxon", "C64 Zaxxon cartridge")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  c64_zaxxon_cartridge_device - constructor
//-------------------------------------------------

c64_zaxxon_cartridge_device::c64_zaxxon_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, C64_ZAXXON, tag, owner, clock),
	device_c64_expansion_card_interface(mconfig, *this),
	m_bank(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void c64_zaxxon_cartridge_device::device_start()
{
	// state saving
	save_item(NAME(m_bank));
}


//-------------------------------------------------
//  c64_cd_r - cartridge data read
//-------------------------------------------------

uint8_t c64_zaxxon_cartridge_device::c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!roml)
	{
		data = m_roml[offset & 0xfff];

		m_bank = BIT(offset, 12);
	}
	else if (!romh)
	{
		offs_t addr = (m_bank << 13) | (offset & 0x1fff);
		data = m_romh[addr];
	}

	return data;
}
