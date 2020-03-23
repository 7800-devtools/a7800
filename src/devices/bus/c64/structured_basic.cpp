// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Structured Basic cartridge emulation

**********************************************************************/

#include "emu.h"
#include "structured_basic.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(C64_STRUCTURED_BASIC, c64_structured_basic_cartridge_device, "c64_structured_basic", "C64 Structured Basic cartridge")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  c64_structured_basic_cartridge_device - constructor
//-------------------------------------------------

c64_structured_basic_cartridge_device::c64_structured_basic_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, C64_STRUCTURED_BASIC, tag, owner, clock),
	device_c64_expansion_card_interface(mconfig, *this),
	m_bank(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void c64_structured_basic_cartridge_device::device_start()
{
	// state saving
	save_item(NAME(m_bank));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void c64_structured_basic_cartridge_device::device_reset()
{
	m_exrom = 0;

	m_bank = 0;
}


//-------------------------------------------------
//  c64_cd_r - cartridge data read
//-------------------------------------------------

uint8_t c64_structured_basic_cartridge_device::c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!roml)
	{
		offs_t addr = (m_bank << 13) | (offset & 0x1fff);

		data = m_roml[addr];
	}

	return data;
}


//-------------------------------------------------
//  c64_cd_w - cartridge data write
//-------------------------------------------------

void c64_structured_basic_cartridge_device::c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!io1)
	{
		switch (data & 0x03)
		{
		case 0:
		case 1:
			m_exrom = 0;
			m_bank = 0;
			break;

		case 2:
			m_exrom = 0;
			m_bank = 1;
			break;

		case 3:
			m_exrom = 1;
			m_bank = 0;
			break;
		}
	}
}
