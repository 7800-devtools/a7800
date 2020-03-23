// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    SilverRock Productions cartridge emulation

**********************************************************************/

/*

    PCB Layout
    ----------

    e9048
    |=============|
    |=| LS00      |
    |=|      LS273|
    |=|           |
    |=|           |
    |=| ROM       |
    |=|           |
    |=|           |
    |=|           |
    |=============|

    ROM     - Atmel AT27C010-25PC 128Kx8 OTP EPROM "HUGO 2012"

*/

#include "emu.h"
#include "silverrock.h"



//**************************************************************************
//  MACROS/CONSTANTS
//**************************************************************************

#define UNSCRAMBLE_ADDRESS(_offset) \
	BITSWAP16(_offset,15,14,13,12,1,0,2,3,11,10,9,8,7,6,5,4)

#define UNSCRAMBLE_DATA(_data) \
	BITSWAP8(_data,7,6,5,4,0,1,2,3)



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(C64_SILVERROCK, c64_silverrock_cartridge_device, "c64_silverrock", "C64 SilverRock cartridge")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  c64_silverrock_cartridge_device - constructor
//-------------------------------------------------

c64_silverrock_cartridge_device::c64_silverrock_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, C64_SILVERROCK, tag, owner, clock),
	device_c64_expansion_card_interface(mconfig, *this), m_bank(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void c64_silverrock_cartridge_device::device_start()
{
	// state saving
	save_item(NAME(m_bank));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void c64_silverrock_cartridge_device::device_reset()
{
	m_bank = 0;
}


//-------------------------------------------------
//  c64_cd_r - cartridge data read
//-------------------------------------------------

uint8_t c64_silverrock_cartridge_device::c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!roml)
	{
		offs_t addr = (m_bank << 13) | (offset & 0x1fff);
		addr = (addr & 0x10000) | UNSCRAMBLE_ADDRESS(addr);
		data = UNSCRAMBLE_DATA(m_roml[addr]);
	}

	return data;
}


//-------------------------------------------------
//  c64_cd_w - cartridge data write
//-------------------------------------------------

void c64_silverrock_cartridge_device::c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2)
{
	if (!io1)
	{
		/*

		    bit     description

		    0
		    1
		    2
		    3
		    4       A14
		    5       A15
		    6       A16
		    7       A13

		*/

		m_bank = ((data >> 3) & 0x0e) | BIT(data, 7);
	}
}
