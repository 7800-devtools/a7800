// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Mega-Cart cartridge emulation

**********************************************************************/

#include "emu.h"
#include "megacart.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(VIC20_MEGACART, vic20_megacart_device, "vic20_megacart", "VIC-20 Mega-Cart")


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( vic20_megacart_device::device_add_mconfig )

MACHINE_CONFIG_END



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  vic20_megacart_device - constructor
//-------------------------------------------------

vic20_megacart_device::vic20_megacart_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, VIC20_MEGACART, tag, owner, clock)
	, device_vic20_expansion_card_interface(mconfig, *this)
	, device_nvram_interface(mconfig, *this)
	, m_nvram_en(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void vic20_megacart_device::device_start()
{
	m_nvram.allocate(0x2000);

	// state saving
	save_item(NAME(m_nvram_en));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void vic20_megacart_device::device_reset()
{
}


//-------------------------------------------------
//  vic20_cd_r - cartridge data read
//-------------------------------------------------

uint8_t vic20_megacart_device::vic20_cd_r(address_space &space, offs_t offset, uint8_t data, int ram1, int ram2, int ram3, int blk1, int blk2, int blk3, int blk5, int io2, int io3)
{
	if (!ram1 || !ram2 || !ram3 || !io2 || !io3)
	{
		if (m_nvram_en)
		{
			data = m_nvram[offset & 0x1fff];
		}
	}
	else if (!blk1 || !blk2 || !blk3)
	{
	}
	else if (!blk5)
	{
	}

	return data;
}


//-------------------------------------------------
//  vic20_cd_w - cartridge data write
//-------------------------------------------------

void vic20_megacart_device::vic20_cd_w(address_space &space, offs_t offset, uint8_t data, int ram1, int ram2, int ram3, int blk1, int blk2, int blk3, int blk5, int io2, int io3)
{
	if (!ram1 || !ram2 || !ram3 || !io2)
	{
		if (m_nvram_en)
		{
			m_nvram[offset & 0x1fff] = data;
		}
	}
	else if (!blk1 || !blk2 || !blk3)
	{
	}
	else if (!blk5)
	{
	}
	else if (!io3)
	{
		if (m_nvram_en)
		{
			m_nvram[offset & 0x1fff] = data;
		}
	}
}
