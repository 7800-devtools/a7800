// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore VIC-1010 Expansion Module emulation

**********************************************************************/

#include "emu.h"
#include "vic1010.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(VIC1010, vic1010_device, "vic1010", "VIC-1010 Expansion Module")


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( vic1010_device::device_add_mconfig )
	MCFG_VIC20_PASSTHRU_EXPANSION_SLOT_ADD("slot1")
	MCFG_VIC20_PASSTHRU_EXPANSION_SLOT_ADD("slot2")
	MCFG_VIC20_PASSTHRU_EXPANSION_SLOT_ADD("slot3")
	MCFG_VIC20_PASSTHRU_EXPANSION_SLOT_ADD("slot4")
	MCFG_VIC20_PASSTHRU_EXPANSION_SLOT_ADD("slot5")
	MCFG_VIC20_PASSTHRU_EXPANSION_SLOT_ADD("slot6")
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  vic1010_device - constructor
//-------------------------------------------------

vic1010_device::vic1010_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, VIC1010, tag, owner, clock)
	, device_vic20_expansion_card_interface(mconfig, *this)
	, m_expansion_slot(*this, "slot%u", 1)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void vic1010_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void vic1010_device::device_reset()
{
	for (auto & elem : m_expansion_slot)
	{
		elem->reset();
	}
}


//-------------------------------------------------
//  vic20_cd_r - cartridge data read
//-------------------------------------------------

uint8_t vic1010_device::vic20_cd_r(address_space &space, offs_t offset, uint8_t data, int ram1, int ram2, int ram3, int blk1, int blk2, int blk3, int blk5, int io2, int io3)
{
	for (auto elem : m_expansion_slot)
	{
		uint8_t slot_data = elem->cd_r(space, offset, data, ram1, ram2, ram3, blk1, blk2, blk3, blk5, io2, io3);

		if (data != slot_data)
		{
			data = slot_data;
		}
	}

	return data;
}


//-------------------------------------------------
//  vic20_cd_w - cartridge data write
//-------------------------------------------------

void vic1010_device::vic20_cd_w(address_space &space, offs_t offset, uint8_t data, int ram1, int ram2, int ram3, int blk1, int blk2, int blk3, int blk5, int io2, int io3)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->cd_w(space, offset, data, ram1, ram2, ram3, blk1, blk2, blk3, blk5, io2, io3);
	}
}
