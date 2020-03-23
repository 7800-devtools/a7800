// license:BSD-3-Clause
// copyright-holders:Curt Coder
#include "emu.h"
#include "slutprov.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ABC_SLUTPROV, abc_slutprov_device, "slutprov", "Slutprov")


//-------------------------------------------------
//  ROM( abc_slutprov )
//-------------------------------------------------

ROM_START( abc_slutprov )
	ROM_REGION( 0x800, "slutprov", 0 )
	ROM_LOAD( "slutprov.bin",   0x0000, 0x0800, CRC(a5bb56f4) SHA1(f97cb6526a1d10b189164f26157522e382ca6bc6) )
ROM_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *abc_slutprov_device::device_rom_region() const
{
	return ROM_NAME( abc_slutprov );
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  abc_slutprov_device - constructor
//-------------------------------------------------

abc_slutprov_device::abc_slutprov_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ABC_SLUTPROV, tag, owner, clock),
		device_abcbus_card_interface(mconfig, *this)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void abc_slutprov_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void abc_slutprov_device::device_reset()
{
}



//**************************************************************************
//  ABC BUS INTERFACE
//**************************************************************************

//-------------------------------------------------
//  abcbus_cs -
//-------------------------------------------------

void abc_slutprov_device::abcbus_cs(uint8_t data)
{
}
