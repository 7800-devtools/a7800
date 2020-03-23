// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    MyAB Turbo-Kontroller disk controller emulation

*********************************************************************/

#include "emu.h"
#include "turbo.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define Z80_TAG     "z80"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(TURBO_KONTROLLER, turbo_kontroller_device, "unidisk", "Turbo-Kontroller")


//-------------------------------------------------
//  ROM( turbo_kontroller )
//-------------------------------------------------

ROM_START( turbo_kontroller )
	ROM_REGION( 0x1000, Z80_TAG, 0 )
	ROM_SYSTEM_BIOS( 0, "525", "5\" 25-pin" )
	ROMX_LOAD( "unidis5d.bin", 0x0000, 0x1000, CRC(569dd60c) SHA1(47b810bcb5a063ffb3034fd7138dc5e15d243676), ROM_BIOS(1) )
	ROM_SYSTEM_BIOS( 1, "534", "5\" 34-pin" )
	ROMX_LOAD( "unidiskh.bin", 0x0000, 0x1000, CRC(5079ad85) SHA1(42bb91318f13929c3a440de3fa1f0491a0b90863), ROM_BIOS(2) )
	ROM_SYSTEM_BIOS( 2, "8", "8\"" )
	ROMX_LOAD( "unidisk8.bin", 0x0000, 0x1000, CRC(d04e6a43) SHA1(8db504d46ff0355c72bd58fd536abeb17425c532), ROM_BIOS(3) )
ROM_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *turbo_kontroller_device::device_rom_region() const
{
	return ROM_NAME( turbo_kontroller );
}


//-------------------------------------------------
//  ADDRESS_MAP( turbo_kontroller_mem )
//-------------------------------------------------

static ADDRESS_MAP_START( turbo_kontroller_mem, AS_PROGRAM, 8, turbo_kontroller_device )
	AM_RANGE(0x0000, 0x0fff) AM_ROM AM_REGION(Z80_TAG, 0)
ADDRESS_MAP_END


//-------------------------------------------------
//  ADDRESS_MAP( turbo_kontroller_io )
//-------------------------------------------------

static ADDRESS_MAP_START( turbo_kontroller_io, AS_IO, 8, turbo_kontroller_device )
ADDRESS_MAP_END


//-------------------------------------------------
//  z80_daisy_config daisy_chain
//-------------------------------------------------

static const z80_daisy_config daisy_chain[] =
{
	{ nullptr }
};


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( turbo_kontroller_device::device_add_mconfig )
	MCFG_CPU_ADD(Z80_TAG, Z80, 4000000)
	MCFG_CPU_PROGRAM_MAP(turbo_kontroller_mem)
	MCFG_CPU_IO_MAP(turbo_kontroller_io)
	MCFG_Z80_DAISY_CHAIN(daisy_chain)
MACHINE_CONFIG_END



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  turbo_kontroller_device - constructor
//-------------------------------------------------

turbo_kontroller_device::turbo_kontroller_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TURBO_KONTROLLER, tag, owner, clock),
		device_abcbus_card_interface(mconfig, *this),
		m_maincpu(*this, Z80_TAG)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void turbo_kontroller_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void turbo_kontroller_device::device_reset()
{
}



//**************************************************************************
//  ABC BUS INTERFACE
//**************************************************************************

//-------------------------------------------------
//  abcbus_cs -
//-------------------------------------------------

void turbo_kontroller_device::abcbus_cs(uint8_t data)
{
}
