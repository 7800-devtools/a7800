// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Luxor XEBEC Winchester controller card emulation

*********************************************************************/

#include "emu.h"
#include "hdc.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define Z80_TAG         "z80"
#define SASIBUS_TAG     "sasi"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ABC_HDC, abc_hdc_device, "abc_hdc", "ABC HDC")


//-------------------------------------------------
//  ROM( abc_hdc )
//-------------------------------------------------

ROM_START( abc_hdc )
	ROM_REGION( 0x800, Z80_TAG, 0 )
	ROM_SYSTEM_BIOS( 0, "st4038", "Seagate ST4038 (CHS: 733,5,17,512)" )
	ROMX_LOAD( "st4038.6c", 0x000, 0x800, CRC(4c803b87) SHA1(1141bb51ad9200fc32d92a749460843dc6af8953), ROM_BIOS(1) ) // Seagate ST4038 (http://stason.org/TULARC/pc/hard-drives-hdd/seagate/ST4038-1987-31MB-5-25-FH-MFM-ST412.html)
	ROM_SYSTEM_BIOS( 1, "st225", "Seagate ST225 (CHS: 615,4,17,512)" )
	ROMX_LOAD( "st225.6c",  0x000, 0x800, CRC(c9f68f81) SHA1(7ff8b2a19f71fe0279ab3e5a0a5fffcb6030360c), ROM_BIOS(2) ) // Seagate ST225 (http://stason.org/TULARC/pc/hard-drives-hdd/seagate/ST225-21MB-5-25-HH-MFM-ST412.html)
ROM_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *abc_hdc_device::device_rom_region() const
{
	return ROM_NAME( abc_hdc );
}


//-------------------------------------------------
//  ADDRESS_MAP( abc_hdc_mem )
//-------------------------------------------------

static ADDRESS_MAP_START( abc_hdc_mem, AS_PROGRAM, 8, abc_hdc_device )
	AM_RANGE(0x0000, 0x0ff) AM_ROM AM_REGION(Z80_TAG, 0)
ADDRESS_MAP_END


//-------------------------------------------------
//  ADDRESS_MAP( abc_hdc_io )
//-------------------------------------------------

static ADDRESS_MAP_START( abc_hdc_io, AS_IO, 8, abc_hdc_device )
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

MACHINE_CONFIG_MEMBER( abc_hdc_device::device_add_mconfig )
	MCFG_CPU_ADD(Z80_TAG, Z80, 4000000)
	MCFG_CPU_PROGRAM_MAP(abc_hdc_mem)
	MCFG_CPU_IO_MAP(abc_hdc_io)
	MCFG_Z80_DAISY_CHAIN(daisy_chain)

	MCFG_DEVICE_ADD(SASIBUS_TAG, SCSI_PORT, 0)
	MCFG_SCSIDEV_ADD(SASIBUS_TAG ":" SCSI_PORT_DEVICE1, "harddisk", SCSIHD, SCSI_ID_0)
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  abc_hdc_device - constructor
//-------------------------------------------------

abc_hdc_device::abc_hdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ABC_HDC, tag, owner, clock)
	, device_abcbus_card_interface(mconfig, *this)
	, m_maincpu(*this, Z80_TAG)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void abc_hdc_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void abc_hdc_device::device_reset()
{
}



//**************************************************************************
//  ABC BUS INTERFACE
//**************************************************************************

//-------------------------------------------------
//  abcbus_cs -
//-------------------------------------------------

void abc_hdc_device::abcbus_cs(uint8_t data)
{
}
