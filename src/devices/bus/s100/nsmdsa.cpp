// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    North Star MICRO-DISK System MDS-A (Single Density) emulation

**********************************************************************/

#include "emu.h"
#include "nsmdsa.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(S100_MDS_A, s100_mds_a_device, "s100_nsmdsa", "North Star MDS-A")


//-------------------------------------------------
//  ROM( mds_a )
//-------------------------------------------------

ROM_START( mds_a )
	ROM_REGION( 0x100, "psel", 0 )
	ROM_LOAD( "psel.7g", 0x000, 0x100, NO_DUMP ) // 74S287

	ROM_REGION( 0x100, "pgm", 0 )
	ROM_LOAD( "pgml.3f", 0x000, 0x100, NO_DUMP ) // 74S287
	ROM_LOAD( "pgmr.3e", 0x000, 0x100, NO_DUMP ) // 74S287
	ROM_LOAD( "horizon.bin", 0x000, 0x100, CRC(754e53e5) SHA1(875e42942d639b972252b87d86c3dc2133304967) BAD_DUMP )
ROM_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *s100_mds_a_device::device_rom_region() const
{
	return ROM_NAME( mds_a );
}


//-------------------------------------------------
//  SLOT_INTERFACE( mds_a_floppies )
//-------------------------------------------------

static SLOT_INTERFACE_START( mds_a_floppies )
	SLOT_INTERFACE( "525sd", FLOPPY_525_SD ) // Shugart SA-400
SLOT_INTERFACE_END


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( s100_mds_a_device::device_add_mconfig )
	MCFG_FLOPPY_DRIVE_ADD("floppy0", mds_a_floppies, "525sd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("floppy1", mds_a_floppies, "525sd", floppy_image_device::default_floppy_formats)
MACHINE_CONFIG_END



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  s100_mds_a_device - constructor
//-------------------------------------------------

s100_mds_a_device::s100_mds_a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, S100_MDS_A, tag, owner, clock),
	device_s100_card_interface(mconfig, *this),
	m_floppy0(*this, "floppy0"),
	m_floppy1(*this, "floppy1"),
	m_psel_rom(*this, "psel"),
	m_pgm_rom(*this, "pgm")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void s100_mds_a_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void s100_mds_a_device::device_reset()
{
}


//-------------------------------------------------
//  s100_smemr_r - memory read
//-------------------------------------------------

uint8_t s100_mds_a_device::s100_smemr_r(address_space &space, offs_t offset)
{
	return 0;
}
