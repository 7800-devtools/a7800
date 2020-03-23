// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    IQ151 Disc2 cartridge emulation

***************************************************************************/

#include "emu.h"
#include "disc2.h"
#include "formats/iq151_dsk.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

FLOPPY_FORMATS_MEMBER( iq151_disc2_device::floppy_formats )
	FLOPPY_IQ151_FORMAT
FLOPPY_FORMATS_END

static SLOT_INTERFACE_START( iq151_disc2_floppies )
	SLOT_INTERFACE( "8sssd", FLOPPY_8_SSSD )
SLOT_INTERFACE_END

ROM_START( iq151_disc2 )
	ROM_REGION(0x0800, "disc2", 0)
	ROM_LOAD( "iq151_disc2_12_5_1987_v4_0.rom", 0x0000, 0x0800, CRC(b189b170) SHA1(3e2ca80934177e7a32d0905f5a0ad14072f9dabf))
ROM_END

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(IQ151_DISC2, iq151_disc2_device, "iq151_disc2", "IQ151 Disc2")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  iq151_disc2_device - constructor
//-------------------------------------------------

iq151_disc2_device::iq151_disc2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, IQ151_DISC2, tag, owner, clock)
	, device_iq151cart_interface(mconfig, *this)
	, m_fdc(*this, "fdc"), m_rom(nullptr), m_rom_enabled(false)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void iq151_disc2_device::device_start()
{
	m_rom = (uint8_t*)memregion("disc2")->base();
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void iq151_disc2_device::device_reset()
{
	m_rom_enabled = false;
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( iq151_disc2_device::device_add_mconfig )
	MCFG_UPD765A_ADD("fdc", false, true)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", iq151_disc2_floppies, "8sssd", iq151_disc2_device::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:2", iq151_disc2_floppies, "8sssd", iq151_disc2_device::floppy_formats)
MACHINE_CONFIG_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *iq151_disc2_device::device_rom_region() const
{
	return ROM_NAME( iq151_disc2 );
}

//-------------------------------------------------
//  read
//-------------------------------------------------

void iq151_disc2_device::read(offs_t offset, uint8_t &data)
{
	// interal ROM is mapped at 0xe000-0xe7ff
	if (offset >= 0xe000 && offset < 0xe800 && m_rom_enabled)
		data = m_rom[offset & 0x7ff];
}


//-------------------------------------------------
//  IO read
//-------------------------------------------------

void iq151_disc2_device::io_read(offs_t offset, uint8_t &data)
{
	/* This is gross */
	address_space *space = nullptr;
	if (offset == 0xaa)
		data = m_fdc->msr_r(*space, 0, 0xff);
	else if (offset == 0xab)
		data = m_fdc->fifo_r(*space, 0, 0xff);
}

//-------------------------------------------------
//  IO write
//-------------------------------------------------

void iq151_disc2_device::io_write(offs_t offset, uint8_t data)
{
	address_space *space = nullptr;
	if (offset == 0xab)
		m_fdc->fifo_w(*space, 0, data, 0xff);
	else if (offset == 0xac)
		m_rom_enabled = (data == 0x01);
}
