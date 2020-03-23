// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/**********************************************************************

    Poisk-1 HDC device (model B942)

**********************************************************************/

#include "emu.h"
#include "p1_hdc.h"


//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define VERBOSE_DBG 1

#define DBG_LOG(N,M,A) \
	do { \
		if(VERBOSE_DBG>=N) \
		{ \
			if( M ) \
				logerror("%11.6f: %-24s",machine().time().as_double(),(char*)M ); \
			logerror A; \
		} \
	} while (0)

#define KM1809VG7_TAG   "d17"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(P1_HDC, p1_hdc_device, "p1_hdc", "Poisk-1 MFM disk B942")


//-------------------------------------------------
//  ROM( p1_hdc )
//-------------------------------------------------

ROM_START( p1_hdc )
	ROM_REGION( 0x0800, "p1_hdc", 0 )
	ROM_DEFAULT_BIOS("v17")
	ROM_SYSTEM_BIOS(0, "v11", "ver 1.1") // (c) S. Kovalenko, 1990
	ROMX_LOAD( "b_hd_v11.rf2", 0x00000, 0x0800, CRC(a19c39b2) SHA1(57faa56b320abf801fedbed578cf97d253e5b777), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "v12", "ver 1.2") // (c) S. Kovalenko, 1991
	ROMX_LOAD( "p_hdd_nm.bin", 0x00000, 0x0800, CRC(d5f8e4cc) SHA1(5b533642df30958539715f87a7f25b0d66dd0861), ROM_BIOS(2))
	ROM_SYSTEM_BIOS(2, "v14", "ver 1.4") // (c) `.lesnyh. , 1992
	ROMX_LOAD( "b942_5mb.bin", 0x00000, 0x0800, CRC(a3cfa240) SHA1(0b0aa1ce839a957153bfbbe70310480ca9fe21b6), ROM_BIOS(3))
	ROM_SYSTEM_BIOS(3, "v17", "ver 1.7") // (c) S. Kovalenko, 1992
	ROMX_LOAD( "b942_v17.rom", 0x00000, 0x0800, CRC(869672e3) SHA1(5cf2ae0ec1fa3edb3fe882c805da0bbc3ac21792), ROM_BIOS(4))
ROM_END


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( p1_hdc_device::device_add_mconfig )
	MCFG_DEVICE_ADD(KM1809VG7_TAG, WD2010, 5000000) // XXX clock?
	MCFG_WD2010_IN_DRDY_CB(VCC)
	MCFG_WD2010_IN_INDEX_CB(VCC)
	MCFG_WD2010_IN_WF_CB(VCC)
	MCFG_WD2010_IN_TK000_CB(VCC)
	MCFG_WD2010_IN_SC_CB(VCC)

	MCFG_HARDDISK_ADD("hard0")
	MCFG_HARDDISK_ADD("hard1")
MACHINE_CONFIG_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *p1_hdc_device::device_rom_region() const
{
	return ROM_NAME(p1_hdc);
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************


READ8_MEMBER(p1_hdc_device::p1_HDC_r)
{
	uint8_t data = 0x00;

	switch (offset >> 8)
	{
	case 8:
		data = m_hdc->read(space, offset & 255);
	}
	DBG_LOG(1, "hdc", ("R $%04x == $%02x\n", offset, data));

	return data;
}

WRITE8_MEMBER(p1_hdc_device::p1_HDC_w)
{
	DBG_LOG(1, "hdc", ("W $%04x <- $%02x\n", offset, data));
	switch (offset >> 8)
	{
	case 8:
		m_hdc->write(space, offset & 255, data, 0);
	}
}

//-------------------------------------------------
//  p1_hdc_device - constructor
//-------------------------------------------------

p1_hdc_device::p1_hdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, P1_HDC, tag, owner, clock)
	, device_isa8_card_interface(mconfig, *this)
	, m_hdc(*this, KM1809VG7_TAG)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void p1_hdc_device::device_start()
{
	set_isa_device();
	m_isa->install_rom(this, 0xe2000, 0xe27ff, "XXX", "p1_hdc");
	m_isa->install_memory(0xd0000, 0xd0fff,
		READ8_DELEGATE(p1_hdc_device, p1_HDC_r),
		WRITE8_DELEGATE(p1_hdc_device, p1_HDC_w) );
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void p1_hdc_device::device_reset()
{
}
