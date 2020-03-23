// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2estd80col.c

    Apple IIe Standard 80 Column Card (2K of RAM, no double-hi-res)

*********************************************************************/

#include "emu.h"
#include "a2estd80col.h"


/***************************************************************************
    PARAMETERS
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2EAUX_STD80COL, a2eaux_std80col_device, "a2estd80", "Apple IIe Standard 80-Column Card")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a2eaux_std80col_device::a2eaux_std80col_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		a2eaux_std80col_device(mconfig, A2EAUX_STD80COL, tag, owner, clock)
{
}

a2eaux_std80col_device::a2eaux_std80col_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
		device_t(mconfig, type, tag, owner, clock),
		device_a2eauxslot_card_interface(mconfig, *this)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2eaux_std80col_device::device_start()
{
	set_a2eauxslot_device();
	save_item(NAME(m_ram));
}

void a2eaux_std80col_device::device_reset()
{
}

uint8_t a2eaux_std80col_device::read_auxram(uint16_t offset)
{
	if (offset < 0x800)
	{
		return m_ram[offset];
	}

	return 0xff;
}

void a2eaux_std80col_device::write_auxram(uint16_t offset, uint8_t data)
{
	if (offset < 0x800)
	{
		m_ram[offset] = data;
	}
}

uint8_t *a2eaux_std80col_device::get_vram_ptr()
{
	return &m_ram[0];
}

uint8_t *a2eaux_std80col_device::get_auxbank_ptr()
{
	return &m_ram[0];
}
