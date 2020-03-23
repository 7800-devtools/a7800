// license:BSD-3-Clause
// copyright-holders:hap
/**********************************************************************

    Fujitsu MB8421/22/31/32-90/-90L/-90LL/-12/-12L/-12LL
    CMOS 16K-bit (2KB) dual-port SRAM

    MB84x2 lacks interrupt pins, it's basically as simple as AM_RAM AM_SHARE("x")
    MB843x is same as MB842x, except that it supports slave mode. It makes
    sure there are no clashes, with the _BUSY pin.

**********************************************************************/

#include "emu.h"
#include "machine/mb8421.h"


DEFINE_DEVICE_TYPE(MB8421, mb8421_device, "mb8421", "MB8421 DPSRAM")

//-------------------------------------------------
//  mb8421_device - constructor
//-------------------------------------------------

mb8421_device::mb8421_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MB8421, tag, owner, clock),
		m_intl_handler(*this),
		m_intr_handler(*this)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mb8421_device::device_start()
{
	memset(m_ram, 0, 0x800);

	// resolve callbacks
	m_intl_handler.resolve_safe();
	m_intr_handler.resolve_safe();

	// state save
	save_item(NAME(m_ram));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mb8421_device::device_reset()
{
	m_intl_handler(0);
	m_intr_handler(0);
}



WRITE8_MEMBER(mb8421_device::left_w)
{
	offset &= 0x7ff;
	m_ram[offset] = data;

	if (offset == 0x7ff)
		m_intr_handler(1);
}

READ8_MEMBER(mb8421_device::left_r)
{
	offset &= 0x7ff;

	if (offset == 0x7fe && !machine().side_effect_disabled())
		m_intl_handler(0);

	return m_ram[offset];
}

WRITE8_MEMBER(mb8421_device::right_w)
{
	offset &= 0x7ff;
	m_ram[offset] = data;

	if (offset == 0x7fe)
		m_intl_handler(1);
}

READ8_MEMBER(mb8421_device::right_r)
{
	offset &= 0x7ff;

	if (offset == 0x7ff && !machine().side_effect_disabled())
		m_intr_handler(0);

	return m_ram[offset];
}
