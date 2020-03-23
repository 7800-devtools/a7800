// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************

 A800 SpartaDOS cart emulation

***********************************************************************************************************/


#include "emu.h"
#include "sparta.h"


//-------------------------------------------------
//  constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(A800_ROM_SPARTADOS, a800_rom_spartados_device, "a800_sparta", "Atari 800 SpartaDOS ROM Carts")


a800_rom_spartados_device::a800_rom_spartados_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a800_rom_device(mconfig, A800_ROM_SPARTADOS, tag, owner, clock)
	, m_bank(0)
	, m_subslot_enabled(0)
{
}



void a800_rom_spartados_device::device_start()
{
	save_item(NAME(m_bank));
	save_item(NAME(m_subslot_enabled));
}

void a800_rom_spartados_device::device_reset()
{
	m_bank = 0;
	m_subslot_enabled = 0;
}


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

/*-------------------------------------------------

 SpartaDOS 64K carts

 Similar to Express / Diamond carts, because
 bankswitch is controlled by writing to 7 diff
 offsets in reverse order, but writes to offsets
 0x8-0xf also enable/disable subslot

 -------------------------------------------------*/

READ8_MEMBER(a800_rom_spartados_device::read_80xx)
{
	if (!m_subslot_enabled)
		return m_rom[(offset & 0x1fff) + (m_bank * 0x2000)];
	else
		return 0xff;    // subslot, currently not implemented
}

WRITE8_MEMBER(a800_rom_spartados_device::write_d5xx)
{
	if (offset & 0x08)
		m_subslot_enabled = !BIT(offset, 2);
	else
		m_bank = (offset ^ 0x07) & 0x0f;

}
