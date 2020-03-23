// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/***************************************************************************

    JCB Sound Extension Module

    The Dragon 32 Sound Extension Module is a cartridge by J.C.B. (Microsystems),
    that contains a General Instruments AY-3-8910 sound chip. This allows the
    Dragon to play interesting sound effects and complex chiptunes without
    taking all processor time.
    The cartridge also adds two 8-bit I/O ports (provided also by the AY-3-8910).
    The sound chip can be operated via the new commands in BASIC provided in
    the cartridge ROM, or via the 0xFEFE and 0xFEFF addresses.

***************************************************************************/

#include "emu.h"
#include "dragon_jcbsnd.h"
#include "speaker.h"


ROM_START( dragon_jcbsnd )
	ROM_REGION(0x8000, "eprom", ROMREGION_ERASE00)
	ROM_LOAD("D32SEM.ROM", 0x0000, 0x1000, CRC(4cd0f30b) SHA1(d07bb9272e3d3928059853730ff656905a80b68e))
ROM_END

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(DRAGON_JCBSND, dragon_jcbsnd_device, "dragon_jcbsnd", "Dragon Sound Extension Module")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  dragon_jcbsnd_device - constructor
//-------------------------------------------------

dragon_jcbsnd_device::dragon_jcbsnd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, DRAGON_JCBSND, tag, owner, clock)
	, device_cococart_interface(mconfig, *this )
	, m_ay8910(*this, "ay8910")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void dragon_jcbsnd_device::device_start()
{
	m_cart = dynamic_cast<cococart_slot_device *>(owner());
}

//-------------------------------------------------
//  device_reset - device-specific startup
//-------------------------------------------------

void dragon_jcbsnd_device::device_reset()
{
	set_line_value(line::CART, line_value::Q);

	address_space& space = machine().device("maincpu")->memory().space(AS_PROGRAM);
	space.install_write_handler(0xfefe, 0xfefe, WRITE8_DEVICE_DELEGATE(m_ay8910, ay8910_device, address_w));
	space.install_readwrite_handler(0xfeff, 0xfeff, READ8_DEVICE_DELEGATE(m_ay8910, ay8910_device, data_r), WRITE8_DEVICE_DELEGATE(m_ay8910, ay8910_device, data_w));
}

//-------------------------------------------------
//  dragon_jcbsnd_device::get_cart_base
//-------------------------------------------------

uint8_t* dragon_jcbsnd_device::get_cart_base()
{
	return memregion("eprom")->base();
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( dragon_jcbsnd_device::device_add_mconfig )
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("ay8910", AY8910, DERIVED_CLOCK(1, 4)) /* AY-3-8910 - clock not verified */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)
MACHINE_CONFIG_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *dragon_jcbsnd_device::device_rom_region() const
{
	return ROM_NAME( dragon_jcbsnd );
}
