// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************

 SG-1000 Card Catcher emulation

 Sega Card Catcher is a passthrough adapter for
 SG-1000 to load games in MyCard format into the
 main cartslot

 ***********************************************************************************************************/


#include "emu.h"
#include "ccatch.h"


//-------------------------------------------------
//  constructors
//-------------------------------------------------

DEFINE_DEVICE_TYPE(SEGA8_ROM_CARDCATCH, sega8_cardcatch_device, "sega8_ccatch", "SG-1000 Card Catcher Cart")



sega8_cardcatch_device::sega8_cardcatch_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: sega8_rom_device(mconfig, SEGA8_ROM_CARDCATCH, tag, owner, clock), m_card(*this, "cardslot")
{
}


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

READ8_MEMBER(sega8_cardcatch_device::read_cart)
{
	if (offset < 0x8000)
		return m_card->read_cart(space, offset);

	return 0xff;
}

WRITE8_MEMBER(sega8_cardcatch_device::write_cart)
{
	// this should never happen, because there is no RAM on cards
	if (offset < 0x8000)
		logerror("Attempt to write to MyCard\n");
}

static SLOT_INTERFACE_START(sg1000_card)
	SLOT_INTERFACE_INTERNAL("rom",  SEGA8_ROM_STD)
SLOT_INTERFACE_END

MACHINE_CONFIG_MEMBER( sega8_cardcatch_device::device_add_mconfig )
	MCFG_SG1000_CARD_ADD("cardslot", sg1000_card, nullptr)
MACHINE_CONFIG_END
