// license:BSD-3-Clause
// copyright-holders:S. Smith,David Haywood,Fabio Priuli
/***********************************************************************************************************

 Neo Geo cart emulation
 Standard cart type, possibly with bankswitch in the area beyond 0x100000
 (We also include here V-Liner, which uses a standard cart + RAM + some custom input)

 ***********************************************************************************************************/


#include "emu.h"
#include "rom.h"


//-------------------------------------------------
//  neogeo_rom_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(NEOGEO_ROM, neogeo_rom_device, "neocart_rom", "Neo Geo Standard Carts")


neogeo_rom_device::neogeo_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint16_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_neogeo_cart_interface(mconfig, *this)
{
}

neogeo_rom_device::neogeo_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint16_t clock) :
	neogeo_rom_device(mconfig, NEOGEO_ROM, tag, owner, clock)
{
}


//-------------------------------------------------
//  mapper specific start/reset
//-------------------------------------------------

void neogeo_rom_device::device_start()
{
}

void neogeo_rom_device::device_reset()
{
}


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

READ16_MEMBER(neogeo_rom_device::rom_r)
{
	// to speed up access to ROM, the access to ROM are actually replaced in the driver
	// by accesses to the maincpu rom region, where we have anyway copied the rom content
	uint16_t* rom = (get_rom_size()) ? get_rom_base() : get_region_rom_base();
	return rom[offset];
}


WRITE16_MEMBER(neogeo_rom_device::banksel_w)
{
	// to speed up access to ROM, the banking is taken care of at driver level
	// by mapping higher banks to the corresponding offset in maincpu rom region
}



/*************************************************
 V-Liner : this is plain NeoGeo cart + RAM
 **************************************************/

DEFINE_DEVICE_TYPE(NEOGEO_VLINER_CART, neogeo_vliner_cart_device, "neocart_vliner", "Neo Geo V-Liner Cart")

neogeo_vliner_cart_device::neogeo_vliner_cart_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	neogeo_rom_device(mconfig, NEOGEO_VLINER_CART, tag, owner, clock)
{
}


void neogeo_vliner_cart_device::device_start()
{
	save_item(NAME(m_cart_ram));
}

void neogeo_vliner_cart_device::device_reset()
{
	memset(m_cart_ram, 0, 0x2000);
}
