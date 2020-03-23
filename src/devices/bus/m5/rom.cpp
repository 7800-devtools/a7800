// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************


 M5 cart emulation


 ***********************************************************************************************************/


#include "emu.h"
#include "rom.h"


//-------------------------------------------------
//  m5_rom_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(M5_ROM_STD, m5_rom_device, "m5_rom", "M5 Standard ROM Carts")
DEFINE_DEVICE_TYPE(M5_ROM_RAM, m5_ram_device, "m5_ram", "M5 Expansion memory cart")


m5_rom_device::m5_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_m5_cart_interface(mconfig, *this)
{
}

m5_rom_device::m5_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: m5_rom_device(mconfig, M5_ROM_STD, tag, owner, clock)
{
}


m5_ram_device::m5_ram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: m5_rom_device(mconfig, M5_ROM_RAM, tag, owner, clock)
{
}


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

READ8_MEMBER(m5_rom_device::read_rom)
{
	if (offset < m_rom_size)
		return m_rom[offset];
	else
		return 0xff;
}

READ8_MEMBER(m5_ram_device::read_ram)
{
	return m_ram[offset];
}

WRITE8_MEMBER(m5_ram_device::write_ram)
{
	m_ram[offset] = data;
}
