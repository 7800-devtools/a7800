// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    rom.c

    Z88 ROM cartridges emulation

***************************************************************************/

#include "emu.h"
#include "rom.h"

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(Z88_32K_ROM,  z88_32k_rom_device,  "z88_32k_rom",  "Z88 32KB ROM")
DEFINE_DEVICE_TYPE(Z88_128K_ROM, z88_128k_rom_device, "z88_128k_rom", "Z88 128KB ROM")
DEFINE_DEVICE_TYPE(Z88_256K_ROM, z88_256k_rom_device, "z88_256k_rom", "Z88 256KB ROM")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  z88_32k_rom_device - constructor
//-------------------------------------------------

z88_32k_rom_device::z88_32k_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_rom_device(mconfig, Z88_32K_ROM, tag, owner, clock)
{
}

z88_32k_rom_device::z88_32k_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock), device_z88cart_interface(mconfig, *this), m_rom(nullptr)
{
}

//-------------------------------------------------
//  z88_128k_rom_device - constructor
//-------------------------------------------------

z88_128k_rom_device::z88_128k_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_rom_device(mconfig, Z88_128K_ROM, tag, owner, clock)
{
}

//-------------------------------------------------
//  z88_256k_rom_device - constructor
//-------------------------------------------------

z88_256k_rom_device::z88_256k_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_rom_device(mconfig, Z88_256K_ROM, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void z88_32k_rom_device::device_start()
{
	m_rom = machine().memory().region_alloc(tag(), get_cart_size(), 1, ENDIANNESS_LITTLE)->base();
}

/*-------------------------------------------------
    get_cart_base
-------------------------------------------------*/

uint8_t* z88_32k_rom_device::get_cart_base()
{
	return m_rom;
}

/*-------------------------------------------------
    read
-------------------------------------------------*/

READ8_MEMBER(z88_32k_rom_device::read)
{
	return m_rom[offset & (get_cart_size() - 1)];
}
