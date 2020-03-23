// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    z88.c

    Z88 RAM cartridges emulation

***************************************************************************/

#include "emu.h"
#include "ram.h"

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(Z88_32K_RAM,   z88_32k_ram_device,   "z88_32k_ram",   "Z88 32KB RAM")
DEFINE_DEVICE_TYPE(Z88_128K_RAM,  z88_128k_ram_device,  "z88_128k_ram",  "Z88 128KB RAM")
DEFINE_DEVICE_TYPE(Z88_512K_RAM,  z88_512k_ram_device,  "z88_512k_ram",  "Z88 512KB RAM")
DEFINE_DEVICE_TYPE(Z88_1024K_RAM, z88_1024k_ram_device, "z88_1024k_ram", "Z88 1024KB RAM")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  z88_32k_ram_device - constructor
//-------------------------------------------------

z88_32k_ram_device::z88_32k_ram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_ram_device(mconfig, Z88_32K_RAM, tag, owner, clock)
{
}

z88_32k_ram_device::z88_32k_ram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock), device_z88cart_interface(mconfig, *this), m_ram(nullptr)
{
}

//-------------------------------------------------
//  z88_128k_ram_device - constructor
//-------------------------------------------------

z88_128k_ram_device::z88_128k_ram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_ram_device(mconfig, Z88_128K_RAM, tag, owner, clock)
{
}

//-------------------------------------------------
//  z88_512k_ram_device - constructor
//-------------------------------------------------

z88_512k_ram_device::z88_512k_ram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_ram_device(mconfig, Z88_512K_RAM, tag, owner, clock)
{
}

//-------------------------------------------------
//  z88_1024k_ram_device - constructor
//-------------------------------------------------

z88_1024k_ram_device::z88_1024k_ram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z88_32k_ram_device(mconfig, Z88_1024K_RAM, tag, owner, clock)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void z88_32k_ram_device::device_start()
{
	m_ram = machine().memory().region_alloc(tag(), get_cart_size(), 1, ENDIANNESS_LITTLE)->base();
	memset(m_ram, 0, get_cart_size());
}

/*-------------------------------------------------
    get_cart_base
-------------------------------------------------*/

uint8_t* z88_32k_ram_device::get_cart_base()
{
	return m_ram;
}

/*-------------------------------------------------
    read
-------------------------------------------------*/

READ8_MEMBER(z88_32k_ram_device::read)
{
	return m_ram[offset & (get_cart_size() - 1)];
}

/*-------------------------------------------------
    write
-------------------------------------------------*/

WRITE8_MEMBER(z88_32k_ram_device::write)
{
	m_ram[offset & (get_cart_size() - 1)] = data;
}
