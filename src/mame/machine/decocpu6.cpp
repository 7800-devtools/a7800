// license:BSD-3-Clause
// copyright-holders:David Haywood
/* apparently Deco CPU-6 used by ProGolf
 just seems to be a bitswap on the opcodes like 222, but not the same one
 not a complex scheme like CPU-7?
*/


#include "emu.h"
#include "decocpu6.h"

DEFINE_DEVICE_TYPE(DECO_CPU6, deco_cpu6_device, "decocpu6", "DECO CPU-6")


deco_cpu6_device::deco_cpu6_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	m6502_device(mconfig, DECO_CPU6, tag, owner, clock)
{
}

void deco_cpu6_device::device_start()
{
	mintf = new mi_decrypt;
	init();
}

void deco_cpu6_device::device_reset()
{
	m6502_device::device_reset();
}

uint8_t deco_cpu6_device::mi_decrypt::read_sync(uint16_t adr)
{
	if (adr&1)
		return BITSWAP8(direct->read_byte(adr),6,4,7,5,3,2,1,0);
	else
		return direct->read_byte(adr);
}
