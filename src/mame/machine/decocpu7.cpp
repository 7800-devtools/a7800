// license:BSD-3-Clause
// copyright-holders:David Haywood

#include "emu.h"
#include "decocpu7.h"

DEFINE_DEVICE_TYPE(DECO_CPU7, deco_cpu7_device, "decocpu7", "DECO CPU-7")


deco_cpu7_device::deco_cpu7_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	m6502_device(mconfig, DECO_CPU7, tag, owner, clock)
{
}

void deco_cpu7_device::device_start()
{
	mintf = new mi_decrypt;
	init();
}

void deco_cpu7_device::device_reset()
{
	m6502_device::device_reset();
	static_cast<mi_decrypt *>(mintf)->had_written = false;
}

uint8_t deco_cpu7_device::mi_decrypt::read_sync(uint16_t adr)
{
	uint8_t res = direct->read_byte(adr);
	if(had_written) {
		had_written = false;
		if((adr & 0x0104) == 0x0104)
			res = BITSWAP8(res, 6,5,3,4,2,7,1,0);
	}
	return res;
}

void deco_cpu7_device::mi_decrypt::write(uint16_t adr, uint8_t val)
{
	program->write_byte(adr, val);
	had_written = true;
}
