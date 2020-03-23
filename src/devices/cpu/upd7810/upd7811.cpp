// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller

/*

    uPD7811 - variant of uPD7810 with internal ROM

*/

#include "emu.h"
#include "upd7811.h"

/*
    the MODE pins can cause this to work with external ROM instead
    todo: document MODE pins

    M0  M1

    0   0 -
    0   1 -
    1   0 -
    1   1 -

*/

static ADDRESS_MAP_START( upd_internal_4096_rom_map, AS_PROGRAM, 8, upd7810_device )
	AM_RANGE(0x0000, 0x0fff) AM_ROM
	AM_RANGE(0xff00, 0xffff) AM_RAM
ADDRESS_MAP_END

DEFINE_DEVICE_TYPE(UPD7811, upd7811_device, "upd78c11", "uPD78C11")

upd7811_device::upd7811_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: upd7810_device(mconfig, UPD7811, tag, owner, clock, ADDRESS_MAP_NAME(upd_internal_4096_rom_map))
{
}
