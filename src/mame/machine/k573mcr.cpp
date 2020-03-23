// license:BSD-3-Clause
// copyright-holders:smf
/*
 * Konami 573 Memory Card Reader
 *
 */

#include "emu.h"
#include "k573mcr.h"

/*
  GE885-PWB(A)A ( contains Toshiba tmpr3904af, ram, rom, tranceiver and glue ).
*/

k573mcr_device::k573mcr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, KONAMI_573_MEMORY_CARD_READER, tag, owner, clock)
{
}

void k573mcr_device::device_start()
{
}

ROM_START( k573mcr )
	ROM_REGION( 0x080000, "tmpr3904", 0 )
	ROM_LOAD( "885a01.bin",   0x000000, 0x080000, CRC(e22d093f) SHA1(927f62f63b5caa7899392decacd12fea0e6fdbea) )
ROM_END

const tiny_rom_entry *k573mcr_device::device_rom_region() const
{
	return ROM_NAME( k573mcr );
}

DEFINE_DEVICE_TYPE(KONAMI_573_MEMORY_CARD_READER, k573mcr_device, "k573mcr", "Konami 573 Memory Card Reader")
