// license:BSD-3-Clause
// copyright-holders:David Haywood,Alex Marshall
/***************************************************************************

 NMK004 emulation

***************************************************************************/

#include "emu.h"
#include "nmk004.h"

#include "sound/2203intf.h"
#include "sound/okim6295.h"


WRITE8_MEMBER( nmk004_device::write )
{
	machine().scheduler().synchronize();
	to_nmk004 = data;
}

READ8_MEMBER( nmk004_device::read )
{
	machine().scheduler().synchronize();
	return to_main;
}

WRITE8_MEMBER(nmk004_device::nmk004_port4_w)
{
	// bit 0x08 toggles frequently but is connected to nothing?

	// bit 0x01 is set to reset the 68k
	m_reset_cb(BIT(data, 0) ? ASSERT_LINE : CLEAR_LINE);
}

WRITE8_MEMBER(nmk004_device::nmk004_oki0_bankswitch_w)
{
	data &= 3;
	membank(":okibank1")->set_entry(data);
}

WRITE8_MEMBER(nmk004_device::nmk004_oki1_bankswitch_w)
{
	data &= 3;
	membank(":okibank2")->set_entry(data);
}

READ8_MEMBER(nmk004_device::nmk004_tonmk004_r)
{
	machine().scheduler().synchronize();
	return to_nmk004;
}

WRITE8_MEMBER(nmk004_device::nmk004_tomain_w)
{
	machine().scheduler().synchronize();
	to_main = data;
}


void nmk004_device::ym2203_irq_handler(int irq)
{
	m_cpu->set_input_line(0, irq ? ASSERT_LINE : CLEAR_LINE);
}

static ADDRESS_MAP_START( nmk004_sound_mem_map, AS_PROGRAM, 8, nmk004_device )
	//AM_RANGE(0x0000, 0x1fff) AM_ROM /* 0x0000 - 0x1fff = internal ROM */
	AM_RANGE(0x2000, 0xefff) AM_ROM AM_REGION(":audiocpu", 0x2000 )
	AM_RANGE(0xf000, 0xf7ff) AM_RAM
	AM_RANGE(0xf800, 0xf801) AM_DEVREADWRITE(":ymsnd", ym2203_device, read, write)
	AM_RANGE(0xf900, 0xf900) AM_DEVREADWRITE(":oki1", okim6295_device, read, write)
	AM_RANGE(0xfa00, 0xfa00) AM_DEVREADWRITE(":oki2", okim6295_device, read, write)
	AM_RANGE(0xfb00, 0xfb00) AM_READ(nmk004_tonmk004_r)    // from main cpu
	AM_RANGE(0xfc00, 0xfc00) AM_WRITE(nmk004_tomain_w)  // to main cpu
	AM_RANGE(0xfc01, 0xfc01) AM_WRITE(nmk004_oki0_bankswitch_w)
	AM_RANGE(0xfc02, 0xfc02) AM_WRITE(nmk004_oki1_bankswitch_w)
ADDRESS_MAP_END


ROM_START( nmk004 )
	ROM_REGION( 0x2000, "mcu", 0 )
	ROM_LOAD( "nmk004.bin", 0x00000, 0x02000, CRC(83b6f611) SHA1(bb7ddc00affe8a066002ecd6858dbd2854af8940) )
ROM_END


DEFINE_DEVICE_TYPE(NMK004, nmk004_device, "nmk004", "NMK004")

nmk004_device::nmk004_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NMK004, tag, owner, clock),
	m_cpu(*this, "mcu"),
	m_reset_cb(*this),
	to_nmk004(0xff),
	to_main(0xff)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void nmk004_device::device_start()
{
	m_reset_cb.resolve_safe();

	membank(":okibank1")->configure_entries(0, 4, memregion(":oki1")->base() + 0x20000, 0x20000);
	membank(":okibank2")->configure_entries(0, 4, memregion(":oki2")->base() + 0x20000, 0x20000);
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------
MACHINE_CONFIG_MEMBER( nmk004_device::device_add_mconfig )
	MCFG_CPU_ADD("mcu",TMP90840, DERIVED_CLOCK(1,1)) // Toshiba TMP90C840AF in QFP64 package with 8Kbyte internal ROM
	MCFG_CPU_PROGRAM_MAP(nmk004_sound_mem_map)
	MCFG_TLCS90_PORT_P4_WRITE_CB(WRITE8(nmk004_device, nmk004_port4_w))
MACHINE_CONFIG_END

//-------------------------------------------------
//  device_rom_region - return a pointer to the
//  the device's ROM definitions
//-------------------------------------------------
const tiny_rom_entry *nmk004_device::device_rom_region() const
{
	return ROM_NAME(nmk004 );
}
