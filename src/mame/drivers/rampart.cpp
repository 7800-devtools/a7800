// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Atari Rampart hardware

    driver by Aaron Giles

    Games supported:
        * Rampart (1990) [3 sets]

    Known bugs:
        * P3 trackball doesn't work, maybe it needs some kind of fake input port

    Note:
        P3 buttons 1 and 2 are mapped twice. THIS IS NOT A BUG!

    bp 548,a0==6c0007 && (d0&ffff)!=0,{print d0&ffff; g}

****************************************************************************

    Memory map (TBA)

***************************************************************************/


#include "emu.h"
#include "includes/rampart.h"

#include "cpu/m68000/m68000.h"
#include "machine/watchdog.h"
#include "sound/okim6295.h"
#include "sound/ym2413.h"
#include "speaker.h"


#define MASTER_CLOCK        XTAL_14_31818MHz


/*************************************
 *
 *  Interrupt handling
 *
 *************************************/

void rampart_state::update_interrupts()
{
	m_maincpu->set_input_line(4, m_scanline_int_state ? ASSERT_LINE : CLEAR_LINE);
}


void rampart_state::scanline_update(screen_device &screen, int scanline)
{
	/* generate 32V signals */
	if ((scanline & 32) == 0)
		scanline_int_gen(*m_maincpu);
}



/*************************************
 *
 *  Initialization
 *
 *************************************/

MACHINE_RESET_MEMBER(rampart_state,rampart)
{
	atarigen_state::machine_reset();
	scanline_timer_reset(*m_screen, 32);
}



/*************************************
 *
 *  Latch write
 *
 *************************************/

WRITE16_MEMBER(rampart_state::latch_w)
{
	/* bit layout in this register:

	    0x8000 == VCR ???
	    0x2000 == LETAMODE1 (controls right trackball)
	    0x1000 == CBANK (color bank -- is it ever set to non-zero?)
	    0x0800 == LETAMODE0 (controls center and left trackballs)
	    0x0400 == LETARES (reset LETA analog control reader)
	    0x0200 == COINCTRL
	    0x0100 == COINCTRR

	    0x0020 == PMIX0 (ADPCM mixer level)
	    0x0010 == /PCMRES (ADPCM reset)
	    0x000E == YMIX2-0 (YM2413 mixer level)
	    0x0001 == /YAMRES (YM2413 reset)
	*/

	/* upper byte being modified? */
	if (ACCESSING_BITS_8_15)
	{
		if (data & 0x1000)
			logerror("Color bank set to 1!\n");
		machine().bookkeeping().coin_counter_w(0, (data >> 9) & 1);
		machine().bookkeeping().coin_counter_w(1, (data >> 8) & 1);
	}

	/* lower byte being modified? */
	if (ACCESSING_BITS_0_7)
	{
		m_oki->set_output_gain(ALL_OUTPUTS, (data & 0x0020) ? 1.0f : 0.0f);
		if (!(data & 0x0010))
			m_oki->reset();
		m_ym2413->set_output_gain(ALL_OUTPUTS, ((data >> 1) & 7) / 7.0f);
		if (!(data & 0x0001))
			m_ym2413->reset();
	}
}



/*************************************
 *
 *  Main CPU memory handlers
 *
 *************************************/

/* full memory map deduced from schematics and GALs */
static ADDRESS_MAP_START( main_map, AS_PROGRAM, 16, rampart_state )
	ADDRESS_MAP_GLOBAL_MASK(0x7fffff)
	AM_RANGE(0x000000, 0x0fffff) AM_ROM
	AM_RANGE(0x140000, 0x147fff) AM_MIRROR(0x438000) AM_ROM /* slapstic goes here */
	AM_RANGE(0x200000, 0x21ffff) AM_RAM AM_SHARE("bitmap")
	AM_RANGE(0x220000, 0x3bffff) AM_WRITENOP    /* the code blasts right through this when initializing */
	AM_RANGE(0x3c0000, 0x3c07ff) AM_MIRROR(0x019800) AM_DEVREADWRITE8("palette", palette_device, read, write, 0xff00) AM_SHARE("palette")
	AM_RANGE(0x3e0000, 0x3e07ff) AM_MIRROR(0x010000) AM_RAM AM_SHARE("mob")
	AM_RANGE(0x3e0800, 0x3e3f3f) AM_MIRROR(0x010000) AM_RAM
	AM_RANGE(0x3e3f40, 0x3e3f7f) AM_MIRROR(0x010000) AM_RAM AM_SHARE("mob:slip")
	AM_RANGE(0x3e3f80, 0x3effff) AM_MIRROR(0x010000) AM_RAM
	AM_RANGE(0x460000, 0x460001) AM_MIRROR(0x019ffe) AM_DEVREADWRITE8("oki", okim6295_device, read, write, 0xff00)
	AM_RANGE(0x480000, 0x480003) AM_MIRROR(0x019ffc) AM_DEVWRITE8("ymsnd", ym2413_device, write, 0xff00)
	AM_RANGE(0x500000, 0x500fff) AM_MIRROR(0x019000) AM_DEVREADWRITE8("eeprom", atari_eeprom_device, read, write, 0x00ff)
	AM_RANGE(0x5a6000, 0x5a6001) AM_MIRROR(0x019ffe) AM_DEVWRITE("eeprom", atari_eeprom_device, unlock_write)
	AM_RANGE(0x640000, 0x640001) AM_MIRROR(0x019ffe) AM_WRITE(latch_w)
	AM_RANGE(0x640000, 0x640001) AM_MIRROR(0x019ffc) AM_READ_PORT("IN0")
	AM_RANGE(0x640002, 0x640003) AM_MIRROR(0x019ffc) AM_READ_PORT("IN1")
	AM_RANGE(0x6c0000, 0x6c0001) AM_MIRROR(0x019ff8) AM_READ_PORT("TRACK0")
	AM_RANGE(0x6c0002, 0x6c0003) AM_MIRROR(0x019ff8) AM_READ_PORT("TRACK1")
	AM_RANGE(0x6c0004, 0x6c0005) AM_MIRROR(0x019ff8) AM_READ_PORT("TRACK2")
	AM_RANGE(0x6c0006, 0x6c0007) AM_MIRROR(0x019ff8) AM_READ_PORT("TRACK3")
	AM_RANGE(0x726000, 0x726001) AM_MIRROR(0x019ffe) AM_DEVWRITE("watchdog", watchdog_timer_device, reset16_w)
	AM_RANGE(0x7e6000, 0x7e6001) AM_MIRROR(0x019ffe) AM_WRITE(scanline_int_ack_w)
ADDRESS_MAP_END



/*************************************
 *
 *  Port definitions
 *
 *************************************/

static INPUT_PORTS_START( rampart )
	PORT_START("IN0")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(3) // alternate button1
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(3)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x00f0, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(3)
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x00f8, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(3) // alternate button2
	PORT_SERVICE( 0x0800, IP_ACTIVE_LOW )
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK0")
	PORT_BIT( 0x00ff, 0, IPT_TRACKBALL_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(30) PORT_REVERSE PORT_PLAYER(2)
	PORT_BIT( 0xff00, 0, IPT_TRACKBALL_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(30) PORT_REVERSE PORT_PLAYER(3)

	PORT_START("TRACK1")
	PORT_BIT( 0x00ff, 0, IPT_TRACKBALL_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(30) PORT_REVERSE PORT_PLAYER(2)
	PORT_BIT( 0xff00, 0, IPT_TRACKBALL_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(30) PORT_REVERSE PORT_PLAYER(3)

	PORT_START("TRACK2")
	PORT_BIT( 0x00ff, 0, IPT_TRACKBALL_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(30) PORT_REVERSE PORT_PLAYER(1)
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK3")
	PORT_BIT( 0x00ff, 0, IPT_TRACKBALL_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(30) PORT_REVERSE PORT_PLAYER(1)
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


static INPUT_PORTS_START( ramprt2p )
	PORT_START("IN0")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(3) // alternate button1
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(3)
	PORT_DIPNAME( 0x0004, 0x0004, DEF_STR( Players ) )
	PORT_DIPSETTING(    0x0000, "2")
	PORT_DIPSETTING(    0x0004, "3")
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x00f0, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(3)
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x00f8, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(3) // alternate button2
	PORT_SERVICE( 0x0800, IP_ACTIVE_LOW )
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK0")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_PLAYER(1)
	PORT_BIT( 0x0080, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_PLAYER(1)
	PORT_BIT( 0x0100, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_PLAYER(3)
	PORT_BIT( 0x0200, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(3)
	PORT_BIT( 0x0400, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_PLAYER(3)
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_PLAYER(3)
	PORT_BIT( 0xf000, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("TRACK1")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK3")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


static INPUT_PORTS_START( rampartj )
	PORT_START("IN0")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x00f0, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x00f8, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_SERVICE( 0x0800, IP_ACTIVE_LOW )
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK0")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_PLAYER(1)
	PORT_BIT( 0x0080, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_PLAYER(1)
	PORT_BIT( 0x0100, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x0200, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x0400, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0xf000, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("TRACK1")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("TRACK3")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END



/*************************************
 *
 *  Graphics definitions
 *
 *************************************/

static const gfx_layout molayout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ 0, 1, 2, 3 },
	{ 0, 4, 8, 12, 16, 20, 24, 28 },
	{ 0*8, 4*8, 8*8, 12*8, 16*8, 20*8, 24*8, 28*8 },
	32*8
};


static GFXDECODE_START( rampart )
	GFXDECODE_ENTRY( "gfx1", 0, molayout,  256, 16 )
GFXDECODE_END



/*************************************
 *
 *  Machine driver
 *
 *************************************/

static MACHINE_CONFIG_START( rampart )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, MASTER_CLOCK/2)
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", atarigen_state, video_int_gen)

	MCFG_SLAPSTIC_ADD("slapstic", 118)

	MCFG_MACHINE_RESET_OVERRIDE(rampart_state,rampart)

	MCFG_ATARI_EEPROM_2816_ADD("eeprom")

	MCFG_WATCHDOG_ADD("watchdog")
	MCFG_WATCHDOG_VBLANK_INIT("screen", 8)

	/* video hardware */
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", rampart)
	MCFG_PALETTE_ADD("palette", 512)
	MCFG_PALETTE_FORMAT(IRRRRRGGGGGBBBBB)
	MCFG_PALETTE_MEMBITS(8)

	MCFG_ATARI_MOTION_OBJECTS_ADD("mob", "screen", rampart_state::s_mob_config)
	MCFG_ATARI_MOTION_OBJECTS_GFXDECODE("gfxdecode")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_VIDEO_ATTRIBUTES(VIDEO_UPDATE_BEFORE_VBLANK)
	/* note: these parameters are from published specs, not derived */
	/* the board uses an SOS-2 chip to generate video signals */
	MCFG_SCREEN_RAW_PARAMS(MASTER_CLOCK/2, 456, 0+12, 336+12, 262, 0, 240)
	MCFG_SCREEN_UPDATE_DRIVER(rampart_state, screen_update_rampart)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_VIDEO_START_OVERRIDE(rampart_state,rampart)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_OKIM6295_ADD("oki", MASTER_CLOCK/4/3, PIN7_LOW)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.60)

	MCFG_SOUND_ADD("ymsnd", YM2413, MASTER_CLOCK/4)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END



/*************************************
 *
 *  ROM definition(s)
 *
 *************************************/

ROM_START( rampart )
	ROM_REGION( 0x148000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "136082-1033.13l", 0x00000, 0x80000, CRC(5c36795f) SHA1(2f3dcdfd6b04d851aa1082848624687ac0cec9e2) )
	ROM_LOAD16_BYTE( "136082-1032.13j", 0x00001, 0x80000, CRC(ec7bc38c) SHA1(72d4dbb11e92c69cb560bbb39d7bbd5e845b1e4d) )
	ROM_LOAD16_BYTE( "136082-2031.13l", 0x00000, 0x10000, CRC(07650c7e) SHA1(0a8eec76aefd4fd1515c1a0d5b96f71c674cdce7) )
	ROM_LOAD16_BYTE( "136082-2030.13h", 0x00001, 0x10000, CRC(e2bf2a26) SHA1(be15b3b0e302382518436441875a1b72954a589a) )

	ROM_REGION( 0x20000, "gfx1", ROMREGION_INVERT )
	ROM_LOAD( "136082-1009.2n",   0x000000, 0x20000, CRC(23b95f59) SHA1(cec8523eaf83d4c9bb0055f34024a6e9c52c4c0c) )

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM data */
	ROM_LOAD( "136082-1007.2d", 0x00000, 0x20000, CRC(c96a0fc3) SHA1(6e7e242d0afa4714ca31d77ccbf8ee487bbdb1e4) )
	ROM_LOAD( "136082-1008.1d", 0x20000, 0x20000, CRC(518218d9) SHA1(edf1b11579dcfa9a872fa4bd866dc2f95fac767d) )

	ROM_REGION( 0x800, "eeprom:eeprom", 0 )
	ROM_LOAD( "rampart-eeprom.bin", 0x0000, 0x800, CRC(0be57615) SHA1(bd1f9eef410c78c091d2c925d6275427c77c7ecd) )

	ROM_REGION( 0x0c00, "plds", 0 )
	ROM_LOAD( "gal16v8-136082-1000.1j",  0x0000, 0x0117, CRC(18f82b38) SHA1(2ffd43a143396617704ced51da78fec2cf12cced) )
	ROM_LOAD( "gal16v8-136082-1001.4l",  0x0200, 0x0117, CRC(74d75d68) SHA1(dc3ee765ec48a76af6433026243284437958a39a) )
	ROM_LOAD( "gal16v8-136082-1002.7k",  0x0400, 0x0117, CRC(f593401f) SHA1(fbc258cd389f397a005a522812d412f4ed9bf407) )
	ROM_LOAD( "gal20v8-136082-1003.8j",  0x0600, 0x0157, CRC(67bb9705) SHA1(65bb31421f1303fce546781a463cc76921e58b25) )
	ROM_LOAD( "gal20v8-136082-1004.8m",  0x0800, 0x0157, CRC(0001ed7d) SHA1(c16a695361ee17d7508f6fb46854a9189549e3a3) )
	ROM_LOAD( "gal16v8-136082-1006.12c", 0x0a00, 0x0117, CRC(1f3b735d) SHA1(d3243cb3565e32e25637987de6044fe6e453d2f0) )
ROM_END


ROM_START( rampart2p )
	ROM_REGION( 0x148000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "136082-1033.13l",  0x00000, 0x80000, CRC(5c36795f) SHA1(2f3dcdfd6b04d851aa1082848624687ac0cec9e2) )
	ROM_LOAD16_BYTE( "136082-1032.13j",  0x00001, 0x80000, CRC(ec7bc38c) SHA1(72d4dbb11e92c69cb560bbb39d7bbd5e845b1e4d) )
	ROM_LOAD16_BYTE( "136082-2051.13kl", 0x00000, 0x20000, CRC(d4e26d0f) SHA1(5106549e6d003711bfd390aa2e19e6e5f33f2cf9) )
	ROM_LOAD16_BYTE( "136082-2050.13h",  0x00001, 0x20000, CRC(ed2a49bd) SHA1(b97ee41b7f930ba7b8b113c1b19c7729a5880b1f) )

	ROM_REGION( 0x20000, "gfx1", ROMREGION_INVERT )
	ROM_LOAD( "136082-1019.2n",   0x000000, 0x20000, CRC(efa38bef) SHA1(d38448138134e7a0be2a75c3cd6ab0729da5b83b) )

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM data */
	ROM_LOAD( "136082-1007.2d", 0x00000, 0x20000, CRC(c96a0fc3) SHA1(6e7e242d0afa4714ca31d77ccbf8ee487bbdb1e4) )
	ROM_LOAD( "136082-1008.1d", 0x20000, 0x20000, CRC(518218d9) SHA1(edf1b11579dcfa9a872fa4bd866dc2f95fac767d) )

	ROM_REGION( 0x800, "eeprom:eeprom", 0 )
	ROM_LOAD( "rampart-eeprom.bin", 0x0000, 0x800, CRC(0be57615) SHA1(bd1f9eef410c78c091d2c925d6275427c77c7ecd) )

	ROM_REGION( 0x0c00, "plds", 0 )
	ROM_LOAD( "gal16v8-136082-1000.1j",  0x0000, 0x0117, CRC(18f82b38) SHA1(2ffd43a143396617704ced51da78fec2cf12cced) )
	ROM_LOAD( "gal16v8-136082-1001.4l",  0x0200, 0x0117, CRC(74d75d68) SHA1(dc3ee765ec48a76af6433026243284437958a39a) )
	ROM_LOAD( "gal16v8-136082-1002.7k",  0x0400, 0x0117, CRC(f593401f) SHA1(fbc258cd389f397a005a522812d412f4ed9bf407) )
	ROM_LOAD( "gal20v8-136082-1003.8j",  0x0600, 0x0157, CRC(67bb9705) SHA1(65bb31421f1303fce546781a463cc76921e58b25) )
	ROM_LOAD( "gal20v8-136082-1004.8m",  0x0800, 0x0157, CRC(0001ed7d) SHA1(c16a695361ee17d7508f6fb46854a9189549e3a3) )
	ROM_LOAD( "gal16v8-136082-1056.12c", 0x0a00, 0x0117, CRC(bd70bf25) SHA1(e89ed789fae0c5776a10bbebc7dda1d85fc79374) )
ROM_END


ROM_START( rampartj )
	ROM_REGION( 0x148000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "136082-3451.bin",  0x00000, 0x20000, CRC(c6596d32) SHA1(3e3e0cbb3b5fc6dd9685bbc4b18c22e0858d9282) )
	ROM_LOAD16_BYTE( "136082-3450.bin",  0x00001, 0x20000, CRC(563b33cc) SHA1(8b454bc19644f1d3d76e4a13f08071cf5eab36e2) )
	ROM_LOAD16_BYTE( "136082-1463.bin",  0x40000, 0x20000, CRC(65fe3491) SHA1(3aa3b98fb7fe808ef89e100b5e1ee1c99c4312b6) )
	ROM_LOAD16_BYTE( "136082-1462.bin",  0x40001, 0x20000, CRC(ba731652) SHA1(298adda4fd67991b5153e5316f50da79320754ee) )
	ROM_LOAD16_BYTE( "136082-1465.bin",  0x80000, 0x20000, CRC(9cb87d1b) SHA1(95f24ec2c42b39878b3680c4948bfb0d712cd60e) )
	ROM_LOAD16_BYTE( "136082-1464.bin",  0x80001, 0x20000, CRC(2ff75c40) SHA1(9c444402d237c3933219ab4872f180abc392547f) )
	ROM_LOAD16_BYTE( "136082-1467.bin",  0xc0000, 0x20000, CRC(e0cfcda5) SHA1(0a1bf083e0589260caf6dfcb4e556b8f5e1ece25) )
	ROM_LOAD16_BYTE( "136082-1466.bin",  0xc0001, 0x20000, CRC(a7a5a951) SHA1(a9a6adfa315c41cde4cca07d7e7d7f79ecba9f7a) )

	ROM_REGION( 0x20000, "gfx1", ROMREGION_INVERT )
	ROM_LOAD( "136082-2419.bin",   0x000000, 0x20000, CRC(456a8aae) SHA1(f35a3dc2069e20493661cf35fc0d4f4c4e11e420) )

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM data */
	ROM_LOAD( "136082-1007.2d", 0x00000, 0x20000, CRC(c96a0fc3) SHA1(6e7e242d0afa4714ca31d77ccbf8ee487bbdb1e4) )
	ROM_LOAD( "136082-1008.1d", 0x20000, 0x20000, CRC(518218d9) SHA1(edf1b11579dcfa9a872fa4bd866dc2f95fac767d) )

	ROM_REGION( 0x800, "eeprom:eeprom", 0 )
	ROM_LOAD( "rampartj-eeprom.bin", 0x0000, 0x800, CRC(096cacdc) SHA1(48328a27ce1975a27d9a83ae05d068cee7556a90) )

	ROM_REGION( 0x0c00, "plds", 0 )
	ROM_LOAD( "gal16v8-136082-1000.1j",  0x0000, 0x0117, CRC(18f82b38) SHA1(2ffd43a143396617704ced51da78fec2cf12cced) )
	ROM_LOAD( "gal16v8-136082-1001.4l",  0x0200, 0x0117, CRC(74d75d68) SHA1(dc3ee765ec48a76af6433026243284437958a39a) )
	ROM_LOAD( "gal16v8-136082-1002.7k",  0x0400, 0x0117, CRC(f593401f) SHA1(fbc258cd389f397a005a522812d412f4ed9bf407) )
	ROM_LOAD( "gal20v8-136082-1003.8j",  0x0600, 0x0157, CRC(67bb9705) SHA1(65bb31421f1303fce546781a463cc76921e58b25) )
	ROM_LOAD( "gal20v8-136082-1004.8m",  0x0800, 0x0157, CRC(0001ed7d) SHA1(c16a695361ee17d7508f6fb46854a9189549e3a3) )
	ROM_LOAD( "gal16v8-136082-1005.12c", 0x0a00, 0x0117, CRC(42c05114) SHA1(869a7f07da2d096b5a62f694db0dc1ca62d62242) )
ROM_END



/*************************************
 *
 *  Driver initialization
 *
 *************************************/

DRIVER_INIT_MEMBER(rampart_state,rampart)
{
	uint8_t *rom = memregion("maincpu")->base();

	memcpy(&rom[0x140000], &rom[0x40000], 0x8000);
	slapstic_configure(*m_maincpu, 0x140000, 0x438000, memregion("maincpu")->base() + 0x140000);
}



/*************************************
 *
 *  Game driver(s)
 *
 *************************************/

GAME( 1990, rampart,  0,       rampart, rampart,  rampart_state, rampart, ROT0, "Atari Games", "Rampart (Trackball)", MACHINE_SUPPORTS_SAVE )
GAME( 1990, rampart2p,rampart, rampart, ramprt2p, rampart_state, rampart, ROT0, "Atari Games", "Rampart (Joystick)", MACHINE_SUPPORTS_SAVE )
GAME( 1990, rampartj, rampart, rampart, rampartj, rampart_state, rampart, ROT0, "Atari Games", "Rampart (Japan, Joystick)", MACHINE_SUPPORTS_SAVE )
