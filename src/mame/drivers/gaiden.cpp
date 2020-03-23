// license:BSD-3-Clause
// copyright-holders:Phil Stroffolino
/***************************************************************************

Ninja Gaiden memory map (preliminary)

000000-03ffff ROM
060000-063fff RAM
070000-070fff Video RAM (text layer)
072000-075fff VRAM (backgrounds)
076000-077fff Sprite RAM
078000-079fff Palette RAM

memory mapped ports:

read:
07a001    IN0
07a002    IN2
07a003    IN1
07a004    DWSB
07a005    DSWA
see the input_ports definition below for details on the input bits

write:
07a002-07a003 sprite layer Y offset
07a104-07a105 text   layer Y scroll
07a108-07a109 text   layer Y scroll offset
07a10c-07a10d text   layer X scroll
07a204-07a205 front  layer Y scroll
07a208-07a209 front  layer Y scroll offset
07a20c-07a20d front  layer X scroll
07a304-07a305 back   layer Y scroll
07a308-07a309 back   layer Y scroll offset
07a30c-07a30d back   layer X scroll

unknown writes during boot sequence and/or game start:
07a000, 07a004, 07a006, 07a100, 07a110, 07a200, 07a210, 07a300, 07a310

Notes:
- The sprite Y size control is slightly different from gaiden/wildfang to
  raiga. In the first two, size X and Y change together, while in the latter
  they are changed independently. This is handled with a variable set in
  DRIVER_INIT, but it might also be a selectable hardware feature, since
  the two extra bits used by raiga are perfectly merged with the rest.
  Raiga also uses more sprites than the others, but there's no way to tell
  if hardware is more powerful or the extra sprites were just not needed
  in the earlier games.

- The hardware supports blending sprites and background.
  There are 3 copies of the palette;
  - one for pixels that aren't blended
  - one for pixels that are behind the object that will be blended
  - one for the object that will be blended
  Blending is performed by switching to the appropriate palettes for the
  pixels behind the blended object and the object itself, and then adding
  the RGB values.

todo:

- make sure all of the protection accesses in raiga are handled correctly.
- work out how lower priority sprites are affected by blended sprites.

***************************************************************************/
/***************************************************************************

Strato Fighter (US version)
Tecmo, 1991


PCB Layout
----------

Top Board
---------
0210-A
MG-Y.VO
-----------------------------------------------
|         MN50005XTA         4MHz  DSW2 DSW1  |
|                    6264 6264       8049     |
|           IOP8     1.3S 2.4S                |
|24MHz                                        |
|                                             |
|18.432MHz                                   J|
|                                             |
|              68000P10                      A|
|                                             |
|          6116                              M|
|          6116                               |
|          6116                              M|
|                                             |
|                                            A|
|                                             |
|              6264    YM2203  YM3014         |
|       Z80    3.4B                           |
| 4MHz  6295   4.4A    YM2203  YM3014         |
-----------------------------------------------

Bottom Board
------------
0210-B
MG-Y.VO
-----------------------------------------------
|                TECMO-5                      |
|               -----------                   |
|               | TECMO-06|                   |
| ROM.M1 ROM.M3 | YM6048  |     6264          |
|               -----------     6264          |
|   4164  4164  4164  4164                    |
|   4164  4164  4164  4164                    |
|   4164  4164  4164  4164                    |
|                                             |
|                                             |
|        TECMO-3      TECMO-3      TECMO-3    |
| TECMO-4      TECMO-4      TECMO-4           |
|                                             |
|                                             |
|                                             |
|  6264  6264   6116   6116   6264            |
| ROM.1B        ROM.4B        6116            |
|                             ROM.7A          |
-----------------------------------------------

Notes:
    68k clock:      9.216 MHz (18.432 / 2)
    Z80 clock:      4.000 MHz
    YM2203 clock:       4.000 MHz
    MSM6295 clock:  1.000 MHz (samplerate 7575Hz, i.e. / 132)

    IOP8 manufactured by Ricoh. Full part number: RICOH EPLIOP8BP (PAL or PIC?)

***************************************************************************/

#include "emu.h"
#include "includes/gaiden.h"

#include "cpu/m68000/m68000.h"
#include "cpu/z80/z80.h"
#include "machine/watchdog.h"
#include "sound/2203intf.h"
#include "sound/okim6295.h"
#include "sound/ym2151.h"
#include "speaker.h"


WRITE16_MEMBER(gaiden_state::gaiden_sound_command_w)
{
	if (ACCESSING_BITS_0_7)
		m_soundlatch->write(space, 0, data & 0xff);   /* Ninja Gaiden */
	if (ACCESSING_BITS_8_15)
		m_soundlatch->write(space, 0, data >> 8); /* Tecmo Knight */
	m_audiocpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

WRITE16_MEMBER(gaiden_state::drgnbowl_sound_command_w)
{
	if (ACCESSING_BITS_8_15)
	{
		m_soundlatch->write(space, 0, data >> 8);
		m_audiocpu->set_input_line(0, HOLD_LINE);
	}
}



/* Wild Fang / Tecmo Knight has a simple protection. It writes codes to 0x07a804, */
/* and reads the answer from 0x07a007. The returned values contain the address of */
/* a function to jump to. */

WRITE16_MEMBER(gaiden_state::wildfang_protection_w)
{
	if (ACCESSING_BITS_8_15)
	{
		static const int jumppoints[] =
		{
			0x0c0c,0x0cac,0x0d42,0x0da2,0x0eea,0x112e,0x1300,0x13fa,
			0x159a,0x1630,0x109a,0x1700,0x1750,0x1806,0x18d6,0x1a44,
			0x1b52
		};

		data >>= 8;

//      logerror("PC %06x: prot = %02x\n",space.device().safe_pc(),data);

		switch (data & 0xf0)
		{
			case 0x00:  /* init */
				m_prot = 0x00;
				break;
			case 0x10:  /* high 4 bits of jump code */
				m_jumpcode = (data & 0x0f) << 4;
				m_prot = 0x10;
				break;
			case 0x20:  /* low 4 bits of jump code */
				m_jumpcode |= data & 0x0f;
				if (m_jumpcode >= ARRAY_LENGTH(jumppoints))
				{
					logerror("unknown jumpcode %02x\n", m_jumpcode);
					m_jumpcode = 0;
				}
				m_prot = 0x20;
				break;
			case 0x30:  /* ask for bits 12-15 of function address */
				m_prot = 0x40 | ((jumppoints[m_jumpcode] >> 12) & 0x0f);
				break;
			case 0x40:  /* ask for bits 8-11 of function address */
				m_prot = 0x50 | ((jumppoints[m_jumpcode] >> 8) & 0x0f);
				break;
			case 0x50:  /* ask for bits 4-7 of function address */
				m_prot = 0x60 | ((jumppoints[m_jumpcode] >> 4) & 0x0f);
				break;
			case 0x60:  /* ask for bits 0-3 of function address */
				m_prot = 0x70 | ((jumppoints[m_jumpcode] >> 0) & 0x0f);
				break;
		}
	}
}

READ16_MEMBER(gaiden_state::wildfang_protection_r)
{
//  logerror("PC %06x: read prot %02x\n", space.device().safe_pc(), m_prot);
	return m_prot;
}



/*

Raiga Protection
MCU read routine is at D9CE

startup codes

it reads 00/36/0e/11/33/34/2d, fetching some code copied to RAM,
and a value which is used as an offset to point to the code in RAM.
19/12/31/28 are read repeatedly, from the interrupt, and the returned
value changes, some kind of mode set command?

level codes

00 | score screen after level 1 (5457)
01 | score screen after level 2 (494e)
02 | score screen after level 3 (5f4b)
03 | score screen after level 4 (4149)
04 | score screen after level 5 (5345)
05 | score screen after level 6 + first loop end sequence (525f)
06 | score screen after second loop level 1 (4d49)
07 | score screen after second loop level 2 (5941)
08 | score screen after second loop level 3 (5241)
09 | score screen after second loop level 4 (5349)
0a | score screen after second loop level 5 (4d4f)
0b | score screen after second loop level 6 + final end sequence (4a49)


other game codes

13 | Game over (594f)
15 | at the start of second level attract mode (4e75)
1c | start of level 4 attract mode (4e75)
1e | after continue .. (5349)
23 | after entering hi-score (4e75)
25 | after attract level 1 (4849)
2b | after japan / wdud screen, to get back to 'tecmo presents' (524f)

also bonus life + when the boss appears but I think they use the
same commands as some of the above

*/

/* these are used during startup */
static const int jumppoints_00[0x100] =
{
	0x6669,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
		-1,    -1,    -1,    -1,    -1,    -1,0x4a46,    -1,
		-1,0x6704,    -2,    -1,    -1,    -1,    -1,    -1,
		-1,    -2,    -1,    -1,    -1,    -1,    -1,    -1,
		-1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
		-2,    -1,    -1,    -1,    -1,0x4e75,    -1,    -1,
		-1,    -2,    -1,0x4e71,0x60fc,    -1,0x7288,    -1,
		-1,    -1,    -1,    -1,    -1,    -1,    -1,    -1
};

/* these are used the rest of the time */
static const int jumppoints_other[0x100] =
{
	0x5457,0x494e,0x5f4b,0x4149,0x5345,0x525f,0x4d49,0x5941,
	0x5241,0x5349,0x4d4f,0x4a49,    -1,    -1,    -1,    -1,
		-1,    -1,    -2,0x594f,    -1,0x4e75,    -1,    -1,
		-1,    -2,    -1,    -1,0x4e75,    -1,0x5349,    -1,
		-1,    -1,    -1,0x4e75,    -1,0x4849,    -1,    -1,
		-2,    -1,    -1,0x524f,    -1,    -1,    -1,    -1,
		-1,    -2,    -1,    -1,    -1,    -1,    -1,    -1,
		-1,    -1,    -1,    -1,    -1,    -1,    -1,    -1
};

MACHINE_RESET_MEMBER(gaiden_state,raiga)
{
	m_prot = 0;
	m_jumpcode = 0;

	m_tx_scroll_x = 0;
	m_tx_scroll_y = 0;
	m_bg_scroll_x = 0;
	m_bg_scroll_y = 0;
	m_fg_scroll_x = 0;
	m_fg_scroll_y = 0;

	m_tx_offset_y = 0;
	m_fg_offset_y = 0;
	m_bg_offset_y = 0;
	m_spr_offset_y = 0;
}

MACHINE_START_MEMBER(gaiden_state,raiga)
{
	save_item(NAME(m_prot));
	save_item(NAME(m_jumpcode));

	save_item(NAME(m_tx_scroll_x));
	save_item(NAME(m_tx_scroll_y));
	save_item(NAME(m_bg_scroll_x));
	save_item(NAME(m_bg_scroll_y));
	save_item(NAME(m_fg_scroll_x));
	save_item(NAME(m_fg_scroll_y));

	save_item(NAME(m_tx_offset_y));
	save_item(NAME(m_fg_offset_y));
	save_item(NAME(m_bg_offset_y));
	save_item(NAME(m_spr_offset_y));
}

WRITE16_MEMBER(gaiden_state::raiga_protection_w)
{
	if (ACCESSING_BITS_8_15)
	{
		data >>= 8;

//      logerror("PC %06x: prot = %02x\n", space.device().safe_pc(), data);

		switch (data & 0xf0)
		{
			case 0x00:  /* init */
				m_prot = 0x00;
				break;
			case 0x10:  /* high 4 bits of jump code */
				m_jumpcode = (data & 0x0f) << 4;
				m_prot = 0x10;
				break;
			case 0x20:  /* low 4 bits of jump code */
				m_jumpcode |= data & 0x0f;
				logerror("requested protection jumpcode %02x\n", m_jumpcode);
//              m_jumpcode = 0;
				if (m_raiga_jumppoints[m_jumpcode] == -2)
				{
					m_raiga_jumppoints = jumppoints_other;
				}

				if (m_raiga_jumppoints[m_jumpcode] == -1)
				{
					logerror("unknown jumpcode %02x\n", m_jumpcode);
					popmessage("unknown jumpcode %02x", m_jumpcode);
					m_jumpcode = 0;
				}
				m_prot = 0x20;
				break;
			case 0x30:  /* ask for bits 12-15 of function address */
				m_prot = 0x40 | ((m_raiga_jumppoints[m_jumpcode] >> 12) & 0x0f);
				break;
			case 0x40:  /* ask for bits 8-11 of function address */
				m_prot = 0x50 | ((m_raiga_jumppoints[m_jumpcode] >> 8) & 0x0f);
				break;
			case 0x50:  /* ask for bits 4-7 of function address */
				m_prot = 0x60 | ((m_raiga_jumppoints[m_jumpcode] >> 4) & 0x0f);
				break;
			case 0x60:  /* ask for bits 0-3 of function address */
				m_prot = 0x70 | ((m_raiga_jumppoints[m_jumpcode] >> 0) & 0x0f);
				break;
		}
	}
}

READ16_MEMBER(gaiden_state::raiga_protection_r)
{
//  logerror("PC %06x: read prot %02x\n", space.device().safe_pc(), m_prot);
	return m_prot;
}

static ADDRESS_MAP_START( gaiden_map, AS_PROGRAM, 16, gaiden_state )
	AM_RANGE(0x000000, 0x03ffff) AM_ROM
	AM_RANGE(0x060000, 0x063fff) AM_RAM
	AM_RANGE(0x070000, 0x070fff) AM_RAM_WRITE(gaiden_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x072000, 0x073fff) AM_RAM_WRITE(gaiden_videoram2_w) AM_SHARE("videoram2")
	AM_RANGE(0x074000, 0x075fff) AM_RAM_WRITE(gaiden_videoram3_w) AM_SHARE("videoram3")
	AM_RANGE(0x076000, 0x077fff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x078000, 0x079fff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x07a000, 0x07a001) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x07a002, 0x07a003) AM_READ_PORT("P1_P2") AM_WRITE(gaiden_sproffsety_w)
	AM_RANGE(0x07a004, 0x07a005) AM_READ_PORT("DSW")
	AM_RANGE(0x07a104, 0x07a105) AM_WRITE(gaiden_txscrolly_w)
	AM_RANGE(0x07a108, 0x07a109) AM_WRITE(gaiden_txoffsety_w)
	AM_RANGE(0x07a10c, 0x07a10d) AM_WRITE(gaiden_txscrollx_w)
	AM_RANGE(0x07a204, 0x07a205) AM_WRITE(gaiden_fgscrolly_w)
	AM_RANGE(0x07a208, 0x07a209) AM_WRITE(gaiden_fgoffsety_w)
	AM_RANGE(0x07a20c, 0x07a20d) AM_WRITE(gaiden_fgscrollx_w)
	AM_RANGE(0x07a304, 0x07a305) AM_WRITE(gaiden_bgscrolly_w)
	AM_RANGE(0x07a308, 0x07a309) AM_WRITE(gaiden_bgoffsety_w)
	AM_RANGE(0x07a30c, 0x07a30d) AM_WRITE(gaiden_bgscrollx_w)
	AM_RANGE(0x07a800, 0x07a801) AM_DEVWRITE("watchdog", watchdog_timer_device, reset16_w)
	AM_RANGE(0x07a802, 0x07a803) AM_WRITE(gaiden_sound_command_w)
	AM_RANGE(0x07a806, 0x07a807) AM_WRITENOP
	AM_RANGE(0x07a808, 0x07a809) AM_WRITE(gaiden_flip_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( drgnbowl_map, AS_PROGRAM, 16, gaiden_state )
	AM_RANGE(0x000000, 0x03ffff) AM_ROM
	AM_RANGE(0x060000, 0x063fff) AM_RAM
	AM_RANGE(0x070000, 0x070fff) AM_RAM_WRITE(gaiden_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x072000, 0x073fff) AM_RAM_WRITE(gaiden_videoram2_w) AM_SHARE("videoram2")
	AM_RANGE(0x074000, 0x075fff) AM_RAM_WRITE(gaiden_videoram3_w) AM_SHARE("videoram3")
	AM_RANGE(0x076000, 0x077fff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x078000, 0x079fff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x07a000, 0x07a001) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x07a002, 0x07a003) AM_READ_PORT("P1_P2")
	AM_RANGE(0x07a004, 0x07a005) AM_READ_PORT("DSW")
	AM_RANGE(0x07a00e, 0x07a00f) AM_WRITE(drgnbowl_sound_command_w)
	AM_RANGE(0x07e000, 0x07e001) AM_WRITENOP
	AM_RANGE(0x07f000, 0x07f001) AM_WRITE(gaiden_bgscrolly_w)
	AM_RANGE(0x07f002, 0x07f003) AM_WRITE(gaiden_bgscrollx_w)
	AM_RANGE(0x07f004, 0x07f005) AM_WRITE(gaiden_fgscrolly_w)
	AM_RANGE(0x07f006, 0x07f007) AM_WRITE(gaiden_fgscrollx_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( sound_map, AS_PROGRAM, 8, gaiden_state )
	AM_RANGE(0x0000, 0xdfff) AM_ROM
	AM_RANGE(0xe000, 0xefff) AM_ROM /* raiga only */
	AM_RANGE(0xf000, 0xf7ff) AM_RAM
	AM_RANGE(0xf800, 0xf800) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0xf810, 0xf811) AM_DEVWRITE("ym1", ym2203_device, write)
	AM_RANGE(0xf820, 0xf821) AM_DEVWRITE("ym2", ym2203_device, write)
	AM_RANGE(0xfc00, 0xfc00) AM_NOP /* ?? */
	AM_RANGE(0xfc20, 0xfc20) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
ADDRESS_MAP_END

static ADDRESS_MAP_START( drgnbowl_sound_map, AS_PROGRAM, 8, gaiden_state )
	AM_RANGE(0x0000, 0xf7ff) AM_ROM
	AM_RANGE(0xf800, 0xffff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( drgnbowl_sound_port_map, AS_IO, 8, gaiden_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x01) AM_DEVREADWRITE("ymsnd", ym2151_device, read, write)
	AM_RANGE(0x80, 0x80) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0xc0, 0xc0) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
ADDRESS_MAP_END

static INPUT_PORTS_START( common )
	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN2 )

	PORT_START("P1_P2")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW")
	PORT_DIPNAME( 0x00e0, 0x00e0, DEF_STR( Coin_A ) ) PORT_DIPLOCATION("SWA:3,2,1")
	PORT_DIPSETTING(      0x0000, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x00e0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x0060, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x00a0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x00c0, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0x001c, 0x001c, DEF_STR( Coin_B ) ) PORT_DIPLOCATION("SWA:6,5,4")
	PORT_DIPSETTING(      0x0000, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x001c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x000c, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0014, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x0018, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0x0002, 0x0002, DEF_STR( Flip_Screen ) ) PORT_DIPLOCATION("SWA:7")
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0001, 0x0001, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION("SWA:8")
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0001, DEF_STR( On ) )
	PORT_DIPNAME( 0xc000, 0xc000, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:2,1")
	PORT_DIPSETTING(      0x0000, "1" )
	PORT_DIPSETTING(      0xc000, "2" )
	PORT_DIPSETTING(      0x4000, "3" )
	PORT_DIPSETTING(      0x8000, "4" )
	PORT_DIPNAME( 0x3000, 0x3000, "Energy" ) PORT_DIPLOCATION("SWB:4,3")
	PORT_DIPSETTING(      0x0000, "2" )
	PORT_DIPSETTING(      0x3000, "3" )
	PORT_DIPSETTING(      0x1000, "4" )
	PORT_DIPSETTING(      0x2000, "5" )
	PORT_DIPNAME( 0x0c00, 0x0c00, DEF_STR( Difficulty ) ) PORT_DIPLOCATION("SWB:6,5")
	PORT_DIPSETTING(      0x0c00, DEF_STR( Normal ) )
	PORT_DIPSETTING(      0x0400, "TBL 1" )
	PORT_DIPSETTING(      0x0800, "TBL 2" )
	PORT_DIPSETTING(      0x0000, "TBL 3" )
	PORT_DIPUNUSED_DIPLOC( 0x0200, 0x0200, "SWB:7" )
	PORT_DIPUNUSED_DIPLOC( 0x0100, 0x0100, "SWB:8" )
INPUT_PORTS_END

static INPUT_PORTS_START( drgnbowl )
	PORT_INCLUDE(  common )

	PORT_MODIFY("DSW")
	PORT_DIPUNUSED_DIPLOC( 0x0002, 0x0002, "SWA:7" ) /* No Flip Screen */
INPUT_PORTS_END

static INPUT_PORTS_START( wildfang )
	PORT_INCLUDE(  common )

	PORT_MODIFY("DSW")
	PORT_DIPNAME( 0xc000, 0xc000, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:2,1")
	PORT_DIPSETTING(      0x8000, "1" )
	PORT_DIPSETTING(      0xc000, "2" )
	PORT_DIPSETTING(      0x4000, "3" )
/*  PORT_DIPSETTING(      0x0000, "2" ) */
	/* When bit 0 is On,  use bits 4 and 5 for difficulty */
	PORT_DIPNAME( 0x3000, 0x3000, "Difficulty (Tecmo Knight)" ) PORT_DIPLOCATION("SWB:4,3")
	PORT_DIPSETTING(      0x3000, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Normal ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	/* When bit 0 is 0ff, use bits 2 and 3 for difficulty */
	PORT_DIPNAME( 0x0c00, 0x0c00, "Difficulty (Wild Fang)" ) PORT_DIPLOCATION("SWB:6,5")
	PORT_DIPSETTING(      0x0c00, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Normal ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0100, 0x0100, "Title" ) PORT_DIPLOCATION("SWB:8")   // also affects Difficulty Table (see above)
	PORT_DIPSETTING(      0x0100, "Wild Fang" )
	PORT_DIPSETTING(      0x0000, "Tecmo Knight" )
INPUT_PORTS_END

static INPUT_PORTS_START( tknight )
	PORT_INCLUDE(  wildfang )

	PORT_MODIFY("DSW")
	PORT_DIPUNUSED_DIPLOC( 0x2000, 0x2000, "SWB:3" ) /* No separate difficulty option like parent set */
	PORT_DIPUNUSED_DIPLOC( 0x1000, 0x1000, "SWB:4" ) /* No separate difficulty option like parent set */
	PORT_DIPUNUSED_DIPLOC( 0x0100, 0x0100, "SWB:8" ) /* No title change option */
INPUT_PORTS_END

static INPUT_PORTS_START( raiga )
	PORT_INCLUDE(  common )

	PORT_MODIFY("P1_P2")    /* Only 2 Buttons */
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	/* Dip Switches order fits the first screen */
	PORT_MODIFY("DSW")
	PORT_DIPNAME( 0x00f0, 0x00f0, DEF_STR( Coin_A ) ) PORT_DIPLOCATION("SWA:4,3,2,1")
	PORT_DIPSETTING(      0x0000, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x00a0, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( 3C_2C ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( 4C_3C ) )
	PORT_DIPSETTING(      0x00f0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x00c0, DEF_STR( 3C_4C ) )
	PORT_DIPSETTING(      0x00e0, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(      0x0070, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0060, DEF_STR( 2C_5C ) )
	PORT_DIPSETTING(      0x00b0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x0030, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(      0x00d0, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(      0x0050, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(      0x0090, DEF_STR( 1C_7C ) )
	PORT_DIPNAME( 0x000f, 0x000f, DEF_STR( Coin_B ) ) PORT_DIPLOCATION("SWA:8,7,6,5")
	PORT_DIPSETTING(      0x0000, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x000a, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0001, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( 3C_2C ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( 4C_3C ) )
	PORT_DIPSETTING(      0x000f, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x000c, DEF_STR( 3C_4C ) )
	PORT_DIPSETTING(      0x000e, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(      0x0007, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0006, DEF_STR( 2C_5C ) )
	PORT_DIPSETTING(      0x000b, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x0003, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(      0x000d, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(      0x0005, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(      0x0009, DEF_STR( 1C_7C ) )
	PORT_DIPNAME( 0x8000, 0x0000, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION("SWB:1")
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPUNUSED_DIPLOC( 0x4000, 0x4000, "SWB:2" )
	PORT_DIPNAME( 0x3000, 0x1000, DEF_STR( Difficulty ) ) PORT_DIPLOCATION("SWB:4,3")
	PORT_DIPSETTING(      0x3000, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Normal ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0c00, 0x0c00, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:6,5")
	PORT_DIPSETTING(      0x0000, "2" )
	PORT_DIPSETTING(      0x0c00, "3" )
	PORT_DIPSETTING(      0x0400, "4" )
	PORT_DIPSETTING(      0x0800, "5" )
	PORT_DIPNAME( 0x0300, 0x0300, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:8,7")
	PORT_DIPSETTING(      0x0300, "50k 200k" )
	PORT_DIPSETTING(      0x0100, "100k 300k" )
	PORT_DIPSETTING(      0x0200, "50k only" )
	PORT_DIPSETTING(      0x0000, DEF_STR( None ) )
INPUT_PORTS_END



static const gfx_layout tilelayout =
{
	8,8,    /* tile size */
	RGN_FRAC(1,1),  /* number of tiles */
	4,  /* 4 bits per pixel */
	{ 0, 1, 2, 3 },
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	32*8    /* offset to next tile */
};

static const gfx_layout tile2layout =
{
	16,16,  /* tile size */
	RGN_FRAC(1,1),  /* number of tiles */
	4,  /* 4 bits per pixel */
	{ 0, 1, 2, 3 }, /* the bitplanes are packed in one nibble */
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4,
		32*8+0*4, 32*8+1*4, 32*8+2*4, 32*8+3*4,
		32*8+4*4, 32*8+5*4, 32*8+6*4, 32*8+7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
		16*32, 17*32, 18*32, 19*32, 20*32, 21*32, 22*32, 23*32},
	128*8   /* offset to next tile */
};

static const gfx_layout spritelayout =
{
	8,8,    /* sprites size */
	RGN_FRAC(1,2),  /* number of sprites */
	4,  /* 4 bits per pixel */
	{ 0, 1, 2, 3 }, /* the bitplanes are packed in one nibble */
	{ 0,4,RGN_FRAC(1,2),4+RGN_FRAC(1,2),8,12,8+RGN_FRAC(1,2),12+RGN_FRAC(1,2) },
	{ 0*16, 1*16, 2*16, 3*16, 4*16, 5*16, 6*16, 7*16 },
	16*8    /* offset to next sprite */
};

static GFXDECODE_START( gaiden )
	GFXDECODE_ENTRY( "gfx1", 0, tilelayout,        0x100, 16 )  /* tiles 8x8  */
	GFXDECODE_ENTRY( "gfx2", 0, tile2layout,       0x000, 0x1000 )  /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx3", 0, tile2layout,       0x000, 0x1000 )  /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx4", 0, spritelayout,      0x000, 0x1000 )  /* sprites 8x8 */
GFXDECODE_END

static GFXDECODE_START( raiga )
	GFXDECODE_ENTRY( "gfx1", 0, tilelayout,        0x100, 16 )  /* tiles 8x8  */
	GFXDECODE_ENTRY( "gfx2", 0, tile2layout,       0x000, 0x1000 )  /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx3", 0, tile2layout,       0x000, 0x1000 ) /* tiles 16x16 (only colors 0x00-x0f and 0x80-0x8f are used) */
	GFXDECODE_ENTRY( "gfx4", 0, spritelayout,      0x000, 0x1000 ) /* sprites 8x8 (only colors 0x00-x0f and 0x80-0x8f are used) */
GFXDECODE_END

static const gfx_layout mastninj_tile2layout =
{
	16,16,
	RGN_FRAC(1,1),
	4,
	{ 24,16,8,0 },
	{ 0,1,2,3,4,5,6,7, 256,257,258,259,260,261,262,263 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32, 512+0*32, 512+1*32, 512+2*32, 512+3*32, 512+4*32, 512+5*32, 512+6*32, 512+7*32},
	32*32
};

static const gfx_layout mastninj_spritelayout =
{
	16,16,    /* tile size */
	RGN_FRAC(1,4),  /* number of tiles */
	4,  /* 4 bits per pixel */
	{ RGN_FRAC(0,4),RGN_FRAC(1,4),RGN_FRAC(2,4),RGN_FRAC(3,4) },
	{ 0,1,2,3,4,5,6,7,  128+0,128+1,128+2,128+3,128+4,128+5,128+6,128+7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },
	32*8 /* offset to next tile */
};

static GFXDECODE_START( mastninj )
	GFXDECODE_ENTRY( "gfx1", 0, tilelayout,        0x000, 16 )  /* tiles 8x8  */
	GFXDECODE_ENTRY( "gfx2", 0, mastninj_tile2layout,       0x300, 16 ) /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx3", 0, mastninj_tile2layout,       0x200, 16 ) /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx4", 0, mastninj_spritelayout,      0x100, 16 ) /* sprites 16x16 */
GFXDECODE_END

static const gfx_layout drgnbowl_tile2layout =
{
	16,16,
	RGN_FRAC(1,8),
	4,
	{ RGN_FRAC(3,4),RGN_FRAC(2,4),RGN_FRAC(1,4),RGN_FRAC(0,4) },
	{ STEP8(0,1), STEP8(8*8*2,1) },
	{ STEP16(0,8) },
	32*8
};

static const gfx_layout drgnbowl_spritelayout =
{
	16,16,
	RGN_FRAC(1,4),
	4,
	{ RGN_FRAC(0,4),RGN_FRAC(1,4),RGN_FRAC(2,4),RGN_FRAC(3,4) },
	{ STEP8(0,1), STEP8(8*8*2,1) },
	{ STEP16(0,8) },
	32*8
};

static GFXDECODE_START( drgnbowl )
	GFXDECODE_ENTRY( "gfx1", 0,       tilelayout,                0, 16 )    /* tiles 8x8  */
	GFXDECODE_ENTRY( "gfx2", 0x00000, drgnbowl_tile2layout,  0x300, 16 )    /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx2", 0x20000, drgnbowl_tile2layout,  0x200, 16 )    /* tiles 16x16 */
	GFXDECODE_ENTRY( "gfx3", 0,       drgnbowl_spritelayout, 0x100, 16 )    /* sprites 16x16 */
GFXDECODE_END


static MACHINE_CONFIG_START( shadoww )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, 18432000/2) /* 9.216 MHz */
	MCFG_CPU_PROGRAM_MAP(gaiden_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", gaiden_state,  irq5_line_hold)

	MCFG_CPU_ADD("audiocpu", Z80, 4000000)  /* 4 MHz */
	MCFG_CPU_PROGRAM_MAP(sound_map)
								/* IRQs are triggered by the YM2203 */

	MCFG_MACHINE_START_OVERRIDE(gaiden_state,raiga)
	MCFG_MACHINE_RESET_OVERRIDE(gaiden_state,raiga)

	MCFG_WATCHDOG_ADD("watchdog")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(59.17)   /* verified on pcb */
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 4*8, 32*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(gaiden_state, screen_update_gaiden)

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", gaiden)
	MCFG_PALETTE_ADD("palette", 4096)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	MCFG_DEVICE_ADD("spritegen", TECMO_SPRITE, 0)
	MCFG_TECMO_SPRITE_GFX_REGION(3)

	MCFG_DEVICE_ADD("mixer", TECMO_MIXER, 0)
	MCFG_TECMO_MIXER_SHIFTS(10,9,4)
	MCFG_TECMO_MIXER_BLENDCOLS(   0x0400 + 0x300, 0x0400 + 0x200, 0x0400 + 0x100, 0x0400 + 0x000 )
	MCFG_TECMO_MIXER_REGULARCOLS( 0x0000 + 0x300, 0x0000 + 0x200, 0x0000 + 0x100, 0x0000 + 0x000 )
	MCFG_TECMO_MIXER_BLENDSOURCE( 0x0800 + 0x000, 0x0800 + 0x200)
	MCFG_TECMO_MIXER_REVSPRITETILE
	MCFG_TECMO_MIXER_BGPEN(0x000 + 0x200)

	MCFG_VIDEO_START_OVERRIDE(gaiden_state,gaiden)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ym1", YM2203, 4000000)
	MCFG_YM2203_IRQ_HANDLER(INPUTLINE("audiocpu", 0))
	MCFG_SOUND_ROUTE(0, "mono", 0.15)
	MCFG_SOUND_ROUTE(1, "mono", 0.15)
	MCFG_SOUND_ROUTE(2, "mono", 0.15)
	MCFG_SOUND_ROUTE(3, "mono", 0.60)

	MCFG_SOUND_ADD("ym2", YM2203, 4000000)
	MCFG_SOUND_ROUTE(0, "mono", 0.15)
	MCFG_SOUND_ROUTE(1, "mono", 0.15)
	MCFG_SOUND_ROUTE(2, "mono", 0.15)
	MCFG_SOUND_ROUTE(3, "mono", 0.60)

	MCFG_OKIM6295_ADD("oki", 1000000, PIN7_HIGH)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.20)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( raiga, shadoww )

	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_UPDATE_DRIVER(gaiden_state, screen_update_raiga)

	MCFG_VIDEO_START_OVERRIDE(gaiden_state,raiga)
	MCFG_GFXDECODE_MODIFY("gfxdecode", raiga)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( drgnbowl )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, 20000000/2) /* 10 MHz */
	MCFG_CPU_PROGRAM_MAP(drgnbowl_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", gaiden_state,  irq5_line_hold)

	MCFG_CPU_ADD("audiocpu", Z80, 12000000/2)   /* 6 MHz */
	MCFG_CPU_PROGRAM_MAP(drgnbowl_sound_map)
	MCFG_CPU_IO_MAP(drgnbowl_sound_port_map)

	MCFG_MACHINE_START_OVERRIDE(gaiden_state,raiga)
	MCFG_MACHINE_RESET_OVERRIDE(gaiden_state,raiga)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(gaiden_state, screen_update_drgnbowl)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", drgnbowl)
	MCFG_PALETTE_ADD("palette", 4096)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	/* NOT using Tecmo Sprite device - significant changes, maybe a clone of something else */

	MCFG_VIDEO_START_OVERRIDE(gaiden_state,drgnbowl)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_YM2151_ADD("ymsnd", 4000000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.40)

	MCFG_OKIM6295_ADD("oki", 1000000, PIN7_HIGH)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

/*
Master Ninja

CPUs

QTY     Type                position            function
1x      MC68000P10          68000 main PCB      main
1x      Z8400HB1            ic36 main PCB       sound
2x      YM2203C             ic31,ic32 main PCB  sound
2x      Y3014B              c16,ic17 main PCB   sound
2x      LM324N              ic2,ic6 main PCB    sound
1x      TDA2002             ic1 main PCB        sound

Oscillators
1x      oscillator 20.000 (xl2 main PCB)
1x      blu resonator 400K (xl1 main PCB)

ROMs

QTY     Type                position            status
6x      AM27C512            1-6 main PCB        dumped
32x     AM27C512            8-39 ROMs PCB       dumped

RAMs

QTY     Type                position
4x      HY6264LP-10         ic25,ic28,ic61,ic62 main PCB
3x      HY6116ALP-10        ic33,ic123,ic124 main PCB
1x      HM6148P             ic80 main PCB
1x      MCM2018AN45         ic81 main PCB
4x      HM6148P             ic63-66 ROMs PCB
1x      HY6116ALP-10        ic85 ROMs PCB

PLDs

QTY     Type                position            status
2x      TIBPAL16L8          ic15,ic54 main PCB  read protected
1x      GAL16L8             ic42 ROMs PCB       read protected

Others

1x      JAMMA edge connector
1x      trimmer (volume)
2x      8x2 switches DIP
*/

static ADDRESS_MAP_START( mastninj_sound_map, AS_PROGRAM, 8, gaiden_state )
	AM_RANGE(0x0000, 0xdfff) AM_ROM
	AM_RANGE(0xf000, 0xf7ff) AM_RAM
	AM_RANGE(0xc400, 0xc401) AM_DEVWRITE("ym1", ym2203_device, write)
	AM_RANGE(0xc800, 0xc801) AM_DEVWRITE("ym2", ym2203_device, write)
//  AM_RANGE(0xfc00, 0xfc00) AM_NOP /* ?? */
//  AM_RANGE(0xfc20, 0xfc20) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
ADDRESS_MAP_END

static ADDRESS_MAP_START( mastninj_map, AS_PROGRAM, 16, gaiden_state )
	AM_RANGE(0x000000, 0x03ffff) AM_ROM
	AM_RANGE(0x060000, 0x063fff) AM_RAM
	AM_RANGE(0x070000, 0x070fff) AM_RAM_WRITE(gaiden_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x072000, 0x073fff) AM_RAM_WRITE(gaiden_videoram2_w) AM_SHARE("videoram2")
	AM_RANGE(0x074000, 0x075fff) AM_RAM_WRITE(gaiden_videoram3_w) AM_SHARE("videoram3")
	AM_RANGE(0x076000, 0x077fff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x078000, 0x079fff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
//  AM_RANGE(0x078800, 0x079fff) AM_RAM
	AM_RANGE(0x07a000, 0x07a001) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x07a002, 0x07a003) AM_READ_PORT("P1_P2")
	AM_RANGE(0x07a004, 0x07a005) AM_READ_PORT("DSW")
//  AM_RANGE(0x07a104, 0x07a105) AM_WRITE(gaiden_txscrolly_w)
//  AM_RANGE(0x07a10c, 0x07a10d) AM_WRITE(gaiden_txscrollx_w)
	AM_RANGE(0x07f000, 0x07f001) AM_WRITE(gaiden_bgscrolly_w)
	AM_RANGE(0x07f002, 0x07f003) AM_WRITE(gaiden_bgscrollx_w)
	AM_RANGE(0x07f004, 0x07f005) AM_WRITE(gaiden_fgscrolly_w)
	AM_RANGE(0x07f006, 0x07f007) AM_WRITE(gaiden_fgscrollx_w)
	AM_RANGE(0x07a800, 0x07a801) AM_DEVWRITE("watchdog", watchdog_timer_device, reset16_w)
	AM_RANGE(0x07e000, 0x07e001) AM_WRITE(gaiden_sound_command_w)
//  AM_RANGE(0x07a806, 0x07a807) AM_WRITENOP
//  AM_RANGE(0x07a808, 0x07a809) AM_WRITE(gaiden_flip_w)
ADDRESS_MAP_END

static MACHINE_CONFIG_START( mastninj )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, 10000000)   /* 10 MHz? */
	MCFG_CPU_PROGRAM_MAP(mastninj_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", gaiden_state,  irq5_line_hold)

	MCFG_CPU_ADD("audiocpu", Z80, 4000000)  /* ?? MHz */
	MCFG_CPU_PROGRAM_MAP(mastninj_sound_map)
								/* IRQs are triggered by the YM2203 */

	MCFG_MACHINE_START_OVERRIDE(gaiden_state,raiga)
	MCFG_MACHINE_RESET_OVERRIDE(gaiden_state,raiga)

	MCFG_WATCHDOG_ADD("watchdog")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(gaiden_state, screen_update_drgnbowl)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", mastninj)
	MCFG_PALETTE_ADD("palette", 4096)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	/* NOT using Tecmo Sprite device - significant changes, maybe a clone of something else */

	MCFG_VIDEO_START_OVERRIDE(gaiden_state,drgnbowl)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ym1", YM2203, 4000000) /* ?? MHz */
	MCFG_YM2203_IRQ_HANDLER(INPUTLINE("audiocpu", 0))
	MCFG_SOUND_ROUTE(0, "mono", 0.15)
	MCFG_SOUND_ROUTE(1, "mono", 0.15)
	MCFG_SOUND_ROUTE(2, "mono", 0.15)
	MCFG_SOUND_ROUTE(3, "mono", 0.60)

	MCFG_SOUND_ADD("ym2", YM2203, 4000000) /* ?? MHz */
	MCFG_SOUND_ROUTE(0, "mono", 0.15)
	MCFG_SOUND_ROUTE(1, "mono", 0.15)
	MCFG_SOUND_ROUTE(2, "mono", 0.15)
	MCFG_SOUND_ROUTE(3, "mono", 0.60)

	/* no OKI on the bootleg */
//  MCFG_OKIM6295_ADD("oki", 1000000, PIN7_HIGH)
//  MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.20)
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( shadoww )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "shadowa_1.3s",     0x00000, 0x20000, CRC(8290d567) SHA1(1e2f80c1548c853ec1127e79438f62eda6592a07) )
	ROM_LOAD16_BYTE( "shadowa_2.4s",     0x00001, 0x20000, CRC(f3f08921) SHA1(df6bb7302714e0eab12cbd0a7f2a4ca751a600e1) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "gaiden_3.4b",     0x0000, 0x10000, CRC(75fd3e6a) SHA1(3333e84ed4983caa133e60a8e8895fa897ab4949) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "gaiden_5.7a",     0x000000, 0x10000, CRC(8d4035f7) SHA1(3473456cdd24e312e3073586d7e8f24eb71bbea1) )  /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",       0x000000, 0x20000, CRC(1ecfddaa) SHA1(e71d60ae1a98fe8512498f91cce01c16be9f0871) )
	ROM_LOAD( "15.3b",       0x020000, 0x20000, CRC(1291a696) SHA1(023b05260214adc39bdba81d5e2aa246b6d74a6a) )
	ROM_LOAD( "16.1a",       0x040000, 0x20000, CRC(140b47ca) SHA1(6ffd9b7116658a46a124f9085602d88aa143d829) )
	ROM_LOAD( "17.1b",       0x060000, 0x20000, CRC(7638cccb) SHA1(780d47d3aa248346e0e7abc6e6284542e7392919) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "18.6a",       0x000000, 0x20000, CRC(3fadafd6) SHA1(0cb5387a354c631d5c6aca8f77ecbbc0d175a574) )
	ROM_LOAD( "19.6b",       0x020000, 0x20000, CRC(ddae9d5b) SHA1(108b202ae7ae124a32400a0a404c7d2b614c60bd) )
	ROM_LOAD( "20.4b",       0x040000, 0x20000, CRC(08cf7a93) SHA1(fd3278c3fb3ef30ed03c8a95656d86ba82a163d8) )
	ROM_LOAD( "21.4b",       0x060000, 0x20000, CRC(1ac892f5) SHA1(28364266ca9d1955fb7953f5c2d6f35e114beec6) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	/* Should all these roms be 0x10000 bytes on this board? */
	ROM_LOAD( "6.3m",           0x000000, 0x20000, CRC(e7ccdf9f) SHA1(80ffcefc95660471124898a9c2bee55df36bda13) )   /* sprites A1 */
	ROM_LOAD( "8.3n",           0x020000, 0x20000, CRC(7ef7f880) SHA1(26ba9a76adce24beea3cffa1cb95aeafe6f82f96) )   /* sprites B1 */
	ROM_LOAD( "10.3r",          0x040000, 0x20000, CRC(a6451dec) SHA1(553e7a1453b59055fa0b10ca04125543d9f8987c) )   /* sprites C1 */
	ROM_LOAD( "shadoww_12a.xx", 0x060000, 0x10000, CRC(9bb07731) SHA1(b799b1958dc9b84797fdab2591e33bd5d28884a3) )   /* sprites D1 */
	ROM_LOAD( "shadoww_12b.xx", 0x070000, 0x10000, CRC(a4a950a2) SHA1(9766b5e88edd16554e59179a37cca49d29f83367) )   /* sprites D1 */

	ROM_LOAD( "7.1m",           0x080000, 0x20000, CRC(016bec95) SHA1(6a6757c52ca9a2398ea43d1af4a8d5adde6f4cd2) )   /* sprites A2 */
	ROM_LOAD( "9.1n",           0x0a0000, 0x20000, CRC(6e9b7fd3) SHA1(c86ff61844fc94c02625bb812b9062d0649c8fdf) )   /* sprites B2 */
	ROM_LOAD( "11.1r",          0x0c0000, 0x20000, CRC(7fbfdf5e) SHA1(ab67b72dcadb5f2236d29de751de5bf890a9e423) )   /* sprites C2 */
	ROM_LOAD( "shadoww_13a.xx", 0x0e0000, 0x10000, CRC(996d2fa5) SHA1(a32526949af3635914927ebbbe684c3de9562a9d) )   /* sprites D2 */
	ROM_LOAD( "shadoww_13b.xx", 0x0f0000, 0x10000, CRC(b8df8a34) SHA1(6810f7961052a983b8c78b42d550038051012c6d) )   /* sprites D2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "4.4a",     0x0000, 0x20000, CRC(b0e0faf9) SHA1(2275d2ef5eee356ccf80b9e9644d16fc30a4d107) ) /* samples */
ROM_END

ROM_START( shadowwa )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "shadoww_1.3s",    0x00000, 0x20000, CRC(fefba387) SHA1(20ce28da5877009494c3f3f67488bbe805d91340) )
	ROM_LOAD16_BYTE( "shadoww_2.4s",    0x00001, 0x20000, CRC(9b9d6b18) SHA1(75068611fb1de61120be8bf840f61d90c0dc86ca) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "gaiden_3.4b",     0x0000, 0x10000, CRC(75fd3e6a) SHA1(3333e84ed4983caa133e60a8e8895fa897ab4949) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "gaiden_5.7a",     0x000000, 0x10000, CRC(8d4035f7) SHA1(3473456cdd24e312e3073586d7e8f24eb71bbea1) )  /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",       0x000000, 0x20000, CRC(1ecfddaa) SHA1(e71d60ae1a98fe8512498f91cce01c16be9f0871) )
	ROM_LOAD( "15.3b",       0x020000, 0x20000, CRC(1291a696) SHA1(023b05260214adc39bdba81d5e2aa246b6d74a6a) )
	ROM_LOAD( "16.1a",       0x040000, 0x20000, CRC(140b47ca) SHA1(6ffd9b7116658a46a124f9085602d88aa143d829) )
	ROM_LOAD( "17.1b",       0x060000, 0x20000, CRC(7638cccb) SHA1(780d47d3aa248346e0e7abc6e6284542e7392919) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "18.6a",       0x000000, 0x20000, CRC(3fadafd6) SHA1(0cb5387a354c631d5c6aca8f77ecbbc0d175a574) )
	ROM_LOAD( "19.6b",       0x020000, 0x20000, CRC(ddae9d5b) SHA1(108b202ae7ae124a32400a0a404c7d2b614c60bd) )
	ROM_LOAD( "20.4b",       0x040000, 0x20000, CRC(08cf7a93) SHA1(fd3278c3fb3ef30ed03c8a95656d86ba82a163d8) )
	ROM_LOAD( "21.4b",       0x060000, 0x20000, CRC(1ac892f5) SHA1(28364266ca9d1955fb7953f5c2d6f35e114beec6) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	/* Should all these roms be 0x10000 bytes on this board? */
	ROM_LOAD( "6.3m",           0x000000, 0x20000, CRC(e7ccdf9f) SHA1(80ffcefc95660471124898a9c2bee55df36bda13) )   /* sprites A1 */
	ROM_LOAD( "8.3n",           0x020000, 0x20000, CRC(7ef7f880) SHA1(26ba9a76adce24beea3cffa1cb95aeafe6f82f96) )   /* sprites B1 */
	ROM_LOAD( "10.3r",          0x040000, 0x20000, CRC(a6451dec) SHA1(553e7a1453b59055fa0b10ca04125543d9f8987c) )   /* sprites C1 */
	ROM_LOAD( "shadoww_12a.xx", 0x060000, 0x10000, CRC(9bb07731) SHA1(b799b1958dc9b84797fdab2591e33bd5d28884a3) )   /* sprites D1 */
	ROM_LOAD( "shadoww_12b.xx", 0x070000, 0x10000, CRC(a4a950a2) SHA1(9766b5e88edd16554e59179a37cca49d29f83367) )   /* sprites D1 */

	ROM_LOAD( "7.1m",           0x080000, 0x20000, CRC(016bec95) SHA1(6a6757c52ca9a2398ea43d1af4a8d5adde6f4cd2) )   /* sprites A2 */
	ROM_LOAD( "9.1n",           0x0a0000, 0x20000, CRC(6e9b7fd3) SHA1(c86ff61844fc94c02625bb812b9062d0649c8fdf) )   /* sprites B2 */
	ROM_LOAD( "11.1r",          0x0c0000, 0x20000, CRC(7fbfdf5e) SHA1(ab67b72dcadb5f2236d29de751de5bf890a9e423) )   /* sprites C2 */
	ROM_LOAD( "shadoww_13a.xx", 0x0e0000, 0x10000, CRC(996d2fa5) SHA1(a32526949af3635914927ebbbe684c3de9562a9d) )   /* sprites D2 */
	ROM_LOAD( "shadoww_13b.xx", 0x0f0000, 0x10000, CRC(b8df8a34) SHA1(6810f7961052a983b8c78b42d550038051012c6d) )   /* sprites D2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "4.4a",     0x0000, 0x20000, CRC(b0e0faf9) SHA1(2275d2ef5eee356ccf80b9e9644d16fc30a4d107) ) /* samples */
ROM_END

ROM_START( gaiden )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "gaiden_1.3s",     0x00000, 0x20000, CRC(e037ff7c) SHA1(5418bcb80d4c52f05e3c26668193452fd51f1283) )
	ROM_LOAD16_BYTE( "gaiden_2.4s",     0x00001, 0x20000, CRC(454f7314) SHA1(231296423870f00ea2e545faf0fbb37577430a4f) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "gaiden_3.4b",     0x0000, 0x10000, CRC(75fd3e6a) SHA1(3333e84ed4983caa133e60a8e8895fa897ab4949) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "gaiden_5.7a",     0x000000, 0x10000, CRC(8d4035f7) SHA1(3473456cdd24e312e3073586d7e8f24eb71bbea1) )  /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",       0x000000, 0x20000, CRC(1ecfddaa) SHA1(e71d60ae1a98fe8512498f91cce01c16be9f0871) )
	ROM_LOAD( "15.3b",       0x020000, 0x20000, CRC(1291a696) SHA1(023b05260214adc39bdba81d5e2aa246b6d74a6a) )
	ROM_LOAD( "16.1a",       0x040000, 0x20000, CRC(140b47ca) SHA1(6ffd9b7116658a46a124f9085602d88aa143d829) )
	ROM_LOAD( "17.1b",       0x060000, 0x20000, CRC(7638cccb) SHA1(780d47d3aa248346e0e7abc6e6284542e7392919) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "18.6a",       0x000000, 0x20000, CRC(3fadafd6) SHA1(0cb5387a354c631d5c6aca8f77ecbbc0d175a574) )
	ROM_LOAD( "19.6b",       0x020000, 0x20000, CRC(ddae9d5b) SHA1(108b202ae7ae124a32400a0a404c7d2b614c60bd) )
	ROM_LOAD( "20.4b",       0x040000, 0x20000, CRC(08cf7a93) SHA1(fd3278c3fb3ef30ed03c8a95656d86ba82a163d8) )
	ROM_LOAD( "21.4b",       0x060000, 0x20000, CRC(1ac892f5) SHA1(28364266ca9d1955fb7953f5c2d6f35e114beec6) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "6.3m",         0x000000, 0x20000, CRC(e7ccdf9f) SHA1(80ffcefc95660471124898a9c2bee55df36bda13) ) /* sprites A1 */
	ROM_LOAD( "8.3n",         0x020000, 0x20000, CRC(7ef7f880) SHA1(26ba9a76adce24beea3cffa1cb95aeafe6f82f96) ) /* sprites B1 */
	ROM_LOAD( "10.3r",        0x040000, 0x20000, CRC(a6451dec) SHA1(553e7a1453b59055fa0b10ca04125543d9f8987c) ) /* sprites C1 */
	ROM_LOAD( "gaiden_12.3s", 0x060000, 0x20000, CRC(90f1e13a) SHA1(3fe9fe62aa9e92c871c791a3b11f96c9a48099a9) ) /* sprites D1 */

	ROM_LOAD( "7.1m",         0x080000, 0x20000, CRC(016bec95) SHA1(6a6757c52ca9a2398ea43d1af4a8d5adde6f4cd2) ) /* sprites A2 */
	ROM_LOAD( "9.1n",         0x0a0000, 0x20000, CRC(6e9b7fd3) SHA1(c86ff61844fc94c02625bb812b9062d0649c8fdf) ) /* sprites B2 */
	ROM_LOAD( "11.1r",        0x0c0000, 0x20000, CRC(7fbfdf5e) SHA1(ab67b72dcadb5f2236d29de751de5bf890a9e423) ) /* sprites C2 */
	ROM_LOAD( "gaiden_13.1s", 0x0e0000, 0x20000, CRC(7d9f5c5e) SHA1(200102532ea9a88c7c708e03f8893c46dff827d1) ) /* sprites D2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "4.4a",     0x0000, 0x20000, CRC(b0e0faf9) SHA1(2275d2ef5eee356ccf80b9e9644d16fc30a4d107) ) /* samples */
ROM_END

ROM_START( ryukendn )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "ryukendn_1.3s",  0x00000, 0x20000, CRC(6203a5e2) SHA1(8cfe05c483a351e938b067ffa642d515e28605a3) )
	ROM_LOAD16_BYTE( "ryukendn_2.4s",  0x00001, 0x20000, CRC(9e99f522) SHA1(b2277d8934b5e6e2f556aee5092f5d1050774a34) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "3.4b",   0x0000, 0x10000, CRC(6b686b69) SHA1(f0fa553acb3945f8dbbf466073c8bae35a0375ef) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "hn27512p.7a",   0x000000, 0x10000, CRC(765e7baa) SHA1(4d0a50f091b284739b6d9a8ceb4f81999da445fc) )    /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",       0x000000, 0x20000, CRC(1ecfddaa) SHA1(e71d60ae1a98fe8512498f91cce01c16be9f0871) )
	ROM_LOAD( "15.3b",       0x020000, 0x20000, CRC(1291a696) SHA1(023b05260214adc39bdba81d5e2aa246b6d74a6a) )
	ROM_LOAD( "16.1a",       0x040000, 0x20000, CRC(140b47ca) SHA1(6ffd9b7116658a46a124f9085602d88aa143d829) )
	ROM_LOAD( "17.1b",       0x060000, 0x20000, CRC(7638cccb) SHA1(780d47d3aa248346e0e7abc6e6284542e7392919) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "18.6a",       0x000000, 0x20000, CRC(3fadafd6) SHA1(0cb5387a354c631d5c6aca8f77ecbbc0d175a574) )
	ROM_LOAD( "19.6b",       0x020000, 0x20000, CRC(ddae9d5b) SHA1(108b202ae7ae124a32400a0a404c7d2b614c60bd) )
	ROM_LOAD( "20.4b",       0x040000, 0x20000, CRC(08cf7a93) SHA1(fd3278c3fb3ef30ed03c8a95656d86ba82a163d8) )
	ROM_LOAD( "21.4b",       0x060000, 0x20000, CRC(1ac892f5) SHA1(28364266ca9d1955fb7953f5c2d6f35e114beec6) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	/* Should all these roms be 0x10000 bytes on this board? */
	ROM_LOAD( "6.3m",            0x000000, 0x20000, CRC(e7ccdf9f) SHA1(80ffcefc95660471124898a9c2bee55df36bda13) )  /* sprites A1 */
	ROM_LOAD( "8.3n",            0x020000, 0x20000, CRC(7ef7f880) SHA1(26ba9a76adce24beea3cffa1cb95aeafe6f82f96) )  /* sprites B1 */
	ROM_LOAD( "10.3r",           0x040000, 0x20000, CRC(a6451dec) SHA1(553e7a1453b59055fa0b10ca04125543d9f8987c) )  /* sprites C1 */
	ROM_LOAD( "shadoww_12a.xx",  0x060000, 0x10000, CRC(9bb07731) SHA1(b799b1958dc9b84797fdab2591e33bd5d28884a3) )  /* sprites D1 */
	ROM_LOAD( "ryukendn_12b.xx", 0x070000, 0x10000, CRC(1773628a) SHA1(e7eacc880f2a4174f17b263bedf8c1bc64007dbd) )  /* sprites D1 */

	ROM_LOAD( "7.1m",            0x080000, 0x20000, CRC(016bec95) SHA1(6a6757c52ca9a2398ea43d1af4a8d5adde6f4cd2) )  /* sprites A2 */
	ROM_LOAD( "9.1n",            0x0a0000, 0x20000, CRC(6e9b7fd3) SHA1(c86ff61844fc94c02625bb812b9062d0649c8fdf) )  /* sprites B2 */
	ROM_LOAD( "11.1r",           0x0c0000, 0x20000, CRC(7fbfdf5e) SHA1(ab67b72dcadb5f2236d29de751de5bf890a9e423) )  /* sprites C2 */
	ROM_LOAD( "shadoww_13a.xx",  0x0e0000, 0x10000, CRC(996d2fa5) SHA1(a32526949af3635914927ebbbe684c3de9562a9d) )  /* sprites D2 */
	ROM_LOAD( "ryukendn_13b.xx", 0x0f0000, 0x10000, CRC(1f43c507) SHA1(29f655442c16677855073284c7ab41059c99c497) )  /* sprites D2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "4.4a",     0x0000, 0x20000, CRC(b0e0faf9) SHA1(2275d2ef5eee356ccf80b9e9644d16fc30a4d107) ) /* samples */
ROM_END

/*

Ninja Ryukenden (Japan) (Tecmo 1989)
Dumped from an original Tecmo board. Board No. 6215-A. Serial A-59488.

-------------------------------------------------------------------------------------

*/

ROM_START( ryukendna )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "1.3s",  0x00000, 0x20000, CRC(5532e302) SHA1(8ce48963ba737890d1a46c42a113d9419a3c174c) ) // found on 2 pcbs
//  ROM_LOAD16_BYTE( "1.3s",  0x00000, 0x20000, CRC(0ed5464c) SHA1(2eab6650ad1c38cd560ec3d084f47156756c97a4) ) 2 bytes different ( 022a : 50 instead of 51, 12f9 : 6b instead of 6a) - possible bad rom
	ROM_LOAD16_BYTE( "2.4s",  0x00001, 0x20000, CRC(a93a8256) SHA1(6bf6c189f82cb9341d3427a822de83cbaed27bc0) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "3.4b",   0x0000, 0x10000, CRC(6b686b69) SHA1(f0fa553acb3945f8dbbf466073c8bae35a0375ef) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "hn27512p.7a",   0x000000, 0x10000, CRC(765e7baa) SHA1(4d0a50f091b284739b6d9a8ceb4f81999da445fc) )    /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",       0x000000, 0x20000, CRC(1ecfddaa) SHA1(e71d60ae1a98fe8512498f91cce01c16be9f0871) )
	ROM_LOAD( "15.3b",       0x020000, 0x20000, CRC(1291a696) SHA1(023b05260214adc39bdba81d5e2aa246b6d74a6a) )
	ROM_LOAD( "16.1a",       0x040000, 0x20000, CRC(140b47ca) SHA1(6ffd9b7116658a46a124f9085602d88aa143d829) )
	ROM_LOAD( "17.1b",       0x060000, 0x20000, CRC(7638cccb) SHA1(780d47d3aa248346e0e7abc6e6284542e7392919) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "18.6a",       0x000000, 0x20000, CRC(3fadafd6) SHA1(0cb5387a354c631d5c6aca8f77ecbbc0d175a574) )
	ROM_LOAD( "19.6b",       0x020000, 0x20000, CRC(ddae9d5b) SHA1(108b202ae7ae124a32400a0a404c7d2b614c60bd) )
	ROM_LOAD( "20.4b",       0x040000, 0x20000, CRC(08cf7a93) SHA1(fd3278c3fb3ef30ed03c8a95656d86ba82a163d8) )
	ROM_LOAD( "21.4b",       0x060000, 0x20000, CRC(1ac892f5) SHA1(28364266ca9d1955fb7953f5c2d6f35e114beec6) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "6.3m",     0x000000, 0x20000, CRC(e7ccdf9f) SHA1(80ffcefc95660471124898a9c2bee55df36bda13) ) /* sprites A1 */
	ROM_LOAD( "8.3n",     0x020000, 0x20000, CRC(7ef7f880) SHA1(26ba9a76adce24beea3cffa1cb95aeafe6f82f96) ) /* sprites B1 */
	ROM_LOAD( "10.3r",    0x040000, 0x20000, CRC(a6451dec) SHA1(553e7a1453b59055fa0b10ca04125543d9f8987c) ) /* sprites C1 */
	ROM_LOAD( "12.3s",    0x060000, 0x20000, CRC(277204f0) SHA1(918e05f10959f2b50c16b6e0dc62e3076c99250e) ) /* sprites D1 */

	ROM_LOAD( "7.1m",     0x080000, 0x20000, CRC(016bec95) SHA1(6a6757c52ca9a2398ea43d1af4a8d5adde6f4cd2) ) /* sprites A2 */
	ROM_LOAD( "9.1n",     0x0a0000, 0x20000, CRC(6e9b7fd3) SHA1(c86ff61844fc94c02625bb812b9062d0649c8fdf) ) /* sprites B2 */
	ROM_LOAD( "11.1r",    0x0c0000, 0x20000, CRC(7fbfdf5e) SHA1(ab67b72dcadb5f2236d29de751de5bf890a9e423) ) /* sprites C2 */
	ROM_LOAD( "13.1s",    0x0e0000, 0x20000, CRC(4e56a508) SHA1(f89a6037e602b26d6ce11859e0b43a602b50d985) ) /* sprites D2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "4.4a",     0x0000, 0x20000, CRC(b0e0faf9) SHA1(2275d2ef5eee356ccf80b9e9644d16fc30a4d107) ) /* samples */
ROM_END

ROM_START( mastninj )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "3.ic27",    0x00000, 0x10000, CRC(41fedeb3) SHA1(7e95f8e5b0c38b578eedbce9afbd10dbb14cdddf) )
	ROM_LOAD16_BYTE( "1.ic30",    0x00001, 0x10000, CRC(93b1e3dd) SHA1(325b2f0ef5d92f4d760086de6cb23494d1e6c6e6) )
	ROM_LOAD16_BYTE( "4.ic26",    0x20000, 0x10000, CRC(d375c5f6) SHA1(925e84d79f35595a344a417125d74dc46ebec310) ) // 1ST AND 2ND HALF IDENTICAL (but correct?)
	ROM_LOAD16_BYTE( "2.ic29",    0x20001, 0x10000, CRC(6b53b8b1) SHA1(68bffc992fecae5e113592a9481c0cee80925135) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "5.ic35",       0x000000, 0x10000, CRC(ba528424) SHA1(5ab93059e26483a756d80b8c18d9669d2a3416de) )

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "6.ic120",      0x000000, 0x10000, CRC(847cc552) SHA1(e5e2ed19efcedb52885f9f91a1690c88a6b6261d) )

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD32_BYTE( "8.ic13",       0x000000, 0x10000, CRC(e3987e0f) SHA1(8805443e56575fc455d21703bd2f9ebef434e262) )
	ROM_LOAD32_BYTE( "10.ic11",      0x000001, 0x10000, CRC(5e8afc68) SHA1(ecef113a947b9bda6abbef5f75557cd201e355b3) )
	ROM_LOAD32_BYTE( "12.ic9",       0x000002, 0x10000, CRC(2713e9f1) SHA1(87614a79596216d2b710925167f3130d4c2e07c9) )
	ROM_LOAD32_BYTE( "14.ic7",       0x000003, 0x10000, CRC(ca59280f) SHA1(d5a1d85f75ea667812708758915f43f01c8c9830) )
	ROM_LOAD32_BYTE( "20.ic27",      0x040000, 0x10000, CRC(72e5c1c2) SHA1(0e744407c52a61ed657557978cdfe455fe5e931e) )
	ROM_LOAD32_BYTE( "22.ic25",      0x040001, 0x10000, CRC(55affaf8) SHA1(c121a904ba44dc53c8a10b8d56c4c25ab879d8be) )
	ROM_LOAD32_BYTE( "24.ic23",      0x040002, 0x10000, CRC(bd76fd53) SHA1(bc2ad054b63573f16c99f82c680f0f6de2ee4683) )
	ROM_LOAD32_BYTE( "26.ic21",      0x040003, 0x10000, CRC(f3bfcfd6) SHA1(7d73a2ae00825979b3d09502f52d78f61f3ea1a9) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD32_BYTE( "9.ic12",       0x000000, 0x10000, CRC(ae043a2e) SHA1(3513b21d4ee7f869c9ebda68707845d030b0ecad) )
	ROM_LOAD32_BYTE( "11.ic10",      0x000001, 0x10000, CRC(e197fd97) SHA1(1e92139ae6a1c15de629039757c21e32cacc42d6) )
	ROM_LOAD32_BYTE( "13.ic8",       0x000002, 0x10000, CRC(0871971c) SHA1(ede9bb5d6d968fc532217b4eb1cd4c0d7ea9a4a1) )
	ROM_LOAD32_BYTE( "15.ic6",       0x000003, 0x10000, CRC(6850aea3) SHA1(670820ba2df040ded8739907bfdde4ac97373200) )
	ROM_LOAD32_BYTE( "21.ic26",      0x040000, 0x10000, CRC(dd162ce7) SHA1(70ec5a722ea31434be2a4b3104f9c54a48b8ec05) )
	ROM_LOAD32_BYTE( "23.ic24",      0x040001, 0x10000, CRC(edd65385) SHA1(3b5c0115ae1972bfe696a22edd2da9e6fb9739f4) )
	ROM_LOAD32_BYTE( "25.ic22",      0x040002, 0x10000, CRC(ca691635) SHA1(177f94a17cfaf67c764c2a2dff48475039207fae) )
	ROM_LOAD32_BYTE( "27.ic20",      0x040003, 0x10000, CRC(2ae70f42) SHA1(aad89dbd0309a5e3a786aa028995b56859d5b5ff) )

	ROM_REGION( 0x100000, "gfx4", ROMREGION_INVERT) /* these will need a further descramble to be in the same format as gaiden, although the sprites on the bootleg look different anyway */
	ROM_LOAD( "36.ic50",           0x000000, 0x10000, CRC(3c117e62) SHA1(dee45d6bbe053996e0b3faaba0293a273faf1ffa) )
	ROM_LOAD( "37.ic49",           0x010000, 0x10000, CRC(f6d6422d) SHA1(933487b09d3bcff9714fb2469b3d751b38459cfd) )
	ROM_LOAD( "38.ic48",           0x020000, 0x10000, CRC(642f06e7) SHA1(5b30b5029884b7eddcad201224a639f94ee27823) )
	ROM_LOAD( "39.ic47",           0x030000, 0x10000, CRC(51f00702) SHA1(c2a7819beb37ebf613cb2d65476dcee39f72a781) )
	ROM_LOAD( "32.ic34",           0x040000, 0x10000, CRC(940f3dbb) SHA1(4e2f224ed2ec1b8da992bd375d3ab1cf6fbfdd1f) )
	ROM_LOAD( "33.ic33",           0x050000, 0x10000, CRC(f6baccb0) SHA1(2244d16127efe67fcf59a59e50eabc54e3081dd1) )
	ROM_LOAD( "34.ic32",           0x060000, 0x10000, CRC(bb46ef1b) SHA1(4c8f9e06fa4d7f14206f6180a999b3f32681785a) )
	ROM_LOAD( "35.ic31",           0x070000, 0x10000, CRC(c0b6ba3e) SHA1(8849cf5c7777e4b5e52c695bcb6038b3bad4e04c) )
	ROM_LOAD( "28.ic19",           0x080000, 0x10000, CRC(012da98d) SHA1(413e1f02e2e3267fb4b893b14f627105789aa1c9) )
	ROM_LOAD( "29.ic18",           0x090000, 0x10000, CRC(fa32da96) SHA1(5e240f6f91813bdafacf1d29ea65704f2c4f2ae6) )
	ROM_LOAD( "30.ic17",           0x0a0000, 0x10000, CRC(910fccdb) SHA1(99523b53ae0dbf82783ab5a731df3c02984c72fe) )
	ROM_LOAD( "31.ic16",           0x0b0000, 0x10000, CRC(d16b593b) SHA1(2895b2eba0f3ad5e209fe8c550dd8cc3c3e08742) )
	ROM_LOAD( "16.ic5",            0x0c0000, 0x10000, CRC(216eeef5) SHA1(5167af8cef220a5092add2bf578e8323360132a5) )
	ROM_LOAD( "17.ic4",            0x0d0000, 0x10000, CRC(f72f8bfd) SHA1(ccf0aab11987e76c927a73f12e5cd4bb125c1258) )
	ROM_LOAD( "18.ic3",            0x0e0000, 0x10000, CRC(6de96087) SHA1(0b9028320cb622dad07cf8bde015428eba7f8a5e) )
	ROM_LOAD( "19.ic2",            0x0f0000, 0x10000, CRC(c12c367b) SHA1(9835292f335f1353f7b9bd0bb85124942822646f) )

	ROM_REGION( 0x080000, "misc", 0 )
	ROM_LOAD( "gal16v8.ic42.bad.dump",    0x000, 0x117, BAD_DUMP CRC(61d6a8d7) SHA1(d3a6331b1fccd374e4f080740094d3832ff98ad9) )
	ROM_LOAD( "tibpal16l8.ic15.bad.dump", 0x000, 0x104, BAD_DUMP CRC(e9cd78fb) SHA1(557d3e7ef3b25c1338b24722cac91bca788c02b8) )
	ROM_LOAD( "tibpal16l8.ic54.bad.dump", 0x000, 0x104, BAD_DUMP CRC(e9cd78fb) SHA1(557d3e7ef3b25c1338b24722cac91bca788c02b8) )
ROM_END


ROM_START( wildfang ) /* Dipswitch selectable title of Wild Fang or Tecmo Knight */
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "1.3st",    0x00000, 0x20000, CRC(ab876c9b) SHA1(b02c822f107df4c9c4f0024998f225c1ddbbd496) )
	ROM_LOAD16_BYTE( "2.5st",    0x00001, 0x20000, CRC(1dc74b3b) SHA1(c99051ebefd6ce666b13ab56c0a10b188f15ec28) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "tkni3.bin",    0x0000, 0x10000, CRC(15623ec7) SHA1(db43fe6c417117d7cd90a26e12a52efb0e1a5ca6) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x1000, "mcu", 0 )  /* protection NEC D8749 */
	ROM_LOAD( "a-6v.mcu",         0x00000, 0x1000, NO_DUMP )

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "tkni5.bin",    0x000000, 0x10000, CRC(5ed15896) SHA1(87bdddb26934af0b2c4e704e6d85c69a7531aeb1) ) /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",        0x000000, 0x20000, CRC(0d20c10c) SHA1(209ca4e166d0b91ff99a338e135e5388af2c51f5) )
	ROM_LOAD( "15.3b",        0x020000, 0x20000, CRC(3f40a6b4) SHA1(7486ddfe4b0ac4198512548b74402f4194c804f1) )
	ROM_LOAD( "16.1a",        0x040000, 0x20000, CRC(0f31639e) SHA1(e150db4f617c5fcf505e5ca95d94073c1f6b7d0d) )
	ROM_LOAD( "17.1b",        0x060000, 0x20000, CRC(f32c158e) SHA1(2861754bda37e30799151b5ca73771937edf38a9) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "tkni6.bin",    0x000000, 0x80000, CRC(f68fafb1) SHA1(aeca38eaea2f6dfc484e48ac1114c0c4abaafb9c) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "tkni9.bin",    0x000000, 0x80000, CRC(d22f4239) SHA1(360a9a821faabe911eef407ef85452d8b706538f) ) /* sprites */
	ROM_LOAD( "tkni8.bin",    0x080000, 0x80000, CRC(4931b184) SHA1(864e827ac109c0ee52a898034c021cd5e92ff000) ) /* sprites */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "tkni4.bin",    0x0000, 0x20000, CRC(a7a1dbcf) SHA1(2fee1d9745ce2ab54b0b9cbb6ab2e66ba9677245) ) /* samples */
ROM_END

ROM_START( wildfangs ) /* Wild Fang - No title change option */
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "1.3s",    0x00000, 0x20000, CRC(3421f691) SHA1(7829729e2007a53fc598db3ae3524b971cbf49e9) )
	ROM_LOAD16_BYTE( "2.5s",    0x00001, 0x20000, CRC(d3547708) SHA1(91cc0575b25fe15d668eec26dd74945c51ed67eb) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "tkni3.bin",    0x0000, 0x10000, CRC(15623ec7) SHA1(db43fe6c417117d7cd90a26e12a52efb0e1a5ca6) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x1000, "mcu", 0 )  /* protection NEC D8749 */
	ROM_LOAD( "a-6v.mcu",         0x00000, 0x1000, NO_DUMP )

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "tkni5.bin",    0x000000, 0x10000, CRC(5ed15896) SHA1(87bdddb26934af0b2c4e704e6d85c69a7531aeb1) ) /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "14.3a",        0x000000, 0x20000, CRC(0d20c10c) SHA1(209ca4e166d0b91ff99a338e135e5388af2c51f5) )
	ROM_LOAD( "15.3b",        0x020000, 0x20000, CRC(3f40a6b4) SHA1(7486ddfe4b0ac4198512548b74402f4194c804f1) )
	ROM_LOAD( "16.1a",        0x040000, 0x20000, CRC(0f31639e) SHA1(e150db4f617c5fcf505e5ca95d94073c1f6b7d0d) )
	ROM_LOAD( "17.1b",        0x060000, 0x20000, CRC(f32c158e) SHA1(2861754bda37e30799151b5ca73771937edf38a9) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "tkni6.bin",    0x000000, 0x80000, CRC(f68fafb1) SHA1(aeca38eaea2f6dfc484e48ac1114c0c4abaafb9c) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "tkni9.bin",    0x000000, 0x80000, CRC(d22f4239) SHA1(360a9a821faabe911eef407ef85452d8b706538f) ) /* sprites */
	ROM_LOAD( "tkni8.bin",    0x080000, 0x80000, CRC(4931b184) SHA1(864e827ac109c0ee52a898034c021cd5e92ff000) ) /* sprites */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "tkni4.bin",    0x0000, 0x20000, CRC(a7a1dbcf) SHA1(2fee1d9745ce2ab54b0b9cbb6ab2e66ba9677245) ) /* samples */
ROM_END

ROM_START( tknight ) /* Tecmo Knight - No title change option */
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "tkni1.bin",    0x00000, 0x20000, CRC(9121daa8) SHA1(06ba7779602df8fae32e859371d27c0dbb8d3430) )
	ROM_LOAD16_BYTE( "tkni2.bin",    0x00001, 0x20000, CRC(6669cd87) SHA1(8888522a3aef76a979ffc80ba457dd49f279abf1) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "tkni3.bin",    0x0000, 0x10000, CRC(15623ec7) SHA1(db43fe6c417117d7cd90a26e12a52efb0e1a5ca6) )   /* Audio CPU is a Z80  */

	ROM_REGION( 0x1000, "mcu", 0 )  /* protection NEC D8749 */
	ROM_LOAD( "a-6v.mcu",         0x00000, 0x1000, NO_DUMP )

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "tkni5.bin",    0x000000, 0x10000, CRC(5ed15896) SHA1(87bdddb26934af0b2c4e704e6d85c69a7531aeb1) ) /* 8x8 tiles */

	ROM_REGION( 0x080000, "gfx2", 0 )
	ROM_LOAD( "tkni7.bin",    0x000000, 0x80000, CRC(4b4d4286) SHA1(d386aa223eb288ea829c98d3f39279a75dc66b71) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "tkni6.bin",    0x000000, 0x80000, CRC(f68fafb1) SHA1(aeca38eaea2f6dfc484e48ac1114c0c4abaafb9c) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "tkni9.bin",    0x000000, 0x80000, CRC(d22f4239) SHA1(360a9a821faabe911eef407ef85452d8b706538f) ) /* sprites */
	ROM_LOAD( "tkni8.bin",    0x080000, 0x80000, CRC(4931b184) SHA1(864e827ac109c0ee52a898034c021cd5e92ff000) ) /* sprites */

	ROM_REGION( 0x40000, "oki", 0 ) /* 128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "tkni4.bin",    0x0000, 0x20000, CRC(a7a1dbcf) SHA1(2fee1d9745ce2ab54b0b9cbb6ab2e66ba9677245) ) /* samples */
ROM_END

ROM_START( stratof )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "1.3s",        0x00000, 0x20000, CRC(060822a4) SHA1(82abf6ea64695d2f7b5934ad2487e857648aeecf) )
	ROM_LOAD16_BYTE( "2.4s",        0x00001, 0x20000, CRC(339358fa) SHA1(b662bccc2206ae888ea36f355d44bf98fcd2ee2c) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "a-4b.3",           0x00000, 0x10000, CRC(18655c95) SHA1(8357e0520565a201bb930cadffc759463931ec41) )

	ROM_REGION( 0x1000, "mcu", 0 )  /* protection NEC D8749 */
	ROM_LOAD( "a-6v.mcu",         0x00000, 0x1000, NO_DUMP )

	ROM_REGION( 0x10000, "gfx1", 0 )
	ROM_LOAD( "b-7a.5",           0x00000, 0x10000, CRC(6d2e4bf1) SHA1(edcf96bbcc109da71e3adbb37d119254d3873b29) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "b-1b",  0x00000, 0x80000, CRC(781d1bd2) SHA1(680d91ea02f1e9cb911501f595008f46ad77ded4) )

	ROM_REGION( 0x80000, "gfx3", 0 )
	ROM_LOAD( "b-4b",  0x00000, 0x80000, CRC(89468b84) SHA1(af60fe957c98fa3f00623d420a0941a941f5bc6b) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "b-2m",  0x00000, 0x80000, CRC(5794ec32) SHA1(07e78d8bcb2373da77ef9f8cde6a01f384f8bf7e) )
	ROM_LOAD( "b-1m",  0x80000, 0x80000, CRC(b0de0ded) SHA1(45c74d0c58e3e73c79e587722d9fea9f7ba9cb0a) )

	ROM_REGION( 0x40000, "oki", 0 )
	ROM_LOAD( "a-4a.4", 0x00000, 0x20000, CRC(ef9acdcf) SHA1(8d62a666843f0cb22e8926ae18a961052d4f9ed5) )
ROM_END

ROM_START( raiga )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "a-3s.1",      0x00000, 0x20000, CRC(303c2a6c) SHA1(cd825329fd1f7d87661114f07cc87e43fd34e251) )
	ROM_LOAD16_BYTE( "a-4s.2",      0x00001, 0x20000, CRC(5f31fecb) SHA1(b0c88d260d0108100c157ea92f7defdc3cbb8933) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "a-4b.3",           0x00000, 0x10000, CRC(18655c95) SHA1(8357e0520565a201bb930cadffc759463931ec41) )

	ROM_REGION( 0x1000, "mcu", 0 )  /* protection NEC D8749 */
	ROM_LOAD( "a-6v.mcu",         0x00000, 0x1000, NO_DUMP )

	ROM_REGION( 0x10000, "gfx1", 0 )
	ROM_LOAD( "b-7a.5",           0x00000, 0x10000, CRC(6d2e4bf1) SHA1(edcf96bbcc109da71e3adbb37d119254d3873b29) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "b-1b",  0x00000, 0x80000, CRC(781d1bd2) SHA1(680d91ea02f1e9cb911501f595008f46ad77ded4) )

	ROM_REGION( 0x80000, "gfx3", 0 )
	ROM_LOAD( "b-4b",  0x00000, 0x80000, CRC(89468b84) SHA1(af60fe957c98fa3f00623d420a0941a941f5bc6b) )

	ROM_REGION( 0x100000, "gfx4", 0 )
	ROM_LOAD( "b-2m",  0x00000, 0x80000, CRC(5794ec32) SHA1(07e78d8bcb2373da77ef9f8cde6a01f384f8bf7e) )
	ROM_LOAD( "b-1m",  0x80000, 0x80000, CRC(b0de0ded) SHA1(45c74d0c58e3e73c79e587722d9fea9f7ba9cb0a) )

	ROM_REGION( 0x40000, "oki", 0 )
	ROM_LOAD( "a-4a.4", 0x00000, 0x20000, CRC(ef9acdcf) SHA1(8d62a666843f0cb22e8926ae18a961052d4f9ed5) )
ROM_END

/*
Dragon Bowl
Nics, 1992

PCB Layout
----------

D.B.001 (sticker)
|------------------------------------------------------|
|   1.2R 2.3R               15.5R  17.6R  19.7R  21.8R |
|   6116 3.3Q               14.5Q  16.6Q  18.7Q  20.8Q |
|   Z80        6295     |--------|                     |
|   3569  12MHz         |        |                     |
|7105       PAL         | FPGA   |            6116     |
|                       |        |                     |
|    DIPA               |--------|   22.6M             |
|J          PAL                      6116              |
|A          |-------------|          6116              |
|M          |    68000    |                            |
|M   DIPB   |-------------|                            |
|A            6264  6264                       2018    |
|             4.3H  5.4H                 PAL   2018    |
|                                                      |
|                           |--------|                 |
|        20MHz              |        |                 |
|        6116               | FPGA   |                 |
|        6116               |        |                 |
|        6264   6264        |--------|                 |
|                           7.5B   9.6B   11.7B  13.8B |
|                           6.5A   8.6A   10.7A  12.8A |
|------------------------------------------------------|
Notes:
      68000 : Motorola MC68000P10 CPU running at 10.000MHz [20/2] (DIP64)
      Z80   : Goldstar Z8400B running at 6.000MHz [12/2] (DIP40)
      6295  : Oki M6295 running at 1.000MHz [12/12] (QFP44), sample rate = 1000000 / 132
      3569  : Looks like YM3812 or YM3526 or some other YM compatible YM35xx DIP24 chip.
              Input clock is 4MHz on pin 24 and output clock is 2MHz on pin 23 (tied to DAC)
      7105  : Likely YM3012 compatible DAC (pin 2 has 2MHz clock)
      FPGA  : Unknown FPGA (x2, PLCC84)
      2018  : 2K x8 SRAM (x2, NDIP24)
      6116  : 2K x8 SRAM (x6, DIP24)
      6264  : 8K x8 SRAM (x4, DIP28)
      DIPA/B: 8 position DIP Switches
      VSync : 60Hz
*/

ROM_START( drgnbowl )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "4.3h",         0x00000, 0x20000, CRC(90730008) SHA1(84f0668cf978d99f861cbaeb4b33f7cb1428a648) )
	ROM_LOAD16_BYTE( "5.4h",         0x00001, 0x20000, CRC(193cc915) SHA1(e898f31766eaf515e0787848134b1365e75b32a9) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "1.2r",         0x00000, 0x10000, CRC(d9cbf84a) SHA1(d14d749a41a440a56fea1d836a8d62be65786d68) ) /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "22.6m",        0x00000, 0x10000, CRC(86e41198) SHA1(40201a139a668e6fc441d500f40601c7af934b1d) )  /* 8x8 tiles */

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "6.5a",         0x00000, 0x20000, CRC(b15759f7) SHA1(1710e5ebe8197fdc622ed5c2813257ebe662b7f2) )
	ROM_LOAD( "7.5b",         0x20000, 0x20000, CRC(2541d445) SHA1(a9688cb216bc56fe1b454bc79f967582709991b1) )
	ROM_LOAD( "8.6a",         0x40000, 0x20000, CRC(51a2f5c4) SHA1(dba1278303055b420b128907ba9909e7a39b2df6) )
	ROM_LOAD( "9.6b",         0x60000, 0x20000, CRC(f4c8850f) SHA1(d618c3b8b5d93b9e6fa47b833d8f06a664f63e49) )
	ROM_LOAD( "10.7a",        0x80000, 0x20000, CRC(9e4b3c61) SHA1(5a3739a40d8ffe551262fe42fc36d5a07a59457e) )
	ROM_LOAD( "11.7b",        0xa0000, 0x20000, CRC(0d33d083) SHA1(204889531cce4f7251edfa44f723b43a08c3b28c) )
	ROM_LOAD( "12.8a",        0xc0000, 0x20000, CRC(6c497ad3) SHA1(f0bbf5d7b6efe64c34829104f97b343def705d7f) )
	ROM_LOAD( "13.8b",        0xe0000, 0x20000, CRC(7a84adff) SHA1(86b15842e1fcdb882af6159ff3d44c5806fe3ced) )

	ROM_REGION( 0x100000, "gfx3", ROMREGION_INVERT )
	ROM_LOAD( "15.5r",        0x00000, 0x20000, CRC(7429371c) SHA1(1412312d429ea4bb00db2b8704a7c3d7e14db19b) )
	ROM_LOAD( "14.5q",        0x20000, 0x20000, CRC(4301b97f) SHA1(70614691794a04e0ac1547ba1772ee527fe77ba8) )
	ROM_LOAD( "17.6r",        0x40000, 0x20000, CRC(9088af09) SHA1(8b5090d8a88ad06152030e92acecd76cb2f0f88c) )
	ROM_LOAD( "16.6q",        0x60000, 0x20000, CRC(8ade4e01) SHA1(f02fcc66d1f842ff3861813431942a95de08f654) )
	ROM_LOAD( "19.7r",        0x80000, 0x20000, CRC(5082ceff) SHA1(fad67375b774236b345d3496ce17665947a21201) )
	ROM_LOAD( "18.7q",        0xa0000, 0x20000, CRC(d18a7ffb) SHA1(8ea792dfb8e7c9e6df0fd7596c3972f79b15d860) )
	ROM_LOAD( "21.8r",        0xc0000, 0x20000, CRC(0cee8711) SHA1(5ec071db383a56629a7063d86264bd2bbb6b0036) )
	ROM_LOAD( "20.8q",        0xe0000, 0x20000, CRC(9647e02a) SHA1(97b05716b13dd77f31ac6a08326267ec175115f1) )

	ROM_REGION( 0x40000, "oki", 0 ) /* 2*128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "3.3q",         0x00000, 0x20000, CRC(489c6d0e) SHA1(5a276fad500a760c83a16e0a4cd91d5963ad8089) ) /* samples */
	ROM_LOAD( "2.3r",         0x20000, 0x20000, CRC(7710ce39) SHA1(7a7cf0b4005b000589d0bad380575d625d9d20f7) ) /* samples */
ROM_END


ROM_START( drgnbowla )
	ROM_REGION( 0x40000, "maincpu", 0 ) /* 2*128k for 68000 code */
	ROM_LOAD16_BYTE( "dbowl_4.u4",         0x00000, 0x20000, CRC(58d69235) SHA1(58ab422793787cae5dfffd07d3bbbb7fee48b628) )
	ROM_LOAD16_BYTE( "dbowl_5.u3",         0x00001, 0x20000, CRC(e3176ebb) SHA1(9513a84c016b372fbb17117998e6910bde1f72a2) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "1.2r",         0x00000, 0x10000, CRC(d9cbf84a) SHA1(d14d749a41a440a56fea1d836a8d62be65786d68) ) /* Audio CPU is a Z80  */

	ROM_REGION( 0x010000, "gfx1", 0 )
	ROM_LOAD( "22.6m",        0x00000, 0x10000, CRC(86e41198) SHA1(40201a139a668e6fc441d500f40601c7af934b1d) )  /* 8x8 tiles */

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "6.5a",         0x00000, 0x20000, CRC(b15759f7) SHA1(1710e5ebe8197fdc622ed5c2813257ebe662b7f2) )
	ROM_LOAD( "7.5b",         0x20000, 0x20000, CRC(2541d445) SHA1(a9688cb216bc56fe1b454bc79f967582709991b1) )
	ROM_LOAD( "8.6a",         0x40000, 0x20000, CRC(51a2f5c4) SHA1(dba1278303055b420b128907ba9909e7a39b2df6) )
	ROM_LOAD( "9.6b",         0x60000, 0x20000, CRC(f4c8850f) SHA1(d618c3b8b5d93b9e6fa47b833d8f06a664f63e49) )
	ROM_LOAD( "10.7a",        0x80000, 0x20000, CRC(9e4b3c61) SHA1(5a3739a40d8ffe551262fe42fc36d5a07a59457e) )
	ROM_LOAD( "11.7b",        0xa0000, 0x20000, CRC(0d33d083) SHA1(204889531cce4f7251edfa44f723b43a08c3b28c) )
	ROM_LOAD( "12.8a",        0xc0000, 0x20000, CRC(6c497ad3) SHA1(f0bbf5d7b6efe64c34829104f97b343def705d7f) )
	ROM_LOAD( "13.8b",        0xe0000, 0x20000, CRC(7a84adff) SHA1(86b15842e1fcdb882af6159ff3d44c5806fe3ced) )

	ROM_REGION( 0x100000, "gfx3", ROMREGION_INVERT )
	ROM_LOAD( "15.5r",        0x00000, 0x20000, CRC(7429371c) SHA1(1412312d429ea4bb00db2b8704a7c3d7e14db19b) )
	ROM_LOAD( "14.5q",        0x20000, 0x20000, CRC(4301b97f) SHA1(70614691794a04e0ac1547ba1772ee527fe77ba8) )
	ROM_LOAD( "17.6r",        0x40000, 0x20000, CRC(9088af09) SHA1(8b5090d8a88ad06152030e92acecd76cb2f0f88c) )
	ROM_LOAD( "16.6q",        0x60000, 0x20000, CRC(8ade4e01) SHA1(f02fcc66d1f842ff3861813431942a95de08f654) )
	ROM_LOAD( "19.7r",        0x80000, 0x20000, CRC(5082ceff) SHA1(fad67375b774236b345d3496ce17665947a21201) )
	ROM_LOAD( "18.7q",        0xa0000, 0x20000, CRC(d18a7ffb) SHA1(8ea792dfb8e7c9e6df0fd7596c3972f79b15d860) )
	ROM_LOAD( "21.8r",        0xc0000, 0x20000, CRC(0cee8711) SHA1(5ec071db383a56629a7063d86264bd2bbb6b0036) )
	ROM_LOAD( "20.8q",        0xe0000, 0x20000, CRC(9647e02a) SHA1(97b05716b13dd77f31ac6a08326267ec175115f1) )

	ROM_REGION( 0x40000, "oki", 0 ) /* 2*128k for ADPCM samples - sound chip is OKIM6295 */
	ROM_LOAD( "3.3q",         0x00000, 0x20000, CRC(489c6d0e) SHA1(5a276fad500a760c83a16e0a4cd91d5963ad8089) ) /* samples */
	ROM_LOAD( "2.3r",         0x20000, 0x20000, CRC(7710ce39) SHA1(7a7cf0b4005b000589d0bad380575d625d9d20f7) ) /* samples */
ROM_END


DRIVER_INIT_MEMBER(gaiden_state,shadoww)
{
	/* sprite size Y = sprite size X */
	m_sprite_sizey = 0;
	m_raiga_jumppoints = jumppoints_00;
}

DRIVER_INIT_MEMBER(gaiden_state,wildfang)
{
	/* sprite size Y = sprite size X */
	m_sprite_sizey = 0;
	m_raiga_jumppoints = jumppoints_00;

	m_prot = 0;
	m_jumpcode = 0;
	m_maincpu->space(AS_PROGRAM).install_read_handler(0x07a006, 0x07a007, read16_delegate(FUNC(gaiden_state::wildfang_protection_r),this));
	m_maincpu->space(AS_PROGRAM).install_write_handler(0x07a804, 0x07a805, write16_delegate(FUNC(gaiden_state::wildfang_protection_w),this));
}

DRIVER_INIT_MEMBER(gaiden_state,raiga)
{
	/* sprite size Y independent from sprite size X */
	m_sprite_sizey = 2;
	m_raiga_jumppoints = jumppoints_00;

	m_prot = 0;
	m_jumpcode = 0;
	m_maincpu->space(AS_PROGRAM).install_read_handler(0x07a006, 0x07a007, read16_delegate(FUNC(gaiden_state::raiga_protection_r),this));
	m_maincpu->space(AS_PROGRAM).install_write_handler(0x07a804, 0x07a805, write16_delegate(FUNC(gaiden_state::raiga_protection_w),this));
}

void gaiden_state::descramble_drgnbowl(int descramble_cpu)
{
	int i;
	uint8_t *ROM = memregion("maincpu")->base();
	size_t size = memregion("maincpu")->bytes();

	if (descramble_cpu)
	{
		std::vector<uint8_t> buffer(size);

		memcpy(&buffer[0], ROM, size);
		for( i = 0; i < size; i++ )
		{
			ROM[i] = buffer[BITSWAP24(i,23,22,21,20,
										19,18,17,15,
								16,14,13,12,
								11,10, 9, 8,
									7, 6, 5, 4,
									3, 2, 1, 0)];
		}
	}

	ROM = memregion("gfx2")->base();
	size = memregion("gfx2")->bytes();
	{
		std::vector<uint8_t> buffer(size);

		memcpy(&buffer[0],ROM,size);
		for( i = 0; i < size; i++ )
		{
			ROM[i] = buffer[BITSWAP24(i,23,22,21,20,
										19,18,16,17,
										15,14,13, 4,
											3,12,11,10,
											9, 8, 7, 6,
											5, 2, 1, 0)];
		}
	}
}

DRIVER_INIT_MEMBER(gaiden_state,drgnbowl)
{
	m_raiga_jumppoints = jumppoints_00;

	descramble_drgnbowl(1);
}

DRIVER_INIT_MEMBER(gaiden_state,drgnbowla)
{
	m_raiga_jumppoints = jumppoints_00;

	descramble_drgnbowl(0);
}

void gaiden_state::descramble_mastninj_gfx(uint8_t* src)
{
	int len = 0x80000;

	/*  rearrange gfx */
	{
		std::vector<uint8_t> buffer(len);
		int i;
		for (i = 0;i < len; i++)
		{
			buffer[i] = src[BITSWAP24(i,
			23,22,21,20,
			19,18,17,16,
			15,5,14,13,12,
			11,10,9,8,
			7,6,4,
			3,2,1,0)];
		}
		memcpy(src, &buffer[0], len);
	}

	{
		std::vector<uint8_t> buffer(len);
		int i;
		for (i = 0; i < len; i++)
		{
			buffer[i] = src[BITSWAP24(i,
			23,22,21,20,
			19,18,17,16,
			15,6,14,13,12,
			11,10,9,8,
			7,5,4,
			3,2,1,0)];
		}
		memcpy(src, &buffer[0], len);
	}
}

DRIVER_INIT_MEMBER(gaiden_state,mastninj)
{
	// rearrange the graphic roms into a format that MAME can decode
	descramble_mastninj_gfx(memregion("gfx2")->base());
	descramble_mastninj_gfx(memregion("gfx3")->base());
	DRIVER_INIT_CALL(shadoww);
}

//    YEAR, NAME,      PARENT,   MACHINE,  INPUT,    STATE,        INIT,     MONITOR,COMPANY,   FULLNAME,FLAGS
GAME( 1988, shadoww,   0,        shadoww,  common,   gaiden_state, shadoww,  ROT0,   "Tecmo",   "Shadow Warriors (World, set 1)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, shadowwa,  shadoww,  shadoww,  common,   gaiden_state, shadoww,  ROT0,   "Tecmo",   "Shadow Warriors (World, set 2)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, gaiden,    shadoww,  shadoww,  common,   gaiden_state, shadoww,  ROT0,   "Tecmo",   "Ninja Gaiden (US)",              MACHINE_SUPPORTS_SAVE )
GAME( 1989, ryukendn,  shadoww,  shadoww,  common,   gaiden_state, shadoww,  ROT0,   "Tecmo",   "Ninja Ryukenden (Japan, set 1)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, ryukendna, shadoww,  shadoww,  common,   gaiden_state, shadoww,  ROT0,   "Tecmo",   "Ninja Ryukenden (Japan, set 2)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, mastninj,  shadoww,  mastninj, common,   gaiden_state, mastninj, ROT0,   "bootleg", "Master Ninja (bootleg of Shadow Warriors / Ninja Gaiden)", MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE ) // sprites need fixing, sound and yscroll too. - it is confirmed the curtains don't scroll on the pcb
GAME( 1992, drgnbowl,  0,        drgnbowl, drgnbowl, gaiden_state, drgnbowl, ROT0,   "Nics",    "Dragon Bowl (set 1, encrypted program)",   MACHINE_SUPPORTS_SAVE ) // Dragon Bowl is based on Ninja Gaiden code
GAME( 1992, drgnbowla, drgnbowl, drgnbowl, drgnbowl, gaiden_state, drgnbowla,ROT0,   "Nics",    "Dragon Bowl (set 2, unencrypted program)", MACHINE_SUPPORTS_SAVE )

GAME( 1989, wildfang,  0,        shadoww,  wildfang, gaiden_state, wildfang, ROT0,   "Tecmo",   "Wild Fang / Tecmo Knight", MACHINE_SUPPORTS_SAVE )
GAME( 1989, wildfangs, wildfang, shadoww,  tknight,  gaiden_state, wildfang, ROT0,   "Tecmo",   "Wild Fang",                MACHINE_SUPPORTS_SAVE )
GAME( 1989, tknight,   wildfang, shadoww,  tknight,  gaiden_state, wildfang, ROT0,   "Tecmo",   "Tecmo Knight",             MACHINE_SUPPORTS_SAVE )

GAME( 1991, stratof,   0,        raiga,    raiga,    gaiden_state, raiga,    ROT0,   "Tecmo",   "Raiga - Strato Fighter (US)",    MACHINE_IMPERFECT_GRAPHICS | MACHINE_SUPPORTS_SAVE )
GAME( 1991, raiga,     stratof,  raiga,    raiga,    gaiden_state, raiga,    ROT0,   "Tecmo",   "Raiga - Strato Fighter (Japan)", MACHINE_IMPERFECT_GRAPHICS | MACHINE_SUPPORTS_SAVE )
