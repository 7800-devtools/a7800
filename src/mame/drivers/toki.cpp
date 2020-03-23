// license:BSD-3-Clause
// copyright-holders:Jarek Parchanski
/***************************************************************************

Toki

driver by Jarek Parchanski

Coin inputs are handled by the sound CPU, so they don't work with sound
disabled. Use the service switch instead.

Mametesters bug tokiu056gre - "tokiu: "0000000" is always displayed as the top hiscore during gameplay,
regardless of what it actually is. This does not happen in the other Toki sets."

Notes by bmcphail@vcmame.net, 1/1/2008

Toki stores high score at $60008 in main ram (init code at $ADA, compared with player score at $1A1BA)
Tokiu stores high score at $60010 instead (init code at $B16, equivalent compare code at $1a204), $60008
is used for different purposes in many parts of the code.

Both games feature a common routine ($1cba2 in toki, $1cbfa in tokiu) that prints the high score to screen,
the problem is that the version in Tokiu has not been adjusted for the different high score location and
it reads from the $68008 location instead of $680010.  From analysing the code I'm certain this is a bug
in the original USA version code and not an emulation bug.

TODO
----
Does the bootleg use a 68000 @ 10MHz ? This causes some bad slow-
downs at the floating monkey machine (round 1), so set to 12 MHz
for now. Even at 12 this slowdown still happens a little.

***************************************************************************
Toki
TAD Corporation, 1989

PCB Layout
----------

TOKI-TM 1989 TAD CORPORATION
|------------------------------------------------------------------|
|VOL YM3014   9 14.31818MHz  SG0140 SEI0021BU SEI0021BU SIS6091    |
|HB-41 M6295  8  SEI80BU  82S129    SEI0021BU SEI0021BU 2 SEI0050BU|
|LA4460 YM3812 Z80          SIS6091  SIS6091  SIS6091   1    12MHz |
|              5814 PLHS18P8  UEC-51                     82S135    |
| SEI0100BU    7                         SEI0010BU     SEI0010BU   |
|UEC-02        74LS154                BK2        BK1   SEI0010BU   |
|                                                        PLHS18P8  |
|J             6        4   PLHS18P8 74F841                        |
|A  UEC-01                  PLHS18P8 74F841 SIS6091  SEI0060BU     |
|M  UEC-01     5        3   74F269   74F827    SG0140  SIS6091     |
|M  UEC-01                                                  SIS6091|
|A  UEC-01   58257    58257                                        |
|                                       SIS6091                    |
| DSW1(8)                                                          |
|                                SIS6091                           |
|        |------------------|           SIS6091 SG0140  SIS6091    |
| DSW2(8)|     68000-10     |                    SEI0060BU  SIS6091|
|        |------------------|                    PLHS18P8          |
|                                SG0140             OBJ1 SEI0010BU |
|20MHz    PLHS18P8 PLHS18P8                         OBJ2 SEI0010BU |
|------------------------------------------------------------------|
Notes:
       68000 - Clock 10.000MHz [20/2]
         Z80 - Clock 3.579545MHz [14.31818/4]
       M6295 - Clock 1.000MHz [20/20]
      YM3812 - Clock 3.579545MHz [14.31818/4]
       58257 - 32kx8 SRAM
        5814 - 2kx8 SRAM
    4, 6 & 9 - 128kx8 EPROM/maskROM (i.e. 27C010)
 1,2,3,5 & 7 - 64kx8 EPROM/maskROM (i.e. 27C512)
           8 - 8kx8 EPROM (i.e. 27C64)
     BK1/BK2 - 512kx8 DIP40 mask ROM
   OBJ1/OBJ2 - 512kx8 DIP40 mask ROM
      82S129 - Philips 82S129 1k-bit (256x4) Bipolar PROM at location J3
      82S135 - Philips 82S135 2k-bit (256x8) Bipolar PROM at location B6
      LA4460 - Sanyo LA4460 12W Power AMP
   SEI0100BU - Custom chip marked 'SEI0100BU YM3931' (SDIP64)
     SIS6091 - Unknown QFP80. Probably RAM? (there's no known sprite/BG RAM on the PCB)
      SG0140 - Custom chip marked 'SG0140', actually Toshiba Gate Array TC110G05AN-0012
   SEI0021BU - Custom chip marked 'SEI0021BU', actually Toshiba Gate Array TC17G008AN-0022
     SEI80BU - Custom chip marked 'SEI80BU'
   SEI0060BU - Custom chip marked 'SEI0060BU', actually Toshiba Gate Array TC17G008AN-0024
   SEI0050BU - Custom chip marked 'SEI0050BU M ^ 844 00' ^=triangle symbol (possibly Mitsubishi?)
   SEI0010BU - Custom chip marked 'SEI0010BU', actually Toshiba Gate Array TC17G005AN-0025
    PLHS18P8 - Signetics PLHS18P8AN Programmable logic chip (DIP20)
      UEC-51 - Custom ceramic module, maybe RGB DAC?
       HB-41 - Custom ceramic module, audio DAC/filter
      UEC-01 - Custom ceramic module, I/O
      UEC-02 - Custom ceramic module, drives the coin counters
       VSync - 59.6094Hz
       HSync - 15.31996kHz

***************************************************************************/

#include "emu.h"
#include "includes/toki.h"

#include "cpu/m68000/m68000.h"
#include "cpu/z80/z80.h"
#include "machine/watchdog.h"
#include "sound/3812intf.h"
#include "sound/okim6295.h"
#include "speaker.h"


WRITE16_MEMBER(toki_state::tokib_soundcommand_w)
{
	m_soundlatch->write(space, 0, data & 0xff);
	m_audiocpu->set_input_line(0, HOLD_LINE);
}

READ16_MEMBER(toki_state::pip_r)
{
	return ~0;
}



WRITE_LINE_MEMBER(toki_state::tokib_adpcm_int)
{
	m_msm->data_w(m_msm5205next);
	m_msm5205next >>= 4;

	m_toggle ^= 1;
	if (m_toggle)
		m_audiocpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

WRITE8_MEMBER(toki_state::tokib_adpcm_control_w)
{
	/* the code writes either 2 or 3 in the bottom two bits */
	membank("bank1")->set_entry(data & 1);

	m_msm->reset_w(data & 0x08);
}

WRITE8_MEMBER(toki_state::tokib_adpcm_data_w)
{
	m_msm5205next = data;
}


/*****************************************************************************/

static ADDRESS_MAP_START( toki_map, AS_PROGRAM, 16, toki_state )
	AM_RANGE(0x000000, 0x05ffff) AM_ROM
	AM_RANGE(0x060000, 0x06d7ff) AM_RAM
	AM_RANGE(0x06d800, 0x06dfff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x06e000, 0x06e7ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x06e800, 0x06efff) AM_RAM_WRITE(background1_videoram_w) AM_SHARE("bg1_vram")
	AM_RANGE(0x06f000, 0x06f7ff) AM_RAM_WRITE(background2_videoram_w) AM_SHARE("bg2_vram")
	AM_RANGE(0x06f800, 0x06ffff) AM_RAM_WRITE(foreground_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x080000, 0x08000d) AM_DEVREADWRITE("seibu_sound", seibu_sound_device, main_word_r, main_word_w)
	AM_RANGE(0x0a0000, 0x0a005f) AM_WRITE(toki_control_w) AM_SHARE("scrollram")
	AM_RANGE(0x0c0000, 0x0c0001) AM_READ_PORT("DSW")
	AM_RANGE(0x0c0002, 0x0c0003) AM_READ_PORT("INPUTS")
	AM_RANGE(0x0c0004, 0x0c0005) AM_READ_PORT("SYSTEM")
ADDRESS_MAP_END

/* In the bootleg, sound and sprites are remapped to 0x70000 */
static ADDRESS_MAP_START( tokib_map, AS_PROGRAM, 16, toki_state )
	AM_RANGE(0x000000, 0x05ffff) AM_ROM
	AM_RANGE(0x060000, 0x06dfff) AM_RAM
	AM_RANGE(0x06e000, 0x06e7ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x06e800, 0x06efff) AM_RAM_WRITE(background1_videoram_w) AM_SHARE("bg1_vram")
	AM_RANGE(0x06f000, 0x06f7ff) AM_RAM_WRITE(background2_videoram_w) AM_SHARE("bg2_vram")
	AM_RANGE(0x06f800, 0x06ffff) AM_RAM_WRITE(foreground_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x071000, 0x071001) AM_WRITENOP    /* sprite related? seems another scroll register */
				/* gets written the same value as 75000a (bg2 scrollx) */
	AM_RANGE(0x071804, 0x071807) AM_WRITENOP    /* sprite related, always 01be0100 */
	AM_RANGE(0x07180e, 0x071e45) AM_WRITEONLY AM_SHARE("spriteram")
	AM_RANGE(0x072000, 0x072001) AM_DEVREAD("watchdog", watchdog_timer_device, reset16_r)   /* probably */
	AM_RANGE(0x075000, 0x075001) AM_WRITE(tokib_soundcommand_w)
	AM_RANGE(0x075004, 0x07500b) AM_WRITEONLY AM_SHARE("scrollram")
	AM_RANGE(0x0c0000, 0x0c0001) AM_READ_PORT("DSW")
	AM_RANGE(0x0c0002, 0x0c0003) AM_READ_PORT("INPUTS")
	AM_RANGE(0x0c0004, 0x0c0005) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x0c000e, 0x0c000f) AM_READ(pip_r)  /* sound related, if we return 0 the code writes */
				/* the sound command quickly followed by 0 and the */
				/* sound CPU often misses the command. */
ADDRESS_MAP_END

/*****************************************************************************/

static ADDRESS_MAP_START( toki_audio_map, AS_PROGRAM, 8, toki_state )
	AM_RANGE(0x0000, 0x1fff) AM_DEVREAD("sei80bu", sei80bu_device, data_r)
	AM_RANGE(0x2000, 0x27ff) AM_RAM
	AM_RANGE(0x4000, 0x4000) AM_DEVWRITE("seibu_sound", seibu_sound_device, pending_w)
	AM_RANGE(0x4001, 0x4001) AM_DEVWRITE("seibu_sound", seibu_sound_device, irq_clear_w)
	AM_RANGE(0x4002, 0x4002) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst10_ack_w)
	AM_RANGE(0x4003, 0x4003) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst18_ack_w)
	AM_RANGE(0x4007, 0x4007) AM_DEVWRITE("seibu_sound", seibu_sound_device, bank_w)
	AM_RANGE(0x4008, 0x4009) AM_DEVREADWRITE("seibu_sound", seibu_sound_device, ym_r, ym_w)
	AM_RANGE(0x4010, 0x4011) AM_DEVREAD("seibu_sound", seibu_sound_device, soundlatch_r)
	AM_RANGE(0x4012, 0x4012) AM_DEVREAD("seibu_sound", seibu_sound_device, main_data_pending_r)
	AM_RANGE(0x4013, 0x4013) AM_READ_PORT("COIN")
	AM_RANGE(0x4018, 0x4019) AM_DEVWRITE("seibu_sound", seibu_sound_device, main_data_w)
	AM_RANGE(0x401b, 0x401b) AM_DEVWRITE("seibu_sound", seibu_sound_device, coin_w)
	AM_RANGE(0x6000, 0x6000) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("seibu_bank1")
ADDRESS_MAP_END

static ADDRESS_MAP_START( toki_audio_opcodes_map, AS_OPCODES, 8, toki_state )
	AM_RANGE(0x0000, 0x1fff) AM_DEVREAD("sei80bu", sei80bu_device, opcode_r)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("seibu_bank1")
ADDRESS_MAP_END

static ADDRESS_MAP_START( jujuba_audio_map, AS_PROGRAM, 8, toki_state )
	AM_RANGE(0x0000, 0x1fff) AM_READ(jujuba_z80_data_decrypt)
	AM_RANGE(0x2000, 0x27ff) AM_RAM
	AM_RANGE(0x4000, 0x4000) AM_DEVWRITE("seibu_sound", seibu_sound_device, pending_w)
	AM_RANGE(0x4001, 0x4001) AM_DEVWRITE("seibu_sound", seibu_sound_device, irq_clear_w)
	AM_RANGE(0x4002, 0x4002) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst10_ack_w)
	AM_RANGE(0x4003, 0x4003) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst18_ack_w)
	AM_RANGE(0x4007, 0x4007) AM_DEVWRITE("seibu_sound", seibu_sound_device, bank_w)
	AM_RANGE(0x4008, 0x4009) AM_DEVREADWRITE("seibu_sound", seibu_sound_device, ym_r, ym_w)
	AM_RANGE(0x4010, 0x4011) AM_DEVREAD("seibu_sound", seibu_sound_device, soundlatch_r)
	AM_RANGE(0x4012, 0x4012) AM_DEVREAD("seibu_sound", seibu_sound_device, main_data_pending_r)
	AM_RANGE(0x4013, 0x4013) AM_READ_PORT("COIN")
	AM_RANGE(0x4018, 0x4019) AM_DEVWRITE("seibu_sound", seibu_sound_device, main_data_w)
	AM_RANGE(0x401b, 0x401b) AM_DEVWRITE("seibu_sound", seibu_sound_device, coin_w)
	AM_RANGE(0x6000, 0x6000) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("seibu_bank1")
ADDRESS_MAP_END

static ADDRESS_MAP_START( jujuba_audio_opcodes_map, AS_OPCODES, 8, toki_state )
	AM_RANGE(0x0000, 0x1fff) AM_ROM AM_REGION("audiocpu", 0)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("seibu_bank1")
ADDRESS_MAP_END

READ8_MEMBER(toki_state::jujuba_z80_data_decrypt)
{
	return m_audiocpu_rom[offset] ^ 0x55;
}

static ADDRESS_MAP_START( tokib_audio_map, AS_PROGRAM, 8, toki_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_ROMBANK("bank1")
	AM_RANGE(0xe000, 0xe000) AM_WRITE(tokib_adpcm_control_w) /* MSM5205 + ROM bank */
	AM_RANGE(0xe400, 0xe400) AM_WRITE(tokib_adpcm_data_w)
	AM_RANGE(0xec00, 0xec01) AM_MIRROR(0x0008) AM_DEVREADWRITE("ymsnd", ym3812_device, read, write)
	AM_RANGE(0xf000, 0xf7ff) AM_RAM
	AM_RANGE(0xf800, 0xf800) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
ADDRESS_MAP_END

/*****************************************************************************/

static INPUT_PORTS_START( toki )
	SEIBU_COIN_INPUTS   /* coin inputs read through sound cpu */

	PORT_START("DSW")
	PORT_DIPNAME( 0x001f, 0x001f, DEF_STR( Coinage ) ) PORT_DIPLOCATION("SW1:1,2,3,4,5")
	PORT_DIPSETTING(      0x0015, DEF_STR( 6C_1C ) )
	PORT_DIPSETTING(      0x0017, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0019, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x001b, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0003, DEF_STR( 8C_3C ) )
	PORT_DIPSETTING(      0x001d, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x0005, DEF_STR( 5C_3C ) )
	PORT_DIPSETTING(      0x0007, DEF_STR( 3C_2C ) )
	PORT_DIPSETTING(      0x001f, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x0009, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(      0x0013, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0011, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x000f, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(      0x000d, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(      0x000b, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(      0x001e, "A 1/1 B 1/2" )
	PORT_DIPSETTING(      0x0014, "A 2/1 B 1/3" )
	PORT_DIPSETTING(      0x000a, "A 3/1 B 1/5" )
	PORT_DIPSETTING(      0x0000, "A 5/1 B 1/6" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Free_Play ) )
	PORT_DIPNAME( 0x0020, 0x0000, "Joysticks" ) PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(      0x0020, "1" )
	PORT_DIPSETTING(      0x0000, "2" )
	PORT_DIPNAME( 0x0040, 0x0040, DEF_STR( Cabinet ) ) PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(      0x0040, DEF_STR( Upright ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Flip_Screen ) ) PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0300, 0x0300, DEF_STR( Lives ) )  PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(      0x0200, "2" )
	PORT_DIPSETTING(      0x0300, "3" )
	PORT_DIPSETTING(      0x0100, "5" )
	PORT_DIPSETTING(      0x0000, "Infinite (Cheat)")
	PORT_DIPNAME( 0x0c00, 0x0c00, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SW2:3,4")
	PORT_DIPSETTING(      0x0800, "50000 150000" )
	PORT_DIPSETTING(      0x0000, "70000 140000 210000" )
	PORT_DIPSETTING(      0x0c00, "70000" )
	PORT_DIPSETTING(      0x0400, "100000 200000" )
	PORT_DIPNAME( 0x3000, 0x3000, DEF_STR( Difficulty ) ) PORT_DIPLOCATION("SW2:5,6")
	PORT_DIPSETTING(      0x2000, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x3000, DEF_STR( Medium ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SW2:7")
	PORT_DIPSETTING(      0x0000, DEF_STR( No ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION("SW2:8")
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( On ) )

	PORT_START("INPUTS")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x0002, IP_ACTIVE_LOW,  IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x0004, IP_ACTIVE_LOW,  IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x0008, IP_ACTIVE_LOW,  IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x0010, IP_ACTIVE_LOW,  IPT_BUTTON1 )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW,  IPT_BUTTON2 )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW,  IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW,  IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW,  IPT_JOYSTICK_UP ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x0200, IP_ACTIVE_LOW,  IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x0400, IP_ACTIVE_LOW,  IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x0800, IP_ACTIVE_LOW,  IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x1000, IP_ACTIVE_LOW,  IPT_BUTTON1 ) PORT_COCKTAIL
	PORT_BIT( 0x2000, IP_ACTIVE_LOW,  IPT_BUTTON2 ) PORT_COCKTAIL
	PORT_BIT( 0x4000, IP_ACTIVE_LOW,  IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW,  IPT_UNKNOWN )

	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

static INPUT_PORTS_START( tokib )
	PORT_START("DSW")
	PORT_DIPNAME( 0x001f, 0x001f, DEF_STR( Coinage ) )
	PORT_DIPSETTING(      0x0015, DEF_STR( 6C_1C ) )
	PORT_DIPSETTING(      0x0017, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0019, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x001b, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0003, DEF_STR( 8C_3C ) )
	PORT_DIPSETTING(      0x001d, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x0005, DEF_STR( 5C_3C ) )
	PORT_DIPSETTING(      0x0007, DEF_STR( 3C_2C ) )
	PORT_DIPSETTING(      0x001f, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x0009, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(      0x0013, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0011, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x000f, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(      0x000d, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(      0x000b, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(      0x001e, "A 1/1 B 1/2" )
	PORT_DIPSETTING(      0x0014, "A 2/1 B 1/3" )
	PORT_DIPSETTING(      0x000a, "A 3/1 B 1/5" )
	PORT_DIPSETTING(      0x0000, "A 5/1 B 1/6" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Free_Play ) )
	PORT_DIPNAME( 0x0020, 0x0000, "Joysticks" )
	PORT_DIPSETTING(      0x0020, "1" )
	PORT_DIPSETTING(      0x0000, "2" )
	PORT_DIPNAME( 0x0040, 0x0040, DEF_STR( Cabinet ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( Upright ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Flip_Screen ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0300, 0x0300, DEF_STR( Lives ) )
	PORT_DIPSETTING(      0x0200, "2" )
	PORT_DIPSETTING(      0x0300, "3" )
	PORT_DIPSETTING(      0x0100, "5" )
	PORT_DIPSETTING(      0x0000, "Infinite (Cheat)")
	PORT_DIPNAME( 0x0c00, 0x0c00, DEF_STR( Bonus_Life ) )
	PORT_DIPSETTING(      0x0800, "50000 150000" )
	PORT_DIPSETTING(      0x0000, "70000 140000 210000" )
	PORT_DIPSETTING(      0x0c00, "70000" )
	PORT_DIPSETTING(      0x0400, "100000 200000" )
	PORT_DIPNAME( 0x3000, 0x3000, DEF_STR( Difficulty ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x3000, DEF_STR( Medium ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Allow_Continue ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( No ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Demo_Sounds ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( On ) )

	PORT_START("INPUTS")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x0002, IP_ACTIVE_LOW,  IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x0004, IP_ACTIVE_LOW,  IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x0008, IP_ACTIVE_LOW,  IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x0010, IP_ACTIVE_LOW,  IPT_BUTTON1 )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW,  IPT_BUTTON2 )
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW,  IPT_JOYSTICK_UP ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x0200, IP_ACTIVE_LOW,  IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x0400, IP_ACTIVE_LOW,  IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x0800, IP_ACTIVE_LOW,  IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x1000, IP_ACTIVE_LOW,  IPT_BUTTON1 ) PORT_COCKTAIL
	PORT_BIT( 0x2000, IP_ACTIVE_LOW,  IPT_BUTTON2 ) PORT_COCKTAIL
	PORT_BIT( 0x4000, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_LOW,  IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW,  IPT_COIN2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW,  IPT_SERVICE1 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW,  IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW,  IPT_START2 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )
INPUT_PORTS_END


/*****************************************************************************/

static const gfx_layout toki_charlayout =
{
	8,8,
	RGN_FRAC(1,2),
	4,
	{ RGN_FRAC(1,2), RGN_FRAC(1,2)+4, 0, 4 },
	{ 3, 2, 1, 0, 8+3, 8+2, 8+1, 8+0 },
	{ 0*16, 1*16, 2*16, 3*16, 4*16, 5*16, 6*16, 7*16 },
	16*8
};

static const gfx_layout toki_tilelayout =
{
	16,16,
	RGN_FRAC(1,1),
	4,
	{ 2*4, 3*4, 0*4, 1*4 },
	{ 3, 2, 1, 0, 16+3, 16+2, 16+1, 16+0,
			64*8+3, 64*8+2, 64*8+1, 64*8+0, 64*8+16+3, 64*8+16+2, 64*8+16+1, 64*8+16+0 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
			8*32, 9*32, 10*32, 11*32, 12*32, 13*32, 14*32, 15*32 },
	128*8
};

static GFXDECODE_START( toki )
	GFXDECODE_ENTRY( "gfx1", 0, toki_charlayout, 16*16, 16 )
	GFXDECODE_ENTRY( "gfx2", 0, toki_tilelayout,  0*16, 16 )
	GFXDECODE_ENTRY( "gfx3", 0, toki_tilelayout, 32*16, 16 )
	GFXDECODE_ENTRY( "gfx4", 0, toki_tilelayout, 48*16, 16 )
GFXDECODE_END

static const gfx_layout tokib_tilelayout =
{
	16,16,  /* 16 by 16 */
	4096,   /* 4096 characters */
	4,  /* 4 bits per pixel */
	{ 4096*16*16*3,4096*16*16*2,4096*16*16*1,4096*16*16*0 },    /* planes */
	{ 0, 1, 2, 3, 4, 5, 6, 7,
		0x8000*8+0, 0x8000*8+1, 0x8000*8+2, 0x8000*8+3, 0x8000*8+4,
		0x8000*8+5, 0x8000*8+6, 0x8000*8+7 },           /* x bit */
	{
		0,8,16,24,32,40,48,56,
		0x10000*8+ 0, 0x10000*8+ 8, 0x10000*8+16, 0x10000*8+24, 0x10000*8+32,
		0x10000*8+40, 0x10000*8+48, 0x10000*8+56 },         /* y bit */
	8*8
};

static const gfx_layout tokib_spriteslayout =
{
	16,16,
	RGN_FRAC(1,4),
	4,
	{ RGN_FRAC(3,4), RGN_FRAC(2,4), RGN_FRAC(1,4), RGN_FRAC(0,4) },
	{    0,     1,     2,     3,     4,     5,     6,     7,
		128+0, 128+1, 128+2, 128+3, 128+4, 128+5, 128+6, 128+7 },
	{ 0,8,16,24,32,40,48,56,64,72,80,88,96,104,112,120 },
	16*16
};

static GFXDECODE_START( tokib )
	GFXDECODE_ENTRY( "gfx1", 0, gfx_8x8x4_planar,   16*16, 16 )
	GFXDECODE_ENTRY( "gfx2", 0, tokib_spriteslayout,  0*16, 16 )
	GFXDECODE_ENTRY( "gfx3", 0, tokib_tilelayout,   32*16, 16 )
	GFXDECODE_ENTRY( "gfx4", 0, tokib_tilelayout,   48*16, 16 )
GFXDECODE_END


/*****************************************************************************/

static MACHINE_CONFIG_START( toki ) /* KOYO 20.000MHz near the cpu */

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000,XTAL_20MHz /2)   /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(toki_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", toki_state,  irq1_line_hold)/* VBL */

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_14_31818MHz/4) // verified on pcb
	MCFG_CPU_PROGRAM_MAP(toki_audio_map)
	MCFG_CPU_DECRYPTED_OPCODES_MAP(toki_audio_opcodes_map)

	MCFG_DEVICE_ADD("sei80bu", SEI80BU, 0)
	MCFG_DEVICE_ROM("audiocpu")

	/* video hardware */
	MCFG_BUFFERED_SPRITERAM16_ADD("spriteram")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(59.61)    /* verified on pcb */
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)  /* verified */
	MCFG_SCREEN_UPDATE_DRIVER(toki_state, screen_update_toki)
	MCFG_SCREEN_VBLANK_CALLBACK(DEVWRITELINE("spriteram", buffered_spriteram16_device, vblank_copy_rising))
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", toki)
	MCFG_PALETTE_ADD("palette", 1024)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ymsnd", YM3812, XTAL_14_31818MHz/4)
	MCFG_YM3812_IRQ_HANDLER(DEVWRITELINE("seibu_sound", seibu_sound_device, fm_irqhandler))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_OKIM6295_ADD("oki", XTAL_12MHz/12, PIN7_HIGH) // verified on pcb
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_DEVICE_ADD("seibu_sound", SEIBU_SOUND, 0)
	MCFG_SEIBU_SOUND_CPU("audiocpu")
	MCFG_SEIBU_SOUND_ROMBANK("seibu_bank1")
	MCFG_SEIBU_SOUND_YM_READ_CB(DEVREAD8("ymsnd", ym3812_device, read))
	MCFG_SEIBU_SOUND_YM_WRITE_CB(DEVWRITE8("ymsnd", ym3812_device, write))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( jujuba, toki )
	MCFG_DEVICE_REMOVE("sei80bu")

	MCFG_DEVICE_MODIFY("audiocpu")
	MCFG_CPU_PROGRAM_MAP(jujuba_audio_map)
	MCFG_CPU_DECRYPTED_OPCODES_MAP(jujuba_audio_opcodes_map)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( tokib )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, 10000000)   /* 10MHz causes bad slowdowns with monkey machine rd1, but is correct, 20Mhz XTAL */
	MCFG_CPU_PROGRAM_MAP(tokib_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", toki_state,  irq6_line_hold)/* VBL (could be level1, same vector) */

	MCFG_CPU_ADD("audiocpu", Z80, 4000000)  /* verified with PCB */
	MCFG_CPU_PROGRAM_MAP(tokib_audio_map)

	MCFG_WATCHDOG_ADD("watchdog")

	/* video hardware */
	MCFG_BUFFERED_SPRITERAM16_ADD("spriteram")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)  /* verified */
	MCFG_SCREEN_UPDATE_DRIVER(toki_state, screen_update_tokib)
	MCFG_SCREEN_VBLANK_CALLBACK(DEVWRITELINE("spriteram", buffered_spriteram16_device, vblank_copy_rising))
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", tokib)
	MCFG_PALETTE_ADD("palette", 1024)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ymsnd", YM3812, 3579545)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_SOUND_ADD("msm", MSM5205, 384000)
	MCFG_MSM5205_VCLK_CB(WRITELINE(toki_state, tokib_adpcm_int)) /* interrupt function */
	MCFG_MSM5205_PRESCALER_SELECTOR(S96_4B)  /* 4KHz               */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.60)
MACHINE_CONFIG_END



/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( toki )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "6e.m10",   0x00000, 0x20000, CRC(94015d91) SHA1(8b8d7c589eff038467f55e81ffd450f726c5a8b5) )
	ROM_LOAD16_BYTE( "4e.k10",   0x00001, 0x20000, CRC(531bd3ef) SHA1(2e561f92f5c5f2da16c4791274ccbd421b9b0a05) )
	ROM_LOAD16_BYTE( "5.m12",    0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "3.k12",    0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "8.m3",   0x00000, 0x02000, CRC(6c87c4c5) SHA1(d76822bcde3d42afae72a0945b6acbf3c6a1d955) )  /* encrypted */
	ROM_LOAD( "7.m7",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )  /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "1.c5",   0x000000, 0x10000, CRC(8aa964a2) SHA1(875129bdd5f699ee30a98160718603a3bc958d84) )   /* chars */
	ROM_LOAD( "2.c3",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "toki_obj1.c20",   0x000000, 0x80000, CRC(a27a80ba) SHA1(3dd3b6b0ace6ca6653603bea952b828b154a2223) )   /* sprites */
	ROM_LOAD( "toki_obj2.c22",   0x080000, 0x80000, CRC(fa687718) SHA1(f194b742399d8124d97cfa3d59beb980c36cfb3c) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki_bk1.cd8",   0x000000, 0x80000, CRC(fdaa5f4b) SHA1(ea850361bc8274639e8433bd2a5307fd3a0c9a24) )   /* tiles 1 */

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki_bk2.ef8",   0x000000, 0x80000, CRC(d86ac664) SHA1(bcb64d8e7ad29b8201ebbada1f858075eb8a0f1d) )   /* tiles 2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "9.m1",   0x00000, 0x20000, CRC(ae7a6b8b) SHA1(1d410f91354ffd1774896b2e64f20a2043607805) )

	ROM_REGION( 0x0200, "proms", 0 ) /* video-related */
	ROM_LOAD( "PROM26.B6",      0x0000, 0x0100, CRC(ea6312c6) SHA1(44e2ae948cb79884a3acd8d7d3ff1c9e31562e3e) )
	ROM_LOAD( "PROM27.J3",      0x0100, 0x0100, CRC(e616ae85) SHA1(49614f87615f1a608eeb90bc68d5fc6d9109a565) )
ROM_END


ROM_START( tokip )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "6 10-M",   0x00000, 0x20000, CRC(91b554a3) SHA1(ab003e82552eba381099eb2d00577f952cad42f7) ) // different
	ROM_LOAD16_BYTE( "4 10-K",   0x00001, 0x20000, CRC(404220f7) SHA1(f614692d05f1280cbe801fe0486a611f38b5e866) ) // different
	ROM_LOAD16_BYTE( "5 12-M",   0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "3 12-K",   0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "8 3-M",   0x00000, 0x02000, CRC(6c87c4c5) SHA1(d76822bcde3d42afae72a0945b6acbf3c6a1d955) )  /* encrypted */
	ROM_LOAD( "7 7-M",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )  /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "1 5-C",   0x000000, 0x10000, CRC(fd0ff303) SHA1(e861b8efd7b3050b95a7d9ff1732bb9641e4dbcc) )   /* chars */ // different
	ROM_LOAD( "2 3-C",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD16_BYTE( "OBJ 1-0.ROM10",  0x00000, 0x20000, CRC(a027bd8e) SHA1(33cc4ae75332ab35df1c03f74db8cb17f2749ead) )
	ROM_LOAD16_BYTE( "OBJ 1-1.ROM9",   0x00001, 0x20000, CRC(43a767ea) SHA1(bfc879ff714828f7a1b8f784db8728c91287ed20) )
	ROM_LOAD16_BYTE( "OBJ 1-2.ROM12",  0x40000, 0x20000, CRC(1aecc9d8) SHA1(e7a79783e71de472f07761f9dc71f2a78e629676) )
	ROM_LOAD16_BYTE( "OBJ 1-3.ROM11",  0x40001, 0x20000, CRC(d65c0c6d) SHA1(6b895ce06dae1ecc21c64993defbb3be6b6f8ac2) )
	ROM_LOAD16_BYTE( "OBJ 2-0.ROM14",  0x80000, 0x20000, CRC(cedaccaf) SHA1(82f135c9f6a51e49df543e370861918d582a7923) )
	ROM_LOAD16_BYTE( "OBJ 2-1.ROM13",  0x80001, 0x20000, CRC(013f539b) SHA1(d62c048a95b9c331cedc5343f70947bb50e49c87) )
	ROM_LOAD16_BYTE( "OBJ 2-2.ROM16",  0xc0000, 0x20000, CRC(6a8e6e22) SHA1(a6144201e9a18aa46f65957694653a40071d92d4) )
	ROM_LOAD16_BYTE( "OBJ 2-3.ROM15",  0xc0001, 0x20000, CRC(25d9a16c) SHA1(059d1e2e874bb41f8ef576e0cf33bdbffb57ddc0) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD16_BYTE( "BACK 1-0.ROM5",  0x00000, 0x20000, CRC(fac7e32f) SHA1(13f789c209aa6a6866dfc5a83ca68d83271b12c6) )
	ROM_LOAD16_BYTE( "BACK 1-1.ROM6",  0x00001, 0x20000 ,CRC(ee1135d6) SHA1(299bb3f82d6ded4f401fb407e298842a47a45b1d) )
	ROM_LOAD16_BYTE( "BACK 1-2.ROM7",  0x40000, 0x20000, CRC(78db8d57) SHA1(a03bb854205c410c05d9a82f20354370c0af0bda) )
	ROM_LOAD16_BYTE( "BACK 1-3.ROM8",  0x40001, 0x20000, CRC(d719de71) SHA1(decbb5213d97f75b80ae74e4ccf2ff465d1dfad9) )

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD16_BYTE( "BACK 2-0.ROM1",  0x00000, 0x20000, CRC(949d8025) SHA1(919821647d1bfd0b5b35afcb1c76fddc51a74854))
	ROM_LOAD16_BYTE( "BACK 2-1.ROM2",  0x00001, 0x20000, CRC(4b28b4b4) SHA1(22e5d9098069833ab1dcc89abe07f9ade1b00459) )
	ROM_LOAD16_BYTE( "BACK 2-2.ROM3",  0x40000, 0x20000, CRC(1aa9a5cf) SHA1(305101589f6f56584c8147456dbb4360eaa31fef) )
	ROM_LOAD16_BYTE( "BACK 2-3.ROM4",  0x40001, 0x20000, CRC(6759571f) SHA1(bff3a73ed33c236b38425570f3eb0bbf9a3ca84c) )

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "9 1-M",   0x00000, 0x20000, CRC(ae7a6b8b) SHA1(1d410f91354ffd1774896b2e64f20a2043607805) ) //

	ROM_REGION( 0x0200, "proms", 0 ) /* video-related */
	ROM_LOAD( "PROM26.B6",      0x0000, 0x0100, CRC(ea6312c6) SHA1(44e2ae948cb79884a3acd8d7d3ff1c9e31562e3e) )
	ROM_LOAD( "PROM27.J3",      0x0100, 0x0100, CRC(e616ae85) SHA1(49614f87615f1a608eeb90bc68d5fc6d9109a565) )
ROM_END

ROM_START( tokia )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "6.m10",    0x00000, 0x20000, CRC(03d726b1) SHA1(bbe3a1ea1943cd73b821b3de4d5bf3dfbffd2168) )
	ROM_LOAD16_BYTE( "4c.k10",   0x00001, 0x20000, CRC(b2c345c5) SHA1(ff8ff31551e835e29192d7ddd3e1601968b3e2c5) )
	ROM_LOAD16_BYTE( "5.m12",    0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "3.k12",    0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "8.m3",   0x00000, 0x02000, CRC(6c87c4c5) SHA1(d76822bcde3d42afae72a0945b6acbf3c6a1d955) )  /* encrypted */
	ROM_LOAD( "7.m7",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )  /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "1.c5",   0x000000, 0x10000, CRC(8aa964a2) SHA1(875129bdd5f699ee30a98160718603a3bc958d84) )   /* chars */
	ROM_LOAD( "2.c3",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "toki_obj1.c20",   0x000000, 0x80000, CRC(a27a80ba) SHA1(3dd3b6b0ace6ca6653603bea952b828b154a2223) )   /* sprites */
	ROM_LOAD( "toki_obj2.c22",   0x080000, 0x80000, CRC(fa687718) SHA1(f194b742399d8124d97cfa3d59beb980c36cfb3c) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki_bk1.cd8",   0x000000, 0x80000, CRC(fdaa5f4b) SHA1(ea850361bc8274639e8433bd2a5307fd3a0c9a24) )   /* tiles 1 */

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki_bk2.ef8",   0x000000, 0x80000, CRC(d86ac664) SHA1(bcb64d8e7ad29b8201ebbada1f858075eb8a0f1d) )   /* tiles 2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "9.m1",   0x00000, 0x20000, CRC(ae7a6b8b) SHA1(1d410f91354ffd1774896b2e64f20a2043607805) )

	ROM_REGION( 0x0200, "proms", 0 ) /* video-related */
	ROM_LOAD( "PROM26.B6",      0x0000, 0x0100, CRC(ea6312c6) SHA1(44e2ae948cb79884a3acd8d7d3ff1c9e31562e3e) )
	ROM_LOAD( "PROM27.J3",      0x0100, 0x0100, CRC(e616ae85) SHA1(49614f87615f1a608eeb90bc68d5fc6d9109a565) )
ROM_END

ROM_START( tokiua )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "6.m10",    0x00000, 0x20000, CRC(03d726b1) SHA1(bbe3a1ea1943cd73b821b3de4d5bf3dfbffd2168) )
	ROM_LOAD16_BYTE( "4u.k10",   0x00001, 0x20000, CRC(ca2f50d9) SHA1(e2660a9627850fa39469804a3ff563caedd0782b) )
	ROM_LOAD16_BYTE( "5.m12",    0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "3.k12",    0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "8.m3",   0x00000, 0x02000, CRC(6c87c4c5) SHA1(d76822bcde3d42afae72a0945b6acbf3c6a1d955) )  /* encrypted */
	ROM_LOAD( "7.m7",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )  /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "1.c5",   0x000000, 0x10000, CRC(8aa964a2) SHA1(875129bdd5f699ee30a98160718603a3bc958d84) )   /* chars */
	ROM_LOAD( "2.c3",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "toki_obj1.c20",   0x000000, 0x80000, CRC(a27a80ba) SHA1(3dd3b6b0ace6ca6653603bea952b828b154a2223) )   /* sprites */
	ROM_LOAD( "toki_obj2.c22",   0x080000, 0x80000, CRC(fa687718) SHA1(f194b742399d8124d97cfa3d59beb980c36cfb3c) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki_bk1.cd8",   0x000000, 0x80000, CRC(fdaa5f4b) SHA1(ea850361bc8274639e8433bd2a5307fd3a0c9a24) )   /* tiles 1 */

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki_bk2.ef8",   0x000000, 0x80000, CRC(d86ac664) SHA1(bcb64d8e7ad29b8201ebbada1f858075eb8a0f1d) )   /* tiles 2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "9.m1",   0x00000, 0x20000, CRC(ae7a6b8b) SHA1(1d410f91354ffd1774896b2e64f20a2043607805) )

	ROM_REGION( 0x0200, "proms", 0 ) /* video-related */
	ROM_LOAD( "PROM26.B6",      0x0000, 0x0100, CRC(ea6312c6) SHA1(44e2ae948cb79884a3acd8d7d3ff1c9e31562e3e) )
	ROM_LOAD( "PROM27.J3",      0x0100, 0x0100, CRC(e616ae85) SHA1(49614f87615f1a608eeb90bc68d5fc6d9109a565) )
ROM_END


ROM_START( tokiu )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "6b.10m",   0x00000, 0x20000, CRC(3674d9fe) SHA1(7c610bee23b0f7e6a9e3d5d72d6084e025eb89ec) )
	ROM_LOAD16_BYTE( "14.10k",   0x00001, 0x20000, CRC(bfdd48af) SHA1(3e48375019471a51f0c00d3444b0c1d37d2f8e92) )
	ROM_LOAD16_BYTE( "5.m12",    0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "3.k12",    0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "8.m3",   0x00000, 0x02000, CRC(6c87c4c5) SHA1(d76822bcde3d42afae72a0945b6acbf3c6a1d955) )  /* encrypted */
	ROM_LOAD( "7.m7",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )  /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "1.c5",   0x000000, 0x10000, CRC(8aa964a2) SHA1(875129bdd5f699ee30a98160718603a3bc958d84) )   /* chars */
	ROM_LOAD( "2.c3",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "toki_obj1.c20",   0x000000, 0x80000, CRC(a27a80ba) SHA1(3dd3b6b0ace6ca6653603bea952b828b154a2223) )   /* sprites */
	ROM_LOAD( "toki_obj2.c22",   0x080000, 0x80000, CRC(fa687718) SHA1(f194b742399d8124d97cfa3d59beb980c36cfb3c) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki_bk1.cd8",   0x000000, 0x80000, CRC(fdaa5f4b) SHA1(ea850361bc8274639e8433bd2a5307fd3a0c9a24) )   /* tiles 1 */

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki_bk2.ef8",   0x000000, 0x80000, CRC(d86ac664) SHA1(bcb64d8e7ad29b8201ebbada1f858075eb8a0f1d) )   /* tiles 2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "9.m1",   0x00000, 0x20000, CRC(ae7a6b8b) SHA1(1d410f91354ffd1774896b2e64f20a2043607805) )

	ROM_REGION( 0x0200, "proms", 0 ) /* video-related */
	ROM_LOAD( "PROM26.B6",      0x0000, 0x0100, CRC(ea6312c6) SHA1(44e2ae948cb79884a3acd8d7d3ff1c9e31562e3e) )
	ROM_LOAD( "PROM27.J3",      0x0100, 0x0100, CRC(e616ae85) SHA1(49614f87615f1a608eeb90bc68d5fc6d9109a565) )
ROM_END

ROM_START( juju )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "6.m10",   0x00000, 0x20000, CRC(03d726b1) SHA1(bbe3a1ea1943cd73b821b3de4d5bf3dfbffd2168) )
	ROM_LOAD16_BYTE( "4.k10",   0x00001, 0x20000, CRC(54a45e12) SHA1(240538c8b010bb6e1e7fea2ed2fb1d5f9bc64b2b) )
	ROM_LOAD16_BYTE( "5.m12",   0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "3.k12",   0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "8.m3",   0x00000, 0x02000, CRC(6c87c4c5) SHA1(d76822bcde3d42afae72a0945b6acbf3c6a1d955) )  /* encrypted */
	ROM_LOAD( "7.m7",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )  /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "1.c5",   0x000000, 0x10000, CRC(8aa964a2) SHA1(875129bdd5f699ee30a98160718603a3bc958d84) )   /* chars */
	ROM_LOAD( "2.c3",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD( "toki_obj1.c20",   0x000000, 0x80000, CRC(a27a80ba) SHA1(3dd3b6b0ace6ca6653603bea952b828b154a2223) )   /* sprites */
	ROM_LOAD( "toki_obj2.c22",   0x080000, 0x80000, CRC(fa687718) SHA1(f194b742399d8124d97cfa3d59beb980c36cfb3c) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki_bk1.cd8",   0x000000, 0x80000, CRC(fdaa5f4b) SHA1(ea850361bc8274639e8433bd2a5307fd3a0c9a24) )   /* tiles 1 */

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki_bk2.ef8",   0x000000, 0x80000, CRC(d86ac664) SHA1(bcb64d8e7ad29b8201ebbada1f858075eb8a0f1d) )   /* tiles 2 */

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "9.m1",   0x00000, 0x20000, CRC(ae7a6b8b) SHA1(1d410f91354ffd1774896b2e64f20a2043607805) )

	ROM_REGION( 0x0200, "proms", 0 ) /* video-related */
	ROM_LOAD( "PROM26.B6",      0x0000, 0x0100, CRC(ea6312c6) SHA1(44e2ae948cb79884a3acd8d7d3ff1c9e31562e3e) )
	ROM_LOAD( "PROM27.J3",      0x0100, 0x0100, CRC(e616ae85) SHA1(49614f87615f1a608eeb90bc68d5fc6d9109a565) )
ROM_END

ROM_START( jujuba )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "8.19g",   0x20000, 0x10000,  CRC(208fb08a) SHA1(113d3924d738705cb73d137712a23fa25cd4c78c) )
	ROM_LOAD16_BYTE( "5.19e",   0x20001, 0x10000,  CRC(722e5183) SHA1(87b813e818670bad45043db7f692619052987ce8) )
	ROM_LOAD16_BYTE( "9.20g",   0x00000, 0x10000,  CRC(cb82cc33) SHA1(1c774b72d12e84e9e159f66fc151f779dabbdfbd) )
	ROM_LOAD16_BYTE( "6.20e",   0x00001, 0x10000,  CRC(826ab39d) SHA1(dd7696d78deac02890a7c12f6beb04edfd1158b1) )
	ROM_LOAD16_BYTE( "10.21g",  0x40000, 0x10000,  CRC(6c7a3ffe) SHA1(c9a266ef7a5aeaa78b4d645c4df28068bcab96d0) )
	ROM_LOAD16_BYTE( "7.21e",   0x40001, 0x10000,  CRC(b0628230) SHA1(f8ed24ee53efc595e4dae13e2563021322c049e1) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* Z80 code, banked data */
	ROM_LOAD( "3.9c",    0x00000, 0x02000, CRC(808f5e44) SHA1(a72d04367adf428b8f0955ef6269c39eb47eee14) ) /* first 0x2000 is empty */
	ROM_CONTINUE(0x0000,0x6000)
	ROM_LOAD( "4.11c",   0x10000, 0x10000, CRC(a67969c4) SHA1(99781fbb005b6ba4a19a9cc83c8b257a3b425fa6) )   /* banked stuff */

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "5.19h",   0x000000, 0x10000, CRC(8aa964a2) SHA1(875129bdd5f699ee30a98160718603a3bc958d84) )   /* chars */
	ROM_LOAD( "6.20h",   0x010000, 0x10000, CRC(86e87e48) SHA1(29634d8c58ef7195cd0ce166f1b7fae01bbc110b) )

	ROM_REGION( 0x100000, "gfx2", 0 )
	ROM_LOAD16_BYTE( "1.17d",  0x00000, 0x20000, CRC(a027bd8e) SHA1(33cc4ae75332ab35df1c03f74db8cb17f2749ead) )
	ROM_LOAD16_BYTE( "27.17b", 0x00001, 0x20000, CRC(43a767ea) SHA1(bfc879ff714828f7a1b8f784db8728c91287ed20) )
	ROM_LOAD16_BYTE( "2.18d",  0x40000, 0x20000, CRC(1aecc9d8) SHA1(e7a79783e71de472f07761f9dc71f2a78e629676) )
	ROM_LOAD16_BYTE( "28.18b", 0x40001, 0x20000, CRC(d65c0c6d) SHA1(6b895ce06dae1ecc21c64993defbb3be6b6f8ac2) )
	ROM_LOAD16_BYTE( "3.20d",  0x80000, 0x20000, CRC(cedaccaf) SHA1(82f135c9f6a51e49df543e370861918d582a7923) )
	ROM_LOAD16_BYTE( "29.20b", 0x80001, 0x20000, CRC(013f539b) SHA1(d62c048a95b9c331cedc5343f70947bb50e49c87) )
	ROM_LOAD16_BYTE( "4.21d",  0xc0000, 0x20000, CRC(6a8e6e22) SHA1(a6144201e9a18aa46f65957694653a40071d92d4) )
	ROM_LOAD16_BYTE( "30.21b", 0xc0001, 0x20000, CRC(25d9a16c) SHA1(059d1e2e874bb41f8ef576e0cf33bdbffb57ddc0) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD16_BYTE( "11.1j",        0x00001, 0x10000, CRC(6ad15560) SHA1(707a05ac0c61a66ac65c8c3718e5d2b958da9142) )
	ROM_LOAD16_BYTE( "12.2j",        0x20001, 0x10000, CRC(68534844) SHA1(ff4aa635e2221a552f844e30db93c73107a70cf2) )
	ROM_LOAD16_BYTE( "13.4j",        0x40001, 0x10000, CRC(f271be5a) SHA1(c9847439f0c48f6bb710999acc172a2d6fc8d58b) )
	ROM_LOAD16_BYTE( "14.5j",        0x60001, 0x10000, CRC(5d4c187a) SHA1(b2e0e705910fe8fd230de14053513248fd76d054) )
	ROM_LOAD16_BYTE( "19.1l",        0x00000, 0x10000, CRC(10afdf03) SHA1(f1388e0f3b720ef80c395d1aa0dbb17bc3c56975) )
	ROM_LOAD16_BYTE( "20.2l",        0x20000, 0x10000, CRC(2dc54f41) SHA1(cee34fae49a60dd0009b5ed89e098f6cbfc19431) )
	ROM_LOAD16_BYTE( "21.4l",        0x40000, 0x10000, CRC(946862a3) SHA1(398913ccef8bd5242987c516194752ac38e10918) )
	ROM_LOAD16_BYTE( "22.5l",        0x60000, 0x10000, CRC(b45f5608) SHA1(fbf9f748db285f8693e0493d9b449c23cf02748b) )

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD16_BYTE( "15.18j",        0x00001, 0x10000, CRC(cb8b1d31) SHA1(8dc858c4a096d71ee66541d04f8a3acb97565ac8) )
	ROM_LOAD16_BYTE( "16.19j",        0x20001, 0x10000, CRC(81594e0a) SHA1(6390798f0829d69a70a05dbb169b0eb9183cc9a9) )
	ROM_LOAD16_BYTE( "17.20j",        0x40001, 0x10000, CRC(4acd44ce) SHA1(517444d22f252784ad0cb2b8948d86d4db186ae4) )
	ROM_LOAD16_BYTE( "18.21j",        0x60001, 0x10000, CRC(25cfe9c3) SHA1(6e649ce1f48f8e46d79f67cf43bd45d072441c77) )
	ROM_LOAD16_BYTE( "23.18l",        0x00000, 0x10000, CRC(06c8d622) SHA1(cc2a5b255d14e9984fcadfb465aefd3be42d8dd9) )
	ROM_LOAD16_BYTE( "24.19l",        0x20000, 0x10000, CRC(362a0506) SHA1(0a712b2c6200bbf8c01d6fa1fb2032efb1eba295) )
	ROM_LOAD16_BYTE( "25.20l",        0x40000, 0x10000, CRC(be064c4b) SHA1(d777b560942e9f6300aed1bf22a07b381c27a479) )
	ROM_LOAD16_BYTE( "26.21l",        0x60000, 0x10000, CRC(f8b5b38d) SHA1(6ec60cf5259469cc9c4bdd9ffc6c63bc2785b708) )

	ROM_REGION( 0x40000, "oki", 0 ) /* OKI samples */
	ROM_LOAD( "1.6a",   0x00000, 0x10000, CRC(377153ad) SHA1(1c184197b344c2b65b5842f9ba99fab776a9577b) )
	ROM_LOAD( "2.7a",   0x10000, 0x10000, CRC(093ca15d) SHA1(1b298146c9eea93c22c03e63513200b483b86a3f) )
ROM_END

ROM_START( tokib )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "toki.e3",      0x00000, 0x20000, CRC(ae9b3da4) SHA1(14eabbd0b3596528e96e4399dde03f5817eddbaa) )
	ROM_LOAD16_BYTE( "toki.e5",      0x00001, 0x20000, CRC(66a5a1d6) SHA1(9a8330d19234863952b0a5dce3f5ad28fcabaa31) )
	ROM_LOAD16_BYTE( "tokijp.005",   0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "tokijp.003",   0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* 64k for code + banked data */
	ROM_LOAD( "toki.e1",      0x00000, 0x10000, CRC(2832ef75) SHA1(c15dc67a1251230fe79625b582c255678f3714d8) )

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "toki.e21",     0x000000, 0x08000, CRC(bb8cacbd) SHA1(05cdd2efe63de30dec2e5d2948567cee22e82a63) )   /* chars */
	ROM_LOAD( "toki.e13",     0x008000, 0x08000, CRC(052ad275) SHA1(0f4a9c752348cf5fb43d706bacbcd3e5937441e7) )
	ROM_LOAD( "toki.e22",     0x010000, 0x08000, CRC(04dcdc21) SHA1(3b74019d764a13ffc155f154522c6fe60cf1c5ea) )
	ROM_LOAD( "toki.e7",      0x018000, 0x08000, CRC(70729106) SHA1(e343c02d139d20a54e837e65b6a964e202f5811e) )

	ROM_REGION( 0x100000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "toki.e26",     0x000000, 0x20000, CRC(a8ba71fc) SHA1(331d7396b6e862e32bb6a0d62c25fc201203b951) )   /* sprites */
	ROM_LOAD( "toki.e28",     0x020000, 0x20000, CRC(29784948) SHA1(9e17e57e2cb65a0aff61385c6d3a97b52474b6e7) )
	ROM_LOAD( "toki.e34",     0x040000, 0x20000, CRC(e5f6e19b) SHA1(77dc5cf961c8062b86ebeb896ad2075c3bfa2205) )
	ROM_LOAD( "toki.e36",     0x060000, 0x20000, CRC(96e8db8b) SHA1(9a0421fc57af27a8886e35b7a1a873aa06a112af) )
	ROM_LOAD( "toki.e30",     0x080000, 0x20000, CRC(770d2b1b) SHA1(27e57f21b462e36a10ffa2d4384955047b84190c) )
	ROM_LOAD( "toki.e32",     0x0a0000, 0x20000, CRC(c289d246) SHA1(596eda73b073e8fc3053734c780e7e2604fb5ca3) )
	ROM_LOAD( "toki.e38",     0x0c0000, 0x20000, CRC(87f4e7fb) SHA1(07d6bf00b1145a11f3d3f0af4425a3c5baeca3db) )
	ROM_LOAD( "toki.e40",     0x0e0000, 0x20000, CRC(96e87350) SHA1(754947f71261d8358e158fa9c8fcfd242cd58bc3) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki.e23",     0x000000, 0x10000, CRC(feb13d35) SHA1(1b78ce1e48d16e58ad0721b30ab87765ded7d24e) )   /* tiles 1 */
	ROM_LOAD( "toki.e24",     0x010000, 0x10000, CRC(5b365637) SHA1(434775b0614d904beaf40d7e00c1eaf59b704cb1) )
	ROM_LOAD( "toki.e15",     0x020000, 0x10000, CRC(617c32e6) SHA1(a80f93c83a06acf836e638e4ad2453692622015d) )
	ROM_LOAD( "toki.e16",     0x030000, 0x10000, CRC(2a11c0f0) SHA1(f9b1910c4932f5b95e5a9a8e8d5376c7210bcde7) )
	ROM_LOAD( "toki.e17",     0x040000, 0x10000, CRC(fbc3d456) SHA1(dd10455f2e6c415fb5e39fb239904c499b38ca3e) )
	ROM_LOAD( "toki.e18",     0x050000, 0x10000, CRC(4c2a72e1) SHA1(52a31f88e02e1689c2fffbbd86cbccd0bdab7dcc) )
	ROM_LOAD( "toki.e8",      0x060000, 0x10000, CRC(46a1b821) SHA1(74d9762aef3891463dc100d1bc2d4fdc3c1d163f) )
	ROM_LOAD( "toki.e9",      0x070000, 0x10000, CRC(82ce27f6) SHA1(db29396a336098664f48e3c04930b973a6ffe969) )

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki.e25",     0x000000, 0x10000, CRC(63026cad) SHA1(c8f3898985d99f2a61d4e17eba66b5989a23d0d7) )   /* tiles 2 */
	ROM_LOAD( "toki.e20",     0x010000, 0x10000, CRC(a7f2ce26) SHA1(6b12b3bd872112b42d91ce3c0d5bc95c0fc0f5b5) )
	ROM_LOAD( "toki.e11",     0x020000, 0x10000, CRC(48989aa0) SHA1(109c68c9f0966862194226cecc8b269d9307dd25) )
	ROM_LOAD( "toki.e12",     0x030000, 0x10000, CRC(c2ad9342) SHA1(7c9b5c14c8061e1a57797b79677741b1b98e64fa) )
	ROM_LOAD( "toki.e19",     0x040000, 0x10000, CRC(6cd22b18) SHA1(8281cfd46738448b6890c50c64fb72941e169bee) )
	ROM_LOAD( "toki.e14",     0x050000, 0x10000, CRC(859e313a) SHA1(18ac471a72b3ed42ba74456789adbe323f723660) )
	ROM_LOAD( "toki.e10",     0x060000, 0x10000, CRC(e15c1d0f) SHA1(d0d571dd1055d7307379850313216da86b0704e6) )
	ROM_LOAD( "toki.e6",      0x070000, 0x10000, CRC(6f4b878a) SHA1(4560b1e705a0eb9fad7fdc11fadf952ff67eb264) )
ROM_END

/* This had Playmark stickers on all the roms */
ROM_START( jujub )
	ROM_REGION( 0x60000, "maincpu", 0 ) /* 6*64k for 68000 code */
	ROM_LOAD16_BYTE( "jujub_playmark.e3",      0x00000, 0x20000, CRC(b50c73ec) SHA1(64855e3f5ceab39abf45035eeee80ae6dc39a421) )
	ROM_LOAD16_BYTE( "jujub_playmark.e5",      0x00001, 0x20000, CRC(b2812942) SHA1(aec7e08770935cc59a8246544d99b283583e9601) )
	ROM_LOAD16_BYTE( "tokijp.005",   0x40000, 0x10000, CRC(d6a82808) SHA1(9fcd3e97f7eaada5374347383dc8a6cea2378f7f) )
	ROM_LOAD16_BYTE( "tokijp.003",   0x40001, 0x10000, CRC(a01a5b10) SHA1(76d6da114105402aab9dd5167c0c00a0bddc3bba) )

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* 64k for code + banked data */
	ROM_LOAD( "toki.e1",      0x00000, 0x10000, CRC(2832ef75) SHA1(c15dc67a1251230fe79625b582c255678f3714d8) )

	ROM_REGION( 0x020000, "gfx1", 0 )
	ROM_LOAD( "toki.e21",     0x000000, 0x08000, CRC(bb8cacbd) SHA1(05cdd2efe63de30dec2e5d2948567cee22e82a63) )   /* chars */
	ROM_LOAD( "toki.e13",     0x008000, 0x08000, CRC(052ad275) SHA1(0f4a9c752348cf5fb43d706bacbcd3e5937441e7) )
	ROM_LOAD( "toki.e22",     0x010000, 0x08000, CRC(04dcdc21) SHA1(3b74019d764a13ffc155f154522c6fe60cf1c5ea) )
	ROM_LOAD( "toki.e7",      0x018000, 0x08000, CRC(70729106) SHA1(e343c02d139d20a54e837e65b6a964e202f5811e) )

	ROM_REGION( 0x100000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "toki.e26",     0x000000, 0x20000, CRC(a8ba71fc) SHA1(331d7396b6e862e32bb6a0d62c25fc201203b951) )   /* sprites */
	ROM_LOAD( "toki.e28",     0x020000, 0x20000, CRC(29784948) SHA1(9e17e57e2cb65a0aff61385c6d3a97b52474b6e7) )
	ROM_LOAD( "toki.e34",     0x040000, 0x20000, CRC(e5f6e19b) SHA1(77dc5cf961c8062b86ebeb896ad2075c3bfa2205) )
	ROM_LOAD( "toki.e36",     0x060000, 0x20000, CRC(96e8db8b) SHA1(9a0421fc57af27a8886e35b7a1a873aa06a112af) )
	ROM_LOAD( "toki.e30",     0x080000, 0x20000, CRC(770d2b1b) SHA1(27e57f21b462e36a10ffa2d4384955047b84190c) )
	ROM_LOAD( "toki.e32",     0x0a0000, 0x20000, CRC(c289d246) SHA1(596eda73b073e8fc3053734c780e7e2604fb5ca3) )
	ROM_LOAD( "toki.e38",     0x0c0000, 0x20000, CRC(87f4e7fb) SHA1(07d6bf00b1145a11f3d3f0af4425a3c5baeca3db) )
	ROM_LOAD( "toki.e40",     0x0e0000, 0x20000, CRC(96e87350) SHA1(754947f71261d8358e158fa9c8fcfd242cd58bc3) )

	ROM_REGION( 0x080000, "gfx3", 0 )
	ROM_LOAD( "toki.e23",     0x000000, 0x10000, CRC(feb13d35) SHA1(1b78ce1e48d16e58ad0721b30ab87765ded7d24e) )   /* tiles 1 */
	ROM_LOAD( "toki.e24",     0x010000, 0x10000, CRC(5b365637) SHA1(434775b0614d904beaf40d7e00c1eaf59b704cb1) )
	ROM_LOAD( "toki.e15",     0x020000, 0x10000, CRC(617c32e6) SHA1(a80f93c83a06acf836e638e4ad2453692622015d) )
	ROM_LOAD( "toki.e16",     0x030000, 0x10000, CRC(2a11c0f0) SHA1(f9b1910c4932f5b95e5a9a8e8d5376c7210bcde7) )
	ROM_LOAD( "toki.e17",     0x040000, 0x10000, CRC(fbc3d456) SHA1(dd10455f2e6c415fb5e39fb239904c499b38ca3e) )
	ROM_LOAD( "toki.e18",     0x050000, 0x10000, CRC(4c2a72e1) SHA1(52a31f88e02e1689c2fffbbd86cbccd0bdab7dcc) )
	ROM_LOAD( "toki.e8",      0x060000, 0x10000, CRC(46a1b821) SHA1(74d9762aef3891463dc100d1bc2d4fdc3c1d163f) )
	ROM_LOAD( "toki.e9",      0x070000, 0x10000, CRC(82ce27f6) SHA1(db29396a336098664f48e3c04930b973a6ffe969) )

	ROM_REGION( 0x080000, "gfx4", 0 )
	ROM_LOAD( "toki.e25",     0x000000, 0x10000, CRC(63026cad) SHA1(c8f3898985d99f2a61d4e17eba66b5989a23d0d7) )   /* tiles 2 */
	ROM_LOAD( "toki.e20",     0x010000, 0x10000, CRC(a7f2ce26) SHA1(6b12b3bd872112b42d91ce3c0d5bc95c0fc0f5b5) )
	ROM_LOAD( "toki.e11",     0x020000, 0x10000, CRC(48989aa0) SHA1(109c68c9f0966862194226cecc8b269d9307dd25) )
	ROM_LOAD( "toki.e12",     0x030000, 0x10000, CRC(c2ad9342) SHA1(7c9b5c14c8061e1a57797b79677741b1b98e64fa) )
	ROM_LOAD( "toki.e19",     0x040000, 0x10000, CRC(6cd22b18) SHA1(8281cfd46738448b6890c50c64fb72941e169bee) )
	ROM_LOAD( "toki.e14",     0x050000, 0x10000, CRC(859e313a) SHA1(18ac471a72b3ed42ba74456789adbe323f723660) )
	ROM_LOAD( "toki.e10",     0x060000, 0x10000, CRC(e15c1d0f) SHA1(d0d571dd1055d7307379850313216da86b0704e6) )
	ROM_LOAD( "toki.e6",      0x070000, 0x10000, CRC(6f4b878a) SHA1(4560b1e705a0eb9fad7fdc11fadf952ff67eb264) )
ROM_END



DRIVER_INIT_MEMBER(toki_state,toki)
{
	uint8_t *ROM = memregion("oki")->base();
	std::vector<uint8_t> buffer(0x20000);
	int i;

	memcpy(&buffer[0],ROM,0x20000);
	for( i = 0; i < 0x20000; i++ )
	{
		ROM[i] = buffer[BITSWAP24(i,23,22,21,20,19,18,17,16,13,14,15,12,11,10,9,8,7,6,5,4,3,2,1,0)];
	}
}


DRIVER_INIT_MEMBER(toki_state,tokib)
{
	std::vector<uint8_t> temp(65536 * 2);
	int i, offs, len;
	uint8_t *rom;

	/* merge background tile graphics together */
	len = memregion("gfx3")->bytes();
	rom = memregion("gfx3")->base();
	for (offs = 0; offs < len; offs += 0x20000)
	{
		uint8_t *base = &rom[offs];
		memcpy (&temp[0], base, 65536 * 2);
		for (i = 0; i < 16; i++)
		{
			memcpy (&base[0x00000 + i * 0x800], &temp[0x0000 + i * 0x2000], 0x800);
			memcpy (&base[0x10000 + i * 0x800], &temp[0x0800 + i * 0x2000], 0x800);
			memcpy (&base[0x08000 + i * 0x800], &temp[0x1000 + i * 0x2000], 0x800);
			memcpy (&base[0x18000 + i * 0x800], &temp[0x1800 + i * 0x2000], 0x800);
		}
	}
	len = memregion("gfx4")->bytes();
	rom = memregion("gfx4")->base();
	for (offs = 0; offs < len; offs += 0x20000)
	{
		uint8_t *base = &rom[offs];
		memcpy (&temp[0], base, 65536 * 2);
		for (i = 0; i < 16; i++)
		{
			memcpy (&base[0x00000 + i * 0x800], &temp[0x0000 + i * 0x2000], 0x800);
			memcpy (&base[0x10000 + i * 0x800], &temp[0x0800 + i * 0x2000], 0x800);
			memcpy (&base[0x08000 + i * 0x800], &temp[0x1000 + i * 0x2000], 0x800);
			memcpy (&base[0x18000 + i * 0x800], &temp[0x1800 + i * 0x2000], 0x800);
		}
	}

	membank("bank1")->configure_entries(0, 2, memregion("audiocpu")->base() + 0x8000, 0x4000);
	save_item(NAME(m_msm5205next));
	save_item(NAME(m_toggle));
}

DRIVER_INIT_MEMBER(toki_state,jujuba)
{
	/* Program ROMs are bitswapped */
	{
		int i;
		uint16_t *prgrom = (uint16_t*)memregion("maincpu")->base();

		for (i = 0; i < 0x60000/2; i++)
		{
			prgrom[i] = BITSWAP16(prgrom[i],15,12,13,14,
											11,10, 9, 8,
											7, 6,  5, 3,
											4, 2,  1, 0);
		}
	}

	{
		uint8_t *ROM = memregion("oki")->base();
		std::vector<uint8_t> buffer(0x20000);
		int i;

		memcpy(&buffer[0],ROM,0x20000);
		for( i = 0; i < 0x20000; i++ )
		{
			ROM[i] = buffer[BITSWAP24(i,23,22,21,20,19,18,17,16,13,14,15,12,11,10,9,8,7,6,5,4,3,2,1,0)];
		}
	}
}


// these 2 are both unique revisions
GAME( 1989, toki,  0,    toki,   toki, toki_state,  toki,  ROT0, "TAD Corporation", "Toki (World, set 1)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, tokiu, toki, toki,   toki, toki_state,  toki,  ROT0, "TAD Corporation (Fabtek license)", "Toki (US, set 1)", MACHINE_SUPPORTS_SAVE )

GAME( 1989, tokip, toki, toki,   toki, toki_state,  toki,  ROT0, "TAD Corporation (Fabtek license)", "Toki (US, prototype?)", MACHINE_SUPPORTS_SAVE )


// these 3 are all the same revision, only the region byte differs
GAME( 1989, tokia, toki, toki,   toki, toki_state,  toki,  ROT0, "TAD Corporation", "Toki (World, set 2)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, tokiua,toki, toki,   toki, toki_state,  toki,  ROT0, "TAD Corporation (Fabtek license)", "Toki (US, set 2)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, juju,  toki, toki,   toki, toki_state,  toki,  ROT0, "TAD Corporation", "JuJu Densetsu (Japan)", MACHINE_SUPPORTS_SAVE )

GAME( 1990, tokib,  toki, tokib, tokib, toki_state, tokib, ROT0, "bootleg (Datsu)", "Toki (Datsu bootleg)", MACHINE_SUPPORTS_SAVE )
GAME( 1990, jujub,  toki, tokib, tokib, toki_state, tokib, ROT0, "bootleg (Playmark)", "JuJu Densetsu (Playmark bootleg)", MACHINE_SUPPORTS_SAVE )
/* Sound hardware seems to have been slightly modified, the coins are handled ok, but there is no music and bad sfx.  Program roms have a slight bitswap, Flipscreen also seems to be ignored */
GAME( 1989, jujuba, toki, jujuba, toki, toki_state, jujuba, ROT180, "bootleg", "JuJu Densetsu (Japan, bootleg)", MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE ) // bootleg of tokia/juju revison
