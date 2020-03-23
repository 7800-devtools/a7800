// license:BSD-3-Clause
// copyright-holders:Jarek Parchanski, Andrea Mazzoleni
/***************************************************************************

Return of the Invaders

driver by Jarek Parchanski, Andrea Mazzoleni

UPL is said to have developed Return of the Invaders for Taito, but some digging
in the ROMs reveals the following strings:

 Programed by NAKAKUMA AKIRA
 V 1.0 (85.3.15)

 NOBUO KODERA
 AKIRA NAKAKUMA
 HIKARU KASAHARA
 COMIX LTD. 1985.

Notes:
- The rom names and locations are derived from PCB pictures.

- The video hardware (especially the sprite system) is quite obviously derived
  from a Namco design.

- Two bits of tilemap RAM might be used for tile flip, but the game never sets
  them so we can't verify without schematics.

- In addition to a dump of the original MCU, we have a dump from a bootleg MCU.
  The game does work with it, but only when the flip screen dip switch is turned
  off. If it is set to on, the game hangs when starting a game because the
  mcu doesn't answer a command.
  See bootleg MCU code at $206 and $435: when the dip switch is on, the
  "lda #$00" should be replaced by "lda #$01".



                   Invaders "G" Connector
    Component Side           |         Solder Side
------------------------------------------------------------------
      Ground             | 1 | A |     Ground
      Video Red          | 2 | B |     Video Ground
      Video Green        | 3 | C |     Video Blue
 Negative Composite Sync | 4 | D |
      Speaker +          | 5 | E |     Speaker -
---------- KEY ----------| 6 | 6 |---------- KEY ----------
                         | 7 | H |
      Coinswitch         | 8 | J |
                         | 9 | K |
                         | 10| L |
                         | 11| M |
      Start Player 1     | 12| N |      Start Player 2
                         | 13| P |
                         | 14| R |
      Joystick Right     | 15| S |
      Joystick Left      | 16| T |
                         | 17| U |
                         | 18| V |
                         | 19| W |
                         | 20| X |
      Fire               | 21| Y |
                         | 22| Z |


                   Invaders "T" Connector
    Component Side           |         Solder Side
------------------------------------------------------------------
      Ground             | 1 | A |     Ground
                         | 2 | B |
                         | 3 | C |
                         | 4 | D |
                         | 5 | E |
                         | 6 | 6 |
---------- KEY ----------| 7 | H |---------- KEY ----------
                         | 8 | J |
                         | 9 | K |
                         | 10| L |
                         | 11| M |
                         | 12| N |
                         | 13| P |
                         | 14| R |
                         | 15| S |
                         | 16| T |
                         | 17| U |
      +5V                | 18| V |     +5V


 Invaders "H" Connector
-----------------------
 | 1 |  Ground
 | 2 |  Ground
 | 3 |
 | 4 |
 | 5 |  +5V
 | 6 |  +5V
 | 7 |
 | 8 |
 | 9 |  +12V
 | 10|-- Key --
 | 11|
 | 12|  +12V


***************************************************************************/

#include "emu.h"
#include "includes/retofinv.h"

#include "cpu/z80/z80.h"
#include "machine/watchdog.h"
#include "sound/sn76496.h"
#include "screen.h"
#include "speaker.h"


void retofinv_state::machine_start()
{
	save_item(NAME(m_main_irq_mask));
	save_item(NAME(m_sub_irq_mask));
	save_item(NAME(m_cpu2_m6000));
}

WRITE8_MEMBER(retofinv_state::cpu1_reset_w)
{
	m_subcpu->set_input_line(INPUT_LINE_RESET, data ? CLEAR_LINE : ASSERT_LINE);
}

WRITE8_MEMBER(retofinv_state::cpu2_reset_w)
{
	m_audiocpu->set_input_line(INPUT_LINE_RESET, data ? CLEAR_LINE : ASSERT_LINE);
}

WRITE8_MEMBER(retofinv_state::mcu_reset_w)
{
	m_68705->reset_w(data ? CLEAR_LINE : ASSERT_LINE);
}

WRITE8_MEMBER(retofinv_state::cpu2_m6000_w)
{
	m_cpu2_m6000 = data;
}

READ8_MEMBER(retofinv_state::cpu0_mf800_r)
{
	return m_cpu2_m6000;
}

WRITE8_MEMBER(retofinv_state::soundcommand_w)
{
	m_soundlatch->write(space, 0, data);
	m_audiocpu->set_input_line(0, HOLD_LINE);
}

WRITE8_MEMBER(retofinv_state::irq0_ack_w)
{
	m_main_irq_mask = data & 1;
	if (!m_main_irq_mask)
		m_maincpu->set_input_line(0, CLEAR_LINE);
}

WRITE8_MEMBER(retofinv_state::irq1_ack_w)
{
	m_sub_irq_mask = data & 1;
	if (!m_sub_irq_mask)
		m_subcpu->set_input_line(0, CLEAR_LINE);
}

WRITE8_MEMBER(retofinv_state::coincounter_w)
{
	machine().bookkeeping().coin_counter_w(0, data & 1);
}

WRITE8_MEMBER(retofinv_state::coinlockout_w)
{
	machine().bookkeeping().coin_lockout_w(0,~data & 1);
}

READ8_MEMBER(retofinv_state::mcu_status_r)
{
	/* bit 4 = when 1, mcu is ready to receive data from main cpu */
	/* bit 5 = when 1, mcu has sent data to the main cpu */
	return
			((CLEAR_LINE == m_68705->host_semaphore_r()) ? 0x10 : 0x00) |
			((CLEAR_LINE != m_68705->mcu_semaphore_r()) ? 0x20 : 0x00);
}

/* It is very likely that the main cpu and the sub cpu share the entire
   8000-ffff address space, each perhaps holding the other in waitstate
   while accessing it. */
/* the main cpu code looks for a string at 7b00 (in the space for the
   empty socket at IC73 from 6000-7fff) and if it finds a particular
   string there, it jumps to that area, presumably for diagnostic use */

static ADDRESS_MAP_START( bootleg_map, AS_PROGRAM, 8, retofinv_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x7fff, 0x7fff) AM_WRITE(coincounter_w)
	AM_RANGE(0x8000, 0x87ff) AM_RAM_WRITE(fg_videoram_w) AM_SHARE("fg_videoram")
	AM_RANGE(0x8800, 0x9fff) AM_RAM AM_SHARE("sharedram")
	AM_RANGE(0xa000, 0xa7ff) AM_RAM_WRITE(bg_videoram_w) AM_SHARE("bg_videoram")
	AM_RANGE(0xb800, 0xb802) AM_WRITE(gfx_ctrl_w)
	AM_RANGE(0xc000, 0xc000) AM_READ_PORT("P1")
	AM_RANGE(0xc001, 0xc001) AM_READ_PORT("P2")
	AM_RANGE(0xc002, 0xc002) AM_READNOP /* bit 7 must be 0, otherwise game resets */
	AM_RANGE(0xc004, 0xc004) AM_READ_PORT("SYSTEM")
	AM_RANGE(0xc005, 0xc005) AM_READ_PORT("DSW1")
	AM_RANGE(0xc006, 0xc006) AM_READ_PORT("DSW2")
	AM_RANGE(0xc007, 0xc007) AM_READ_PORT("DSW3")
	AM_RANGE(0xc800, 0xc800) AM_WRITE(irq0_ack_w)
	AM_RANGE(0xc801, 0xc801) AM_WRITE(coinlockout_w)
	AM_RANGE(0xc802, 0xc802) AM_WRITE(cpu2_reset_w)
//  AM_RANGE(0xc804, 0xc804) AM_WRITE(irq1_ack_w)   // presumably (meaning memory map is shared with cpu 1)
	AM_RANGE(0xc805, 0xc805) AM_WRITE(cpu1_reset_w)
	AM_RANGE(0xd000, 0xd000) AM_DEVWRITE("watchdog", watchdog_timer_device, reset_w)
	AM_RANGE(0xd800, 0xd800) AM_WRITE(soundcommand_w)
	AM_RANGE(0xf800, 0xf800) AM_READ(cpu0_mf800_r)
ADDRESS_MAP_END

static ADDRESS_MAP_START( main_map, AS_PROGRAM, 8, retofinv_state )
	AM_IMPORT_FROM(bootleg_map)
	AM_RANGE(0xc003, 0xc003) AM_READ(mcu_status_r)
	AM_RANGE(0xc803, 0xc803) AM_WRITE(mcu_reset_w)
	AM_RANGE(0xe000, 0xe000) AM_DEVREAD("68705", taito68705_mcu_device, data_r)
	AM_RANGE(0xe800, 0xe800) AM_DEVWRITE("68705", taito68705_mcu_device, data_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( sub_map, AS_PROGRAM, 8, retofinv_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x87ff) AM_RAM_WRITE(fg_videoram_w) AM_SHARE("fg_videoram")
	AM_RANGE(0x8800, 0x9fff) AM_RAM AM_SHARE("sharedram")
	AM_RANGE(0xa000, 0xa7ff) AM_RAM_WRITE(bg_videoram_w) AM_SHARE("bg_videoram")
	AM_RANGE(0xc804, 0xc804) AM_WRITE(irq1_ack_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( sound_map, AS_PROGRAM, 8, retofinv_state )
	AM_RANGE(0x0000, 0x1fff) AM_ROM
	AM_RANGE(0x2000, 0x27ff) AM_RAM /* 6116 sram at IC28 */
	AM_RANGE(0x4000, 0x4000) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
	AM_RANGE(0x6000, 0x6000) AM_WRITE(cpu2_m6000_w)
	AM_RANGE(0x8000, 0x8000) AM_DEVWRITE("sn1", sn76489a_device, write)
	AM_RANGE(0xa000, 0xa000) AM_DEVWRITE("sn2", sn76489a_device, write)
	AM_RANGE(0xe000, 0xffff) AM_ROM         /* space for diagnostic ROM */
ADDRESS_MAP_END



static INPUT_PORTS_START( retofinv )
	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH,IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH,IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_2WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_2WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON1 )

	PORT_START("P2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_2WAY PORT_COCKTAIL
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_2WAY PORT_COCKTAIL
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL

	PORT_START("DSW1")
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Bonus_Life ) )   PORT_DIPLOCATION("DSW-A (RB3):1,2")
	PORT_DIPSETTING(    0x03, "30k, 80k & every 80k" )
	PORT_DIPSETTING(    0x02, "30k, 80k" )
	PORT_DIPSETTING(    0x01, "30k" )
	PORT_DIPSETTING(    0x00, DEF_STR( None ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Free_Play ) )   PORT_DIPLOCATION("DSW-A (RB3):3")
	PORT_DIPSETTING(    0x04, DEF_STR( No ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x18, 0x08, DEF_STR( Lives ) )   PORT_DIPLOCATION("DSW-A (RB3):4,5")
	PORT_DIPSETTING(    0x18, "1" )
	PORT_DIPSETTING(    0x10, "2" )
	PORT_DIPSETTING(    0x08, "3" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unused ) )   PORT_DIPLOCATION("DSW-A (RB3):6")   // according to manual
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Flip_Screen ) )   PORT_DIPLOCATION("DSW-A (RB3):7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Cabinet ) )   PORT_DIPLOCATION("DSW-A (RB3):8")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Cocktail ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x0f, 0x00, DEF_STR( Coin_A ) )   PORT_DIPLOCATION("DSW-B (RB2):1,2,3,4")
	PORT_DIPSETTING(    0x0f, DEF_STR( 9C_1C ) )
	PORT_DIPSETTING(    0x0e, DEF_STR( 8C_1C ) )
	PORT_DIPSETTING(    0x0d, DEF_STR( 7C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 6C_1C ) )
	PORT_DIPSETTING(    0x0b, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x0a, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x09, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 1C_7C ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 1C_8C ) )
	PORT_DIPNAME( 0xf0, 0x00, DEF_STR( Coin_B ) )   PORT_DIPLOCATION("DSW-B (RB2):5,6,7,8")
	PORT_DIPSETTING(    0xf0, DEF_STR( 9C_1C ) )
	PORT_DIPSETTING(    0xe0, DEF_STR( 8C_1C ) )
	PORT_DIPSETTING(    0xd0, DEF_STR( 7C_1C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 6C_1C ) )
	PORT_DIPSETTING(    0xb0, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0xa0, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x90, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x50, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x60, DEF_STR( 1C_7C ) )
	PORT_DIPSETTING(    0x70, DEF_STR( 1C_8C ) )

	PORT_START("DSW3")
	PORT_DIPNAME( 0x01, 0x01, "Push Start to Skip Stage (Cheat)")   PORT_DIPLOCATION("DSW-C (RB1):1")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unused ) )   PORT_DIPLOCATION("DSW-C (RB1):2")   // according to manual
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unused ) )   PORT_DIPLOCATION("DSW-C (RB1):3")   // according to manual
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unused ) )   PORT_DIPLOCATION("DSW-C (RB1):4")   // according to manual
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, "Coin Per Play Display" )   PORT_DIPLOCATION("DSW-C (RB1):5")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x20, 0x20, "Year Display" )   PORT_DIPLOCATION("DSW-C (RB1):6")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x40, 0x40, "Invulnerability (Cheat)")   PORT_DIPLOCATION("DSW-C (RB1):7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Coinage ) )   PORT_DIPLOCATION("DSW-C (RB1):8")  // unused according to manual
	PORT_DIPSETTING(    0x80, "A and B" )
	PORT_DIPSETTING(    0x00, "A only" )
INPUT_PORTS_END

static INPUT_PORTS_START( retofin2 )
	PORT_INCLUDE( retofinv )

	PORT_MODIFY( "DSW1" )
	PORT_DIPNAME( 0x18, 0x08, DEF_STR( Lives ) )   PORT_DIPLOCATION("DSW-A (RB3):4,5")
	PORT_DIPSETTING(    0x18, "1" )
	PORT_DIPSETTING(    0x10, "2" )
	PORT_DIPSETTING(    0x08, "3" )
	PORT_DIPSETTING(    0x00, "4" )
INPUT_PORTS_END



static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,2),  /* bottom half of ROM is empty */
	1,
	{ 0 },
	{ 7, 6, 5, 4, 3, 2, 1, 0 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8
};

static const gfx_layout bglayout =
{
	8,8,
	RGN_FRAC(1,2),
	4,
	{ 0, RGN_FRAC(1,2)+4, RGN_FRAC(1,2), 4 },
	{ 0, 1, 2, 3, 8*8, 8*8+1, 8*8+2, 8*8+3 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	16*8
};

static const gfx_layout spritelayout =
{
	16,16,
	RGN_FRAC(1,2),
	4,
	{ 0, RGN_FRAC(1,2)+4, RGN_FRAC(1,2), 4 },
	{ 0, 1, 2, 3, 8*8, 8*8+1, 8*8+2, 8*8+3, 16*8+0, 16*8+1, 16*8+2, 16*8+3,
			24*8+0, 24*8+1, 24*8+2, 24*8+3 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
			32*8, 33*8, 34*8, 35*8, 36*8, 37*8, 38*8, 39*8 },
	64*8
};

static GFXDECODE_START( retofinv )
	GFXDECODE_ENTRY( "gfx1", 0, charlayout,             0, 256 )
	GFXDECODE_ENTRY( "gfx2", 0, spritelayout,       256*2,  64 )
	GFXDECODE_ENTRY( "gfx3", 0, bglayout,     64*16+256*2,  64 )
GFXDECODE_END

INTERRUPT_GEN_MEMBER(retofinv_state::main_vblank_irq)
{
	if(m_main_irq_mask)
		device.execute().set_input_line(0, ASSERT_LINE);
}

INTERRUPT_GEN_MEMBER(retofinv_state::sub_vblank_irq)
{
	if(m_sub_irq_mask)
		device.execute().set_input_line(0, ASSERT_LINE);
}


static MACHINE_CONFIG_START( retofinv )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_18_432MHz/6)    /* XTAL and divider verified, 3.072 MHz */
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", retofinv_state,  main_vblank_irq)

	MCFG_CPU_ADD("sub", Z80, XTAL_18_432MHz/6)    /* XTAL and divider verified, 3.072 MHz */
	MCFG_CPU_PROGRAM_MAP(sub_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", retofinv_state,  sub_vblank_irq)

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_18_432MHz/6)   /* XTAL and divider verified, 3.072 MHz */
	MCFG_CPU_PROGRAM_MAP(sound_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(retofinv_state, nmi_line_pulse, 2*60) // wrong, should be ~128-132 per frame, not 120.

	MCFG_CPU_ADD("68705", TAITO68705_MCU, XTAL_18_432MHz/6)    /* XTAL and divider verified, 3.072 MHz */

	MCFG_QUANTUM_TIME(attotime::from_hz(6000))  /* 100 CPU slices per frame - enough for the sound CPU to read all commands */

	MCFG_WATCHDOG_ADD("watchdog")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60.58) // vsync measured at 60.58hz
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0)) // not accurate
	MCFG_SCREEN_SIZE(36*8, 28*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 36*8-1, 0*8, 28*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(retofinv_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", retofinv)
	MCFG_PALETTE_ADD("palette", 256*2+64*16+64*16)
	MCFG_PALETTE_INDIRECT_ENTRIES(256)
	MCFG_PALETTE_INIT_OWNER(retofinv_state, retofinv)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("sn1", SN76489A, XTAL_18_432MHz/6)   /* @IC5?; XTAL, chip type, and divider verified, 3.072 MHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.80)

	MCFG_SOUND_ADD("sn2", SN76489A, XTAL_18_432MHz/6)   /* @IC6?; XTAL, chip type, and divider verified, 3.072 MHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.80)
MACHINE_CONFIG_END

/* bootleg has different palette clut */
static MACHINE_CONFIG_DERIVED( retofinvb, retofinv )
	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(retofinv_state, retofinv_bl)
MACHINE_CONFIG_END

/* bootleg which also has no mcu */
static MACHINE_CONFIG_DERIVED( retofinvb_nomcu, retofinvb )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(bootleg_map)

	MCFG_DEVICE_REMOVE("68705")
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( retofinv )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "a37__03.ic70", 0x0000, 0x2000, CRC(eae7459d) SHA1(c105f6adbd4c09decaad68ed13163d8f9b55e646) )
	ROM_LOAD( "a37__02.ic71", 0x2000, 0x2000, CRC(72895e37) SHA1(42fb904338e9f92a79d587eac401d456e7fb6e55) )
	ROM_LOAD( "a37__01.ic72", 0x4000, 0x2000, CRC(505dd20b) SHA1(3a34b1515bb834ff9e2d86b0b43a752d9619307b) )
	// ic73 socket is empty

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "a37__04.ic62", 0x0000, 0x2000, CRC(d2899cc1) SHA1(fdbec743b06f4cdcc134ef863e4e71337ad0b2c5) )
	// ic63 socket is empty
	// ic64 socket is empty
	// ic65 socket is empty

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "a37__05.ic17", 0x0000, 0x2000, CRC(9025abea) SHA1(2f03e8572f23624d7cd1215a55109e97fd66e271) )

	ROM_REGION( 0x0800, "68705:mcu", 0 )    /* 2k for the microcontroller */
	ROM_LOAD( "a37__09.ic37", 0x00000, 0x0800, CRC(6a6d008d) SHA1(dce55b65db22ba97cb7b3d6d545575e7945d42ad) ) /* original mc68705p5 from Taito board */
	/* Interestingly enough, the security bit is NOT set on this MCU, but the
	part is definitely a 68705P5 from an original Taito pcb running original
	Taito code! Compare the MCU code to Taito games A17, A22, A23, A24, A30,
	A34, A45, A47, A50, A54(and A51), A52, A64, A67, A68, probably others too,
	MCU code between 0x420 and 0x73f is identical, and boot vector is 0x570.
	The MCU command jump table at 0x740-0x780 differs on each, as does the
	lookup table at 0x350-0x41f, and the "custom command code area" from
	0x80-0x34F */

	ROM_REGION( 0x02000, "gfx1", 0 )
	ROM_LOAD( "a37__16.gfxboard.ic61", 0x0000, 0x2000, CRC(4e3f501c) SHA1(2d832f4038ae65bfdeedfab870f6f1176ec6b676) )

	ROM_REGION( 0x08000, "gfx2", 0 )
	ROM_LOAD( "a37__10.gfxboard.ic8",  0x0000, 0x2000, CRC(6afdeec8) SHA1(782fe0a8aea48c3c270318b7ba011fc6fce0db7a) )
	ROM_LOAD( "a37__11.gfxboard.ic9",  0x2000, 0x2000, CRC(d3dc9da3) SHA1(0d98d6e993b5a4845a23802751023b7a593dce29) )
	ROM_LOAD( "a37__12.gfxboard.ic10", 0x4000, 0x2000, CRC(d10b2eed) SHA1(3809a0adf935a119f9ee0d4c24f1456c35d2a6fa) )
	ROM_LOAD( "a37__13.gfxboard.ic11", 0x6000, 0x2000, CRC(00ca6b3d) SHA1(08ce5b13d5ebc79cc803949f4ba9e630e6cd92b8) )

	ROM_REGION( 0x04000, "gfx3", 0 )
	ROM_LOAD( "a37__14.gfxboard.ic55", 0x0000, 0x2000, CRC(ef7f8651) SHA1(2d91057501e5e9c4255e0d55fff0d99c2a5be7e8) )
	ROM_LOAD( "a37__15.gfxboard.ic56", 0x2000, 0x2000, CRC(03b40905) SHA1(c10d87796e8a6e6a2a37c6fb713821cc87299cc8) )

	ROM_REGION( 0x0300, "palette", 0 )
	// these three are Fujitsu 7052 PROMS, == 82S129 256x4bit TS proms
	ROM_LOAD( "a37-06.ic13", 0x0000, 0x0100, CRC(e9643b8b) SHA1(7bbb92a42e7c3effb701fc7b2c24f2470f31b063) )   /* palette red bits  */
	ROM_LOAD( "a37-07.ic4",  0x0100, 0x0100, CRC(e8f34e11) SHA1(8f438561b8d46ffff00747ed8baf0ebb6a081615) )   /* palette green bits */
	ROM_LOAD( "a37-08.ic3",  0x0200, 0x0100, CRC(50030af0) SHA1(e748ae0b8702b7d20fb65c254dceee23246b3d13) )   /* palette blue bits   */

	ROM_REGION( 0x0800, "clut", 0 )
	// these four are Harris 7643 PROMS, == 82S137 1kx4 TS proms and form the tile color lookup tables; address is scrambled slightly.
	ROM_LOAD_NIB_HIGH(  "a37-17.gfxboard.ic36",  0x0000, 0x0400, CRC(c63cf10e) SHA1(bca8823aef31ab8f4c22201c4efd51f9a4124c8f) )
	ROM_LOAD_NIB_LOW (  "a37-18.gfxboard.ic37",  0x0000, 0x0400, CRC(6db07bd1) SHA1(05b6728a96fecedae16cb3aa02de642a3a32d99d) )
	ROM_LOAD_NIB_HIGH(  "a37-19.gfxboard.ic83",  0x0400, 0x0400, CRC(a92aea27) SHA1(98e4726f40fdf0df2008ef03801ee35ede99e893) )
	ROM_LOAD_NIB_LOW (  "a37-20.gfxboard.ic84",  0x0400, 0x0400, CRC(77a7aaf6) SHA1(61a474f1ad09b89ff8302f2d903b86a90823116c) )
ROM_END

ROM_START( retofinvb ) // bootleg with black-box reverse-engineered mcu. Unclear what the 'correct' rom labels are for this set.
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "a37-03.70", 0x0000, 0x2000, CRC(eae7459d) SHA1(c105f6adbd4c09decaad68ed13163d8f9b55e646) )
	ROM_LOAD( "a37-02.71", 0x2000, 0x2000, CRC(72895e37) SHA1(42fb904338e9f92a79d587eac401d456e7fb6e55) )
	ROM_LOAD( "a37-01.72", 0x4000, 0x2000, CRC(505dd20b) SHA1(3a34b1515bb834ff9e2d86b0b43a752d9619307b) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "a37-04.62", 0x0000, 0x2000, CRC(d2899cc1) SHA1(fdbec743b06f4cdcc134ef863e4e71337ad0b2c5) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "a37-05.17", 0x0000, 0x2000, CRC(9025abea) SHA1(2f03e8572f23624d7cd1215a55109e97fd66e271) )

	ROM_REGION( 0x0800, "68705:mcu", 0 )    /* 2k for the microcontroller */
	/* this MCU is from a bootleg board and is a 'clean room' reimplementation by bootleggers, and is not 100% functional (flip screen does not work, see notes at top of driver) */
	ROM_LOAD( "a37-09_bootleg.37", 0x00000, 0x0800, CRC(79bd6ded) SHA1(4967e95b4461c1bfb4e933d1804677799014f77b) )

	ROM_REGION( 0x02000, "gfx1", 0 )
	ROM_LOAD( "a37-16.61", 0x0000, 0x2000, CRC(4e3f501c) SHA1(2d832f4038ae65bfdeedfab870f6f1176ec6b676) )

	ROM_REGION( 0x08000, "gfx2", 0 )
	ROM_LOAD( "a37-10.8",  0x0000, 0x2000, CRC(6afdeec8) SHA1(782fe0a8aea48c3c270318b7ba011fc6fce0db7a) )
	ROM_LOAD( "a37-11.9",  0x2000, 0x2000, CRC(d3dc9da3) SHA1(0d98d6e993b5a4845a23802751023b7a593dce29) )
	ROM_LOAD( "a37-12.10", 0x4000, 0x2000, CRC(d10b2eed) SHA1(3809a0adf935a119f9ee0d4c24f1456c35d2a6fa) )
	ROM_LOAD( "a37-13.11", 0x6000, 0x2000, CRC(00ca6b3d) SHA1(08ce5b13d5ebc79cc803949f4ba9e630e6cd92b8) )

	ROM_REGION( 0x04000, "gfx3", 0 )
	ROM_LOAD( "a37-14.55", 0x0000, 0x2000, CRC(ef7f8651) SHA1(2d91057501e5e9c4255e0d55fff0d99c2a5be7e8) )
	ROM_LOAD( "a37-15.56", 0x2000, 0x2000, CRC(03b40905) SHA1(c10d87796e8a6e6a2a37c6fb713821cc87299cc8) )

	ROM_REGION( 0x0300, "palette", 0 )
	ROM_LOAD( "a37-06.13", 0x0000, 0x0100, CRC(e9643b8b) SHA1(7bbb92a42e7c3effb701fc7b2c24f2470f31b063) )   /* palette red bits  */
	ROM_LOAD( "a37-07.4",  0x0100, 0x0100, CRC(e8f34e11) SHA1(8f438561b8d46ffff00747ed8baf0ebb6a081615) )   /* palette green bits */
	ROM_LOAD( "a37-08.3",  0x0200, 0x0100, CRC(50030af0) SHA1(e748ae0b8702b7d20fb65c254dceee23246b3d13) )   /* palette blue bits   */

	ROM_REGION( 0x0800, "clut", 0 ) // bootleg uses a single 82s191 2kx8 TS prom for the tile color lookup tables; data is scrambled slightly.
	ROM_LOAD( "82s191n",   0x0000, 0x0800, CRC(93c891e3) SHA1(643a0107717b6a434432dda73a0102e6e8adbca7) )
ROM_END

ROM_START( retofinvb1 ) // bootleg with mcu hacked out. Unclear what the 'correct' rom labels are for this set other than the 3 main roms.
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "roi.02",  0x0000, 0x2000, CRC(d98fd462) SHA1(fd35e13b7dee58639a01b040b8f84d42bb40b633) )
	ROM_LOAD( "roi.01b", 0x2000, 0x2000, CRC(3379f930) SHA1(c67d687a10b6240bd6e2fbdb15e1b7d276e6fc07) )
	ROM_LOAD( "roi.01",  0x4000, 0x2000, CRC(57679062) SHA1(4f121101ab1cb8de8e693e5984ef23fa587fe696) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "a37-04.62", 0x0000, 0x2000, CRC(d2899cc1) SHA1(fdbec743b06f4cdcc134ef863e4e71337ad0b2c5) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "a37-05.17", 0x0000, 0x2000, CRC(9025abea) SHA1(2f03e8572f23624d7cd1215a55109e97fd66e271) )

	ROM_REGION( 0x02000, "gfx1", 0 )
	ROM_LOAD( "a37-16.61", 0x0000, 0x2000, CRC(4e3f501c) SHA1(2d832f4038ae65bfdeedfab870f6f1176ec6b676) )

	ROM_REGION( 0x08000, "gfx2", 0 )
	ROM_LOAD( "a37-10.8",  0x0000, 0x2000, CRC(6afdeec8) SHA1(782fe0a8aea48c3c270318b7ba011fc6fce0db7a) )
	ROM_LOAD( "a37-11.9",  0x2000, 0x2000, CRC(d3dc9da3) SHA1(0d98d6e993b5a4845a23802751023b7a593dce29) )
	ROM_LOAD( "a37-12.10", 0x4000, 0x2000, CRC(d10b2eed) SHA1(3809a0adf935a119f9ee0d4c24f1456c35d2a6fa) )
	ROM_LOAD( "a37-13.11", 0x6000, 0x2000, CRC(00ca6b3d) SHA1(08ce5b13d5ebc79cc803949f4ba9e630e6cd92b8) )

	ROM_REGION( 0x04000, "gfx3", 0 )
	ROM_LOAD( "a37-14.55", 0x0000, 0x2000, CRC(ef7f8651) SHA1(2d91057501e5e9c4255e0d55fff0d99c2a5be7e8) )
	ROM_LOAD( "a37-15.56", 0x2000, 0x2000, CRC(03b40905) SHA1(c10d87796e8a6e6a2a37c6fb713821cc87299cc8) )

	ROM_REGION( 0x0300, "palette", 0 )
	ROM_LOAD( "a37-06.13", 0x0000, 0x0100, CRC(e9643b8b) SHA1(7bbb92a42e7c3effb701fc7b2c24f2470f31b063) )   /* palette red bits  */
	ROM_LOAD( "a37-07.4",  0x0100, 0x0100, CRC(e8f34e11) SHA1(8f438561b8d46ffff00747ed8baf0ebb6a081615) )   /* palette green bits */
	ROM_LOAD( "a37-08.3",  0x0200, 0x0100, CRC(50030af0) SHA1(e748ae0b8702b7d20fb65c254dceee23246b3d13) )   /* palette blue bits   */

	ROM_REGION( 0x0800, "clut", 0 ) // bootleg uses a single 82s191 2kx8 TS prom for the tile color lookup tables; data is scrambled slightly.
	ROM_LOAD( "82s191n",   0x0000, 0x0800, CRC(93c891e3) SHA1(643a0107717b6a434432dda73a0102e6e8adbca7) )
ROM_END

ROM_START( retofinvb2 ) // bootleg with mcu hacked out. Unclear what the 'correct' rom labels are for this set other than 2 of the 3 main roms.
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "ri-c.1e", 0x0000, 0x2000, CRC(e3c31260) SHA1(cc8774251c567da2e4a54091223927c95f497fe8) )
	ROM_LOAD( "roi.01b", 0x2000, 0x2000, CRC(3379f930) SHA1(c67d687a10b6240bd6e2fbdb15e1b7d276e6fc07) ) // likely should be "ri-b.1d"
	ROM_LOAD( "ri-a.1c", 0x4000, 0x2000, CRC(3ae7c530) SHA1(5d1be375494fa07124071067661c4bfc2d724d54) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "a37-04.62", 0x0000, 0x2000, CRC(d2899cc1) SHA1(fdbec743b06f4cdcc134ef863e4e71337ad0b2c5) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "a37-05.17", 0x0000, 0x2000, CRC(9025abea) SHA1(2f03e8572f23624d7cd1215a55109e97fd66e271) )

	ROM_REGION( 0x02000, "gfx1", 0 )
	ROM_LOAD( "a37-16.61", 0x0000, 0x2000, CRC(4e3f501c) SHA1(2d832f4038ae65bfdeedfab870f6f1176ec6b676) )

	ROM_REGION( 0x08000, "gfx2", 0 )
	ROM_LOAD( "a37-10.8",  0x0000, 0x2000, CRC(6afdeec8) SHA1(782fe0a8aea48c3c270318b7ba011fc6fce0db7a) )
	ROM_LOAD( "a37-11.9",  0x2000, 0x2000, CRC(d3dc9da3) SHA1(0d98d6e993b5a4845a23802751023b7a593dce29) )
	ROM_LOAD( "a37-12.10", 0x4000, 0x2000, CRC(d10b2eed) SHA1(3809a0adf935a119f9ee0d4c24f1456c35d2a6fa) )
	ROM_LOAD( "a37-13.11", 0x6000, 0x2000, CRC(00ca6b3d) SHA1(08ce5b13d5ebc79cc803949f4ba9e630e6cd92b8) )

	ROM_REGION( 0x04000, "gfx3", 0 )
	ROM_LOAD( "a37-14.55", 0x0000, 0x2000, CRC(ef7f8651) SHA1(2d91057501e5e9c4255e0d55fff0d99c2a5be7e8) )
	ROM_LOAD( "a37-15.56", 0x2000, 0x2000, CRC(03b40905) SHA1(c10d87796e8a6e6a2a37c6fb713821cc87299cc8) )

	ROM_REGION( 0x0300, "palette", 0 )
	ROM_LOAD( "a37-06.13", 0x0000, 0x0100, CRC(e9643b8b) SHA1(7bbb92a42e7c3effb701fc7b2c24f2470f31b063) )   /* palette red bits  */
	ROM_LOAD( "a37-07.4",  0x0100, 0x0100, CRC(e8f34e11) SHA1(8f438561b8d46ffff00747ed8baf0ebb6a081615) )   /* palette green bits */
	ROM_LOAD( "a37-08.3",  0x0200, 0x0100, CRC(50030af0) SHA1(e748ae0b8702b7d20fb65c254dceee23246b3d13) )   /* palette blue bits   */

	ROM_REGION( 0x0800, "clut", 0 ) // bootleg uses a single 82s191 2kx8 TS prom for the tile color lookup tables; data is scrambled slightly.
	ROM_LOAD( "82s191n",   0x0000, 0x0800, CRC(93c891e3) SHA1(643a0107717b6a434432dda73a0102e6e8adbca7) )
ROM_END

ROM_START( retofinvb3 ) // Italian bootleg PCB. Only maincpu ROMs differ from parent and the MCU isn't present.
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "1.11", 0x0000, 0x2000, CRC(71c216ca) SHA1(34d04889dae6d6f586bce99413c1864dee52cf39) ) // unique
	ROM_LOAD( "2.10", 0x2000, 0x2000, CRC(3379f930) SHA1(c67d687a10b6240bd6e2fbdb15e1b7d276e6fc07) ) // same as retofinvb1 and retofinvb2
	ROM_LOAD( "3.9",  0x4000, 0x2000, CRC(92d79fa8) SHA1(00610cd51ba73783d12bc97584ca0fd7fff044eb) ) // unique

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "4.15", 0x0000, 0x2000, CRC(d2899cc1) SHA1(fdbec743b06f4cdcc134ef863e4e71337ad0b2c5) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "5.BIN", 0x0000, 0x2000, CRC(9025abea) SHA1(2f03e8572f23624d7cd1215a55109e97fd66e271) )

	ROM_REGION( 0x02000, "gfx1", 0 )
	ROM_LOAD( "16.7", 0x0000, 0x2000, CRC(4e3f501c) SHA1(2d832f4038ae65bfdeedfab870f6f1176ec6b676) )

	ROM_REGION( 0x08000, "gfx2", 0 )
	ROM_LOAD( "10.1",  0x0000, 0x2000, CRC(6afdeec8) SHA1(782fe0a8aea48c3c270318b7ba011fc6fce0db7a) )
	ROM_LOAD( "11.2",  0x2000, 0x2000, CRC(d3dc9da3) SHA1(0d98d6e993b5a4845a23802751023b7a593dce29) )
	ROM_LOAD( "12.3", 0x4000, 0x2000, CRC(d10b2eed) SHA1(3809a0adf935a119f9ee0d4c24f1456c35d2a6fa) )
	ROM_LOAD( "13.4", 0x6000, 0x2000, CRC(00ca6b3d) SHA1(08ce5b13d5ebc79cc803949f4ba9e630e6cd92b8) )

	ROM_REGION( 0x04000, "gfx3", 0 )
	ROM_LOAD( "14.5", 0x0000, 0x2000, CRC(ef7f8651) SHA1(2d91057501e5e9c4255e0d55fff0d99c2a5be7e8) )
	ROM_LOAD( "15.6", 0x2000, 0x2000, CRC(03b40905) SHA1(c10d87796e8a6e6a2a37c6fb713821cc87299cc8) )

	ROM_REGION( 0x0300, "palette", 0 )
	ROM_LOAD( "74s287.b",  0x0000, 0x0100, CRC(e9643b8b) SHA1(7bbb92a42e7c3effb701fc7b2c24f2470f31b063) )   /* palette red bits  */
	ROM_LOAD( "74s287.c",  0x0100, 0x0100, CRC(e8f34e11) SHA1(8f438561b8d46ffff00747ed8baf0ebb6a081615) )   /* palette green bits */
	ROM_LOAD( "74s287.a",  0x0200, 0x0100, CRC(50030af0) SHA1(e748ae0b8702b7d20fb65c254dceee23246b3d13) )   /* palette blue bits   */

	ROM_REGION( 0x0800, "clut", 0 )
	ROM_LOAD_NIB_HIGH( "6353-1.a",  0x0000, 0x0400, CRC(c63cf10e) SHA1(bca8823aef31ab8f4c22201c4efd51f9a4124c8f) )
	ROM_LOAD_NIB_LOW ( "6353-1.b",  0x0000, 0x0400, CRC(6db07bd1) SHA1(05b6728a96fecedae16cb3aa02de642a3a32d99d) )
	ROM_LOAD_NIB_HIGH( "6353-1.d",  0x0400, 0x0400, CRC(a92aea27) SHA1(98e4726f40fdf0df2008ef03801ee35ede99e893) )
	ROM_LOAD_NIB_LOW ( "6353-1.c",  0x0400, 0x0400, CRC(77a7aaf6) SHA1(61a474f1ad09b89ff8302f2d903b86a90823116c) )
ROM_END

GAME( 1985, retofinv,   0,        retofinv,        retofinv, retofinv_state, 0, ROT90, "Taito Corporation", "Return of the Invaders",                        MACHINE_SUPPORTS_SAVE )
GAME( 1985, retofinvb,  retofinv, retofinvb,       retofinv, retofinv_state, 0, ROT90, "bootleg",           "Return of the Invaders (bootleg w/MCU)",        MACHINE_SUPPORTS_SAVE )
GAME( 1985, retofinvb1, retofinv, retofinvb_nomcu, retofinv, retofinv_state, 0, ROT90, "bootleg",           "Return of the Invaders (bootleg no MCU set 1)", MACHINE_SUPPORTS_SAVE )
GAME( 1985, retofinvb2, retofinv, retofinvb_nomcu, retofin2, retofinv_state, 0, ROT90, "bootleg",           "Return of the Invaders (bootleg no MCU set 2)", MACHINE_SUPPORTS_SAVE )
GAME( 1985, retofinvb3, retofinv, retofinvb_nomcu, retofinv, retofinv_state, 0, ROT90, "bootleg",           "Return of the Invaders (bootleg no MCU set 3)", MACHINE_SUPPORTS_SAVE )
