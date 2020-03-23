// license:BSD-3-Clause
// copyright-holders:Phil Stroffolino, Carlos A. Lozano, Rob Rosenbrock
/***************************************************************************

Renegade
(c)1986 Taito

Nekketsu Kouha Kunio Kun
(c)1986 Technos Japan

Nekketsu Kouha Kunio Kun (bootleg)

driver by Phil Stroffolino, Carlos A. Lozano, Rob Rosenbrock

to enter test mode, hold down P1+P2 and press reset

NMI is used to refresh the sprites
IRQ is used to handle coin inputs

Known issues:
- coin counter isn't working properly (interrupt related?)

Memory Map (Preliminary):

Working RAM
  $24       used to mirror bankswitch state
  $25       coin trigger state
  $26       #credits (decimal)
  $27 -  $28    partial credits
  $2C -  $2D    sprite refresh trigger (used by NMI)
  $31       live/demo (if live, player controls are read from input ports)
  $32       indicates 2 player (alternating) game, or 1 player game
  $33       active player
  $37       stage number
  $38       stage state (for stages with more than one part)
  $40       game status flags; 0x80 indicates time over, 0x40 indicates player dead
 $220       player health
 $222 -  $223   stage timer
 $48a -  $48b   horizontal scroll buffer
 $511 -  $690   sprite RAM buffer
 $693       num pending sound commands
 $694 -  $698   sound command queue

$1002       #lives
$1014 - $1015   stage timer - separated digits
$1017 - $1019   stage timer: (ticks,seconds,minutes)
$101a       timer for palette animation
$1020 - $1048   high score table
$10e5 - $10ff   68705 data buffer

Video RAM
$1800 - $1bff   text layer, characters
$1c00 - $1fff   text layer, character attributes
$2000 - $217f   MIX RAM (96 sprites)
$2800 - $2bff   BACK LOW MAP RAM (background tiles)
$2C00 - $2fff   BACK HIGH MAP RAM (background attributes)
$3000 - $30ff   COLOR RG RAM
$3100 - $31ff   COLOR B RAM

Registers
$3800w  scroll(0ff)
$3801w  scroll(300)
$3802w  sound command
$3803w  screen flip (0=flip; 1=noflip)

$3804w  send data to 68705
$3804r  receive data from 68705

$3805w  bankswitch
$3806w  watchdog?
$3807w  coin counter

$3800r  'player1'
    xx      start buttons
      xx    fire buttons
        xxxx    joystick state

$3801r  'player2'
    xx      coin inputs
      xx    fire buttons
        xxxx    joystick state

$3802r  'DIP2'
    x       unused?
     x      vblank
      x     0: 68705 is ready to send information
       x    1: 68705 is ready to receive information
        xx  3rd fire buttons for player 2,1
          xx    difficulty

$3803r 'DIP1'
    x       screen flip
     x      cabinet type
      x     bonus (extra life for high score)
       x    starting lives: 1 or 2
        xxxx    coins per play

ROM
$4000 - $7fff   bankswitched ROM
$8000 - $ffff   ROM

***************************************************************************/

#include "emu.h"
#include "includes/renegade.h"

#include "cpu/m6502/m6502.h"
#include "cpu/m6809/m6809.h"
#include "sound/3526intf.h"
#include "screen.h"
#include "speaker.h"


/**************************************************************************/
/*  ADPCM sound
**
**  Inferred from the 6809 code and analogy with ddragon
**  NMI at end of sample is not needed in order for
**  playback to work, but seems to be what the code expects
*/

WRITE8_MEMBER(renegade_state::adpcm_start_w)
{
	m_msm->reset_w(0);
	m_adpcm_playing = true;
}

WRITE8_MEMBER(renegade_state::adpcm_addr_w)
{
	// table at $CB52 in audiocpu program:
	// 38 38 39 3A 3B 34 35 36 37 2C 2D 2E 2F
	//
	// bits 2-4 are active-low chip select; bit 5 is always set
	// (chip select for an unpopulated fourth ROM?)
	switch (data & 0x1c)
	{
		case 0x18: m_adpcm_pos = 0 * 0x8000 * 2; break;     // 110 -> ic33
		case 0x14: m_adpcm_pos = 1 * 0x8000 * 2; break;     // 101 -> ic32
		case 0x0c: m_adpcm_pos = 2 * 0x8000 * 2; break;     // 011 -> ic31
		default: m_adpcm_pos = m_adpcm_end = 0; return; // doesn't happen
	}
	// bits 0-1 are a13-a14
	m_adpcm_pos |= (data & 0x03) * 0x2000 * 2;
	// a0-a12 are driven by a binary counter; playback ends when it rolls over
	m_adpcm_end = m_adpcm_pos + 0x2000 * 2;
}

WRITE8_MEMBER(renegade_state::adpcm_stop_w)
{
	m_msm->reset_w(1);
	m_adpcm_playing = false;
}

WRITE_LINE_MEMBER(renegade_state::adpcm_int)
{
	if (!m_adpcm_playing) return;

	if (m_adpcm_pos >= m_adpcm_end)
	{
		m_msm->reset_w(1);
		m_adpcm_playing = false;
		m_audiocpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
	}
	else
	{
		uint8_t const data = m_adpcmrom[m_adpcm_pos / 2];
		m_msm->data_w(m_adpcm_pos & 1 ? data & 0xf : data >> 4);
		m_adpcm_pos++;
	}
}

void renegade_state::machine_start()
{
	m_rombank->configure_entries(0, 2, memregion("maincpu")->base(), 0x4000);

	save_item(NAME(m_adpcm_pos));
	save_item(NAME(m_adpcm_end));
	save_item(NAME(m_adpcm_playing));
}


/***************************************************************************

    MCU interface

***************************************************************************/

READ8_MEMBER(renegade_state::mcu_reset_r)
{
	m_mcu->reset_w(PULSE_LINE);
	return 0;
}

CUSTOM_INPUT_MEMBER(renegade_state::mcu_status_r)
{
	if (m_mcu.found())
	{
		return
			((CLEAR_LINE == m_mcu->host_semaphore_r()) ? 0x01 : 0x00) |
			((CLEAR_LINE == m_mcu->mcu_semaphore_r()) ? 0x02 : 0x00);
	}
	else
	{
		return 0x00;
	}
}

/********************************************************************************************/

WRITE8_MEMBER(renegade_state::bankswitch_w)
{
	m_rombank->set_entry(data & 1);
}

TIMER_DEVICE_CALLBACK_MEMBER(renegade_state::interrupt)
{
	int scanline = param;

	if (scanline == 112) // ???
		m_maincpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
	else if(scanline == 240)
		m_maincpu->set_input_line(0, HOLD_LINE);
}

WRITE8_MEMBER(renegade_state::coincounter_w)
{
	//coin_counter_w(offset, data);
}


/********************************************************************************************/

static ADDRESS_MAP_START( renegade_nomcu_map, AS_PROGRAM, 8, renegade_state )
	AM_RANGE(0x0000, 0x17ff) AM_RAM
	AM_RANGE(0x1800, 0x1fff) AM_RAM_WRITE(fg_videoram_w) AM_SHARE("fg_videoram")
	AM_RANGE(0x2000, 0x27ff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x2800, 0x2fff) AM_RAM_WRITE(bg_videoram_w) AM_SHARE("bg_videoram")
	AM_RANGE(0x3000, 0x30ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x3100, 0x31ff) AM_RAM_DEVWRITE("palette", palette_device, write_ext) AM_SHARE("palette_ext")
	AM_RANGE(0x3800, 0x3800) AM_READ_PORT("IN0") AM_WRITE(scroll_lsb_w)       /* Player#1 controls, P1,P2 start */
	AM_RANGE(0x3801, 0x3801) AM_READ_PORT("IN1") AM_WRITE(scroll_msb_w)       /* Player#2 controls, coin triggers */
	AM_RANGE(0x3802, 0x3802) AM_READ_PORT("DSW2") AM_DEVWRITE("soundlatch", generic_latch_8_device, write) /* DIP2  various IO ports */
	AM_RANGE(0x3803, 0x3803) AM_READ_PORT("DSW1") AM_WRITE(flipscreen_w)   /* DIP1 */
	AM_RANGE(0x3805, 0x3805) AM_READNOP AM_WRITE(bankswitch_w)
	AM_RANGE(0x3806, 0x3806) AM_WRITENOP // ?? watchdog
	AM_RANGE(0x3807, 0x3807) AM_WRITE(coincounter_w)
	AM_RANGE(0x4000, 0x7fff) AM_ROMBANK("rombank")
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( renegade_map, AS_PROGRAM, 8, renegade_state )
	AM_RANGE(0x3804, 0x3804) AM_DEVREADWRITE("mcu", taito68705_mcu_device, data_r, data_w)
	AM_RANGE(0x3805, 0x3805) AM_READ(mcu_reset_r)
	AM_IMPORT_FROM(renegade_nomcu_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( renegade_sound_map, AS_PROGRAM, 8, renegade_state )
	AM_RANGE(0x0000, 0x0fff) AM_RAM
	AM_RANGE(0x1000, 0x1000) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
	AM_RANGE(0x1800, 0x1800) AM_WRITE(adpcm_start_w)
	AM_RANGE(0x2000, 0x2000) AM_WRITE(adpcm_addr_w)
	AM_RANGE(0x2800, 0x2801) AM_DEVREADWRITE("ymsnd", ym3526_device, read, write)
	AM_RANGE(0x3000, 0x3000) AM_WRITE(adpcm_stop_w)
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END


static INPUT_PORTS_START( renegade )
	PORT_START("IN0")   /* IN0 */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )  PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )    PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )  PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1) /* attack left */
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1) /* jump */
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START2 )

	PORT_START("IN1")   /* IN1 */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )  PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )    PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )  PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_PLAYER(2)  /* attack left */
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2) PORT_PLAYER(2)  /* jump */
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN2 )

	PORT_START("DSW2")  /* DIP2 */
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Difficulty ) )   PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x02, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x03, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Very_Hard ) )

	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1) /* attack right */
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2) /* attack right */
	PORT_BIT( 0x30, IP_ACTIVE_HIGH, IPT_SPECIAL) PORT_CUSTOM_MEMBER(DEVICE_SELF, renegade_state, mcu_status_r, nullptr)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_CUSTOM ) PORT_VBLANK("screen")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_SERVICE1 )

	PORT_START("DSW1")  /* DIP1 */
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Coin_A ) )       PORT_DIPLOCATION("SW1:1,2")
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Coin_B ) )       PORT_DIPLOCATION("SW1:3,4")
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Lives ) )        PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x10, "1" )
	PORT_DIPSETTING(    0x00, "2" )
	PORT_DIPNAME( 0x20, 0x20, "Bonus" )                 PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(    0x20, "30k" )
	PORT_DIPSETTING(    0x00, DEF_STR( None ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Cabinet ) )      PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Flip_Screen ) )  PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END



static const gfx_layout charlayout =
{
	8,8, /* 8x8 characters */
	1024, /* 1024 characters */
	3, /* bits per pixel */
	{ 2, 4, 6 },    /* plane offsets; bit 0 is always clear */
	{ 1, 0, 65, 64, 129, 128, 193, 192 }, /* x offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 }, /* y offsets */
	32*8 /* offset to next character */
};

static const gfx_layout tileslayout1 =
{
	16,16, /* tile size */
	256, /* number of tiles */
	3, /* bits per pixel */

	/* plane offsets */
	{ 4, 0x8000*8+0, 0x8000*8+4 },

	/* x offsets */
	{ 3, 2, 1, 0, 16*8+3, 16*8+2, 16*8+1, 16*8+0,
		32*8+3,32*8+2 ,32*8+1 ,32*8+0 ,48*8+3 ,48*8+2 ,48*8+1 ,48*8+0 },

	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
		8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },

	64*8 /* offset to next tile */
};

static const gfx_layout tileslayout2 =
{
	16,16, /* tile size */
	256, /* number of tiles */
	3, /* bits per pixel */

	/* plane offsets */
	{ 0, 0xC000*8+0, 0xC000*8+4 },

	/* x offsets */
	{ 3, 2, 1, 0, 16*8+3, 16*8+2, 16*8+1, 16*8+0,
		32*8+3,32*8+2 ,32*8+1 ,32*8+0 ,48*8+3 ,48*8+2 ,48*8+1 ,48*8+0 },

	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
		8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },

	64*8 /* offset to next tile */
};

static const gfx_layout tileslayout3 =
{
	16,16, /* tile size */
	256, /* number of tiles */
	3, /* bits per pixel */

	/* plane offsets */
	{ 0x4000*8+4, 0x10000*8+0, 0x10000*8+4 },

	/* x offsets */
	{ 3, 2, 1, 0, 16*8+3, 16*8+2, 16*8+1, 16*8+0,
		32*8+3,32*8+2 ,32*8+1 ,32*8+0 ,48*8+3 ,48*8+2 ,48*8+1 ,48*8+0 },

	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
		8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },

	64*8 /* offset to next tile */
};

static const gfx_layout tileslayout4 =
{
	16,16, /* tile size */
	256, /* number of tiles */
	3, /* bits per pixel */

	/* plane offsets */
	{ 0x4000*8+0, 0x14000*8+0, 0x14000*8+4 },

	/* x offsets */
	{ 3, 2, 1, 0, 16*8+3, 16*8+2, 16*8+1, 16*8+0,
		32*8+3,32*8+2 ,32*8+1 ,32*8+0 ,48*8+3 ,48*8+2 ,48*8+1 ,48*8+0 },

	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
		8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },

	64*8 /* offset to next tile */
};

static GFXDECODE_START( renegade )
	/* 8x8 text, 8 colors */
	GFXDECODE_ENTRY( "chars", 0x00000, charlayout,   0, 4 ) /* colors   0- 32 */

	/* 16x16 background tiles, 8 colors */
	GFXDECODE_ENTRY( "tiles", 0x00000, tileslayout1, 192, 8 )   /* colors 192-255 */
	GFXDECODE_ENTRY( "tiles", 0x00000, tileslayout2, 192, 8 )
	GFXDECODE_ENTRY( "tiles", 0x00000, tileslayout3, 192, 8 )
	GFXDECODE_ENTRY( "tiles", 0x00000, tileslayout4, 192, 8 )

	GFXDECODE_ENTRY( "tiles", 0x18000, tileslayout1, 192, 8 )
	GFXDECODE_ENTRY( "tiles", 0x18000, tileslayout2, 192, 8 )
	GFXDECODE_ENTRY( "tiles", 0x18000, tileslayout3, 192, 8 )
	GFXDECODE_ENTRY( "tiles", 0x18000, tileslayout4, 192, 8 )

	/* 16x16 sprites, 8 colors */
	GFXDECODE_ENTRY( "sprites", 0x00000, tileslayout1, 128, 4 ) /* colors 128-159 */
	GFXDECODE_ENTRY( "sprites", 0x00000, tileslayout2, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x00000, tileslayout3, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x00000, tileslayout4, 128, 4 )

	GFXDECODE_ENTRY( "sprites", 0x18000, tileslayout1, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x18000, tileslayout2, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x18000, tileslayout3, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x18000, tileslayout4, 128, 4 )

	GFXDECODE_ENTRY( "sprites", 0x30000, tileslayout1, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x30000, tileslayout2, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x30000, tileslayout3, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x30000, tileslayout4, 128, 4 )

	GFXDECODE_ENTRY( "sprites", 0x48000, tileslayout1, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x48000, tileslayout2, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x48000, tileslayout3, 128, 4 )
	GFXDECODE_ENTRY( "sprites", 0x48000, tileslayout4, 128, 4 )
GFXDECODE_END


void renegade_state::machine_reset()
{
	m_rombank->set_entry(0);
	m_msm->reset_w(1);
	m_adpcm_playing = 0;
}


static MACHINE_CONFIG_START( renegade )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, 12000000/8)  /* 1.5 MHz (measured) */
	MCFG_CPU_PROGRAM_MAP(renegade_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", renegade_state, interrupt, "screen", 0, 1)

	MCFG_CPU_ADD("audiocpu", M6809, 12000000/8)
	MCFG_CPU_PROGRAM_MAP(renegade_sound_map)    /* IRQs are caused by the main CPU */

	MCFG_DEVICE_ADD("mcu", TAITO68705_MCU, 12000000/4) // ?

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)*2)  /* not accurate */
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(1*8, 31*8-1, 0, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(renegade_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", renegade)
	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")
	MCFG_GENERIC_LATCH_DATA_PENDING_CB(INPUTLINE("audiocpu", M6809_IRQ_LINE))

	MCFG_SOUND_ADD("ymsnd", YM3526, 12000000/4)
	MCFG_YM3526_IRQ_HANDLER(INPUTLINE("audiocpu", M6809_FIRQ_LINE))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_SOUND_ADD("msm", MSM5205, 12000000/32)
	MCFG_MSM5205_VCLK_CB(WRITELINE(renegade_state, adpcm_int))
	MCFG_MSM5205_PRESCALER_SELECTOR(S48_4B)  /* 8kHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( kuniokunb, renegade )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(renegade_nomcu_map)

	MCFG_DEVICE_REMOVE("mcu")
MACHINE_CONFIG_END


ROM_START( renegade )
	ROM_REGION( 0x10000, "maincpu", 0 ) /* 64k for code + bank switched ROM */
	ROM_LOAD( "na-5.ic52",     0x00000, 0x8000, CRC(de7e7df4) SHA1(7d26ac29e0b5858d9a0c0cdc86c864e464145260) )
	ROM_LOAD( "nb-5.ic51",     0x08000, 0x8000, CRC(ba683ddf) SHA1(7516fac1c4fd14cbf43481e94c0c26c662c4cd28) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "n0-5.ic13",     0x8000, 0x8000, CRC(3587de3b) SHA1(f82e758254b21eb0c5a02469c72adb86d9577065) )

	ROM_REGION( 0x0800, "mcu:mcu", 0 ) /* MC68705P5 */
	ROM_LOAD( "nz-5.ic97",    0x0000, 0x0800, CRC(32e47560) SHA1(93a386b3f3c8eb35a53487612147a877dc7453ff) )

	ROM_REGION( 0x08000, "chars", 0 )
	ROM_LOAD( "nc-5.bin",     0x0000, 0x8000, CRC(9adfaa5d) SHA1(7bdb7bd4387b49e0489f9539161e1ed9d8f9f6a0) )

	ROM_REGION( 0x30000, "tiles", 0 )
	ROM_LOAD( "n1-5.ic1",     0x00000, 0x8000, CRC(4a9f47f3) SHA1(01c94bc4c85314f1e0caa3afe91705875d118c13) )
	ROM_LOAD( "n6-5.ic28",    0x08000, 0x8000, CRC(d62a0aa8) SHA1(a0b55cd3eee352fb91d9bb8c6c4f4f55b2df83e9) )
	ROM_LOAD( "n7-5.ic27",    0x10000, 0x8000, CRC(7ca5a532) SHA1(1110aa1c7562805dd4b298ab2860c66a6cc2685b) )
	ROM_LOAD( "n2-5.ic14",    0x18000, 0x8000, CRC(8d2e7982) SHA1(72fc85ff7b54be10501a2a24303dadd5f33e5650) )
	ROM_LOAD( "n8-5.ic26",    0x20000, 0x8000, CRC(0dba31d3) SHA1(8fe250787debe07e4f6c0002a9f799869b13a5fd) )
	ROM_LOAD( "n9-5.ic25",    0x28000, 0x8000, CRC(5b621b6a) SHA1(45c6a688a5b4e9da71133c43cc48eea568557be3) )

	ROM_REGION( 0x60000, "sprites", 0 )
	ROM_LOAD( "nh-5.bin",     0x00000, 0x8000, CRC(dcd7857c) SHA1(eb530ccc939f2fa42b3c743605d5398f4afe7d7a) )
	ROM_LOAD( "nd-5.bin",     0x08000, 0x8000, CRC(2de1717c) SHA1(af5a994348301fa888092ae65d08cfb6ad124407) )
	ROM_LOAD( "nj-5.bin",     0x10000, 0x8000, CRC(0f96a18e) SHA1(1f7e11e11d5031b4942d9d05161bcb9466514af8) )
	ROM_LOAD( "nn-5.bin",     0x18000, 0x8000, CRC(1bf15787) SHA1(b3371bf33f8b76a4a9887a7a43dba1f26353e978) )
	ROM_LOAD( "ne-5.bin",     0x20000, 0x8000, CRC(924c7388) SHA1(2f3ee2f28d8b04df6258a3949b7b0f60a3ae358f) )
	ROM_LOAD( "nk-5.bin",     0x28000, 0x8000, CRC(69499a94) SHA1(2e92931ef4e8948e3985f0a242db4137016d8eea) )
	ROM_LOAD( "ni-5.bin",     0x30000, 0x8000, CRC(6f597ed2) SHA1(54d34c13cda1b41ef354f9e6f3ce34673ef6c020) )
	ROM_LOAD( "nf-5.bin",     0x38000, 0x8000, CRC(0efc8d45) SHA1(4fea3165fd279539bfd424f1dc355cbd741bc48d) )
	ROM_LOAD( "nl-5.bin",     0x40000, 0x8000, CRC(14778336) SHA1(17b4048942b5fa8167a7f2b471dbc5a5d3f017ee) )
	ROM_LOAD( "no-5.bin",     0x48000, 0x8000, CRC(147dd23b) SHA1(fa4f9b774845d0333909d876590cda38d19b72d8) )
	ROM_LOAD( "ng-5.bin",     0x50000, 0x8000, CRC(a8ee3720) SHA1(df3d40015b16fa7a9bf05f0ed5741c22f7f152c7) )
	ROM_LOAD( "nm-5.bin",     0x58000, 0x8000, CRC(c100258e) SHA1(0e2124e642b9742a9a0045f460974025048bc2dd) )

	ROM_REGION( 0x18000, "adpcm", 0 )
	ROM_LOAD( "n3-5.ic33",    0x00000, 0x8000, CRC(78fd6190) SHA1(995df0e88f5c34946e0634b50bda8c1cc621afaa) )
	ROM_LOAD( "n4-5.ic32",    0x08000, 0x8000, CRC(6557564c) SHA1(b3142be9d48eacb43786079a7ae012010f6afabb) )
	ROM_LOAD( "n5-5.ic31",    0x10000, 0x8000, CRC(7ee43a3c) SHA1(36b14b886096177cdd0bd0c99cbcfcc362b2bc30) )
ROM_END

ROM_START( kuniokun )
	ROM_REGION( 0x10000, "maincpu", 0 ) /* 64k for code + bank switched ROM */
	ROM_LOAD( "ta18-11.bin",  0x00000, 0x8000, CRC(f240f5cd) SHA1(ed6875e8ad2988e88389d4f63ff448d0823c195f) )
	ROM_LOAD( "nb-01.bin",    0x08000, 0x8000, CRC(93fcfdf5) SHA1(51cdb9377544ae17895e427f21d150ce195ab8e7) ) // original

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "n0-5.bin",     0x8000, 0x8000, CRC(3587de3b) SHA1(f82e758254b21eb0c5a02469c72adb86d9577065) )

	ROM_REGION( 0x0800, "mcu:mcu", 0 ) // MC68705P3
	ROM_LOAD( "nz-0.bin",     0x0000, 0x0800, CRC(650bb5f0) SHA1(56a9679e3265de244ce2191a81b709992f012111) )

	ROM_REGION( 0x08000, "chars", 0 )
	ROM_LOAD( "ta18-25.bin",  0x0000, 0x8000, CRC(9bd2bea3) SHA1(fa79c9d4c71c1dbbf0e14cb8d6870f1f94b9af88) )

	ROM_REGION( 0x30000, "tiles", 0 )
	ROM_LOAD( "ta18-01.bin",  0x00000, 0x8000, CRC(daf15024) SHA1(f37de97275f52dfbbad7bf8c82f8108e84bcf63e) )
	ROM_LOAD( "ta18-06.bin",  0x08000, 0x8000, CRC(1f59a248) SHA1(8ab70aa8f0dccbe94240c96835a43b0900d52120) )
	ROM_LOAD( "n7-5.bin",     0x10000, 0x8000, CRC(7ca5a532) SHA1(1110aa1c7562805dd4b298ab2860c66a6cc2685b) )
	ROM_LOAD( "ta18-02.bin",  0x18000, 0x8000, CRC(994c0021) SHA1(9219464decc1b07591d0485502e2bcc0c2d16261) )
	ROM_LOAD( "ta18-04.bin",  0x20000, 0x8000, CRC(55b9e8aa) SHA1(26c91030c53a022c1f1f3131768e8f7ba613168d) )
	ROM_LOAD( "ta18-03.bin",  0x28000, 0x8000, CRC(0475c99a) SHA1(36b7b856e728c68e0dd3ecb844033369a5117270) )

	ROM_REGION( 0x60000, "sprites", 0 )
	ROM_LOAD( "ta18-20.bin",  0x00000, 0x8000, CRC(c7d54139) SHA1(f76d237a6ee8bbcbf344145d31e532834da7c131) )
	ROM_LOAD( "ta18-24.bin",  0x08000, 0x8000, CRC(84677d45) SHA1(cb7fe69e13d2d696acbc464b7584c7514cfc7f85) )
	ROM_LOAD( "ta18-18.bin",  0x10000, 0x8000, CRC(1c770853) SHA1(4fe6051265729a9d36b6d3dd826c3f6dcb4a7a25) )
	ROM_LOAD( "ta18-14.bin",  0x18000, 0x8000, CRC(af656017) SHA1(d395d35fe6d8e281596b2df571099b841f979a97) )
	ROM_LOAD( "ta18-23.bin",  0x20000, 0x8000, CRC(3fd19cf7) SHA1(2e45ab95d19664ed16b19c40bdb8d8c506b98dd1) )
	ROM_LOAD( "ta18-17.bin",  0x28000, 0x8000, CRC(74c64c6e) SHA1(7cbb969c89996476d115f2e55be5a5c5f87c344a) )
	ROM_LOAD( "ta18-19.bin",  0x30000, 0x8000, CRC(c8795fd7) SHA1(ef7aebf21dba248383d5b93cba9620a585e244b9) )
	ROM_LOAD( "ta18-22.bin",  0x38000, 0x8000, CRC(df3a2ff5) SHA1(94bf8968a3d927b410e39d4b6ef28cdfd533179f) )
	ROM_LOAD( "ta18-16.bin",  0x40000, 0x8000, CRC(7244bad0) SHA1(ebd93c82f0b8dfffa905927a6884a61c62ea3879) )
	ROM_LOAD( "ta18-13.bin",  0x48000, 0x8000, CRC(b6b14d46) SHA1(065cfb39c141265fbf92abff67a5efe8e258c2ce) )
	ROM_LOAD( "ta18-21.bin",  0x50000, 0x8000, CRC(c95e009b) SHA1(d45a247d4ebf8587a2cd30c83444cc7bd17a3534) )
	ROM_LOAD( "ta18-15.bin",  0x58000, 0x8000, CRC(a5d61d01) SHA1(9bf1f0b8296667db31ff1c34e28c8eda3ce9f7c3) )

	ROM_REGION( 0x18000, "adpcm", 0 )
	ROM_LOAD( "ta18-09.bin",  0x00000, 0x8000, CRC(07ed4705) SHA1(6fd4b78ca846fa602504f06f3105b2da03bcd00c) )
	ROM_LOAD( "ta18-08.bin",  0x08000, 0x8000, CRC(c9312613) SHA1(fbbdf7c56c34cbee42984e41fcf2a21da2b87a31) )
	ROM_LOAD( "ta18-07.bin",  0x10000, 0x8000, CRC(02e3f3ed) SHA1(ab09b3af2c4ab9a36eb1273bcc7c788350048554) )
ROM_END

ROM_START( kuniokunb )
	ROM_REGION( 0x10000, "maincpu", 0 ) /* 64k for code + bank switched ROM */
	ROM_LOAD( "ta18-11.bin",  0x00000, 0x8000, CRC(f240f5cd) SHA1(ed6875e8ad2988e88389d4f63ff448d0823c195f) )
	ROM_LOAD( "ta18-10.bin",  0x08000, 0x8000, CRC(a90cf44a) SHA1(6d63d9c29da7b8c5bc391e074b6b8fe6ae3892ae) ) // bootleg

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "n0-5.bin",     0x8000, 0x8000, CRC(3587de3b) SHA1(f82e758254b21eb0c5a02469c72adb86d9577065) )

	ROM_REGION( 0x08000, "chars", 0 )
	ROM_LOAD( "ta18-25.bin",  0x0000, 0x8000, CRC(9bd2bea3) SHA1(fa79c9d4c71c1dbbf0e14cb8d6870f1f94b9af88) )

	ROM_REGION( 0x30000, "tiles", 0 )
	ROM_LOAD( "ta18-01.bin",  0x00000, 0x8000, CRC(daf15024) SHA1(f37de97275f52dfbbad7bf8c82f8108e84bcf63e) )
	ROM_LOAD( "ta18-06.bin",  0x08000, 0x8000, CRC(1f59a248) SHA1(8ab70aa8f0dccbe94240c96835a43b0900d52120) )
	ROM_LOAD( "n7-5.bin",     0x10000, 0x8000, CRC(7ca5a532) SHA1(1110aa1c7562805dd4b298ab2860c66a6cc2685b) )
	ROM_LOAD( "ta18-02.bin",  0x18000, 0x8000, CRC(994c0021) SHA1(9219464decc1b07591d0485502e2bcc0c2d16261) )
	ROM_LOAD( "ta18-04.bin",  0x20000, 0x8000, CRC(55b9e8aa) SHA1(26c91030c53a022c1f1f3131768e8f7ba613168d) )
	ROM_LOAD( "ta18-03.bin",  0x28000, 0x8000, CRC(0475c99a) SHA1(36b7b856e728c68e0dd3ecb844033369a5117270) )

	ROM_REGION( 0x60000, "sprites", 0 )
	ROM_LOAD( "ta18-20.bin",  0x00000, 0x8000, CRC(c7d54139) SHA1(f76d237a6ee8bbcbf344145d31e532834da7c131) )
	ROM_LOAD( "ta18-24.bin",  0x08000, 0x8000, CRC(84677d45) SHA1(cb7fe69e13d2d696acbc464b7584c7514cfc7f85) )
	ROM_LOAD( "ta18-18.bin",  0x10000, 0x8000, CRC(1c770853) SHA1(4fe6051265729a9d36b6d3dd826c3f6dcb4a7a25) )
	ROM_LOAD( "ta18-14.bin",  0x18000, 0x8000, CRC(af656017) SHA1(d395d35fe6d8e281596b2df571099b841f979a97) )
	ROM_LOAD( "ta18-23.bin",  0x20000, 0x8000, CRC(3fd19cf7) SHA1(2e45ab95d19664ed16b19c40bdb8d8c506b98dd1) )
	ROM_LOAD( "ta18-17.bin",  0x28000, 0x8000, CRC(74c64c6e) SHA1(7cbb969c89996476d115f2e55be5a5c5f87c344a) )
	ROM_LOAD( "ta18-19.bin",  0x30000, 0x8000, CRC(c8795fd7) SHA1(ef7aebf21dba248383d5b93cba9620a585e244b9) )
	ROM_LOAD( "ta18-22.bin",  0x38000, 0x8000, CRC(df3a2ff5) SHA1(94bf8968a3d927b410e39d4b6ef28cdfd533179f) )
	ROM_LOAD( "ta18-16.bin",  0x40000, 0x8000, CRC(7244bad0) SHA1(ebd93c82f0b8dfffa905927a6884a61c62ea3879) )
	ROM_LOAD( "ta18-13.bin",  0x48000, 0x8000, CRC(b6b14d46) SHA1(065cfb39c141265fbf92abff67a5efe8e258c2ce) )
	ROM_LOAD( "ta18-21.bin",  0x50000, 0x8000, CRC(c95e009b) SHA1(d45a247d4ebf8587a2cd30c83444cc7bd17a3534) )
	ROM_LOAD( "ta18-15.bin",  0x58000, 0x8000, CRC(a5d61d01) SHA1(9bf1f0b8296667db31ff1c34e28c8eda3ce9f7c3) )

	ROM_REGION( 0x18000, "adpcm", 0 ) /* adpcm */
	ROM_LOAD( "ta18-09.bin",  0x00000, 0x8000, CRC(07ed4705) SHA1(6fd4b78ca846fa602504f06f3105b2da03bcd00c) )
	ROM_LOAD( "ta18-08.bin",  0x08000, 0x8000, CRC(c9312613) SHA1(fbbdf7c56c34cbee42984e41fcf2a21da2b87a31) )
	ROM_LOAD( "ta18-07.bin",  0x10000, 0x8000, CRC(02e3f3ed) SHA1(ab09b3af2c4ab9a36eb1273bcc7c788350048554) )
ROM_END



GAME( 1986, renegade,  0,        renegade,  renegade, renegade_state, 0, ROT0, "Technos Japan (Taito America license)", "Renegade (US)", MACHINE_SUPPORTS_SAVE )
GAME( 1986, kuniokun,  renegade, renegade,  renegade, renegade_state, 0, ROT0, "Technos Japan", "Nekketsu Kouha Kunio-kun (Japan)", MACHINE_SUPPORTS_SAVE )
GAME( 1986, kuniokunb, renegade, kuniokunb, renegade, renegade_state, 0, ROT0, "bootleg", "Nekketsu Kouha Kunio-kun (Japan bootleg)", MACHINE_SUPPORTS_SAVE )
