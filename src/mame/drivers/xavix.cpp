// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

    Skeleton driver for XaviX TV PNP console and childs (Let's! Play TV Classic)

    CPU is an M6502 derivative where opcode 0x22 has 3 bytes of operands.
    Definitely not: 65C816 or Mitsu M740.

    Code at F34F is thus:

    F34F:  STA $6200,X
    F352:  INX
    F353:  BNE $F34F
    F355:  UNK 12 FA 80
    F359:  UNK 12 A8 80
    F35D:  UNK 12 1B 80
    F361:  SEC
    F362:  LDA #$CD
    F364:  SBC #$CA

    later on

    F3C9:  UNK 00 E4 C4

    TODO:
    - identify CPU
    - figure out ROM banking

    Notes from http://www.videogameconsolelibrary.com/pg00-xavix.htm#page=reviews (thanks Guru!)

    XaviXPORT arrived on the scene with 3 game titles (XaviX Tennis, XaviX Bowling and XaviX Baseball) using their
    original XaviX Multiprocessor.  This proprietary chip is reported to contain an 8-bit high speed central processing
    unit (6502) at 21 MHz, picture processor, sound processor, DMA controller, 1K bytes high speed RAM, universal timer,
    AD/Converter and I/O device control.  Each cartridge comes with a wireless peripheral to be used with the game (Baseball Bat,
    Tennis Racquet, etc.) that requires "AA" batteries.  The XaviXPORT system retailed for $79.99 USD with the cartridges
    retailing for $49.99 USD.

    The following year at CES 2005, SSD COMPANY LIMITED introduced two new XaviXPORT titles (XaviX Golf and XaviX Bass Fishing) each
    containing the upgraded "Super XaviX".  This new chip is said to sport a 16-bit high central processing unit (65816) at 43 MHz.
    SSD COMPANY LIMITED is already working on their next chip called "XaviX II" that is said to be a 32-bit RISC processor
    with 3D capabilities.

***************************************************************************/


#include "emu.h"
#include "cpu/g65816/g65816.h"
#include "cpu/m6502/m6502.h"
//#include "sound/ay8910.h"
#include "screen.h"
#include "speaker.h"


#define MAIN_CLOCK XTAL_21_4772MHz

class xavix_state : public driver_device
{
public:
	xavix_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu")
	{ }

	// devices
	required_device<cpu_device> m_maincpu;

	// screen updates
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void video_start() override;
};

void xavix_state::video_start()
{
}

uint32_t xavix_state::screen_update( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	return 0;
}

static ADDRESS_MAP_START( xavix_map, AS_PROGRAM, 8, xavix_state )
	AM_RANGE(0x000000, 0x0001ff) AM_RAM
	AM_RANGE(0x00f000, 0x00ffff) AM_ROM AM_REGION("bios", 0x00f000)
	AM_RANGE(0x800000, 0x9fffff) AM_ROM AM_REGION("bios", 0x000000) // wrong
ADDRESS_MAP_END

static INPUT_PORTS_START( xavix )
	/* dummy active high structure */
	PORT_START("SYSA")
	PORT_DIPNAME( 0x01, 0x00, "SYSA" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	/* dummy active low structure */
	PORT_START("DSWA")
	PORT_DIPNAME( 0x01, 0x01, "DSWA" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END

/* correct, 4bpp gfxs */
static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ STEP4(0,1) },
	{ 1*4,0*4,3*4,2*4,5*4,4*4,7*4,6*4 },
	{ STEP8(0,4*8) },
	8*8*4
};

static GFXDECODE_START( xavix )
	GFXDECODE_ENTRY( "bios", 0, charlayout,     0, 1 )
GFXDECODE_END


void xavix_state::machine_start()
{
}

void xavix_state::machine_reset()
{
}


static MACHINE_CONFIG_START( xavix )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",G65816,MAIN_CLOCK/4)
	MCFG_CPU_PROGRAM_MAP(xavix_map)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_UPDATE_DRIVER(xavix_state, screen_update)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 0*8, 32*8-1)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", xavix)

	MCFG_PALETTE_ADD("palette", 16)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
//  MCFG_SOUND_ADD("aysnd", AY8910, MAIN_CLOCK/4)
//  MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.30)
MACHINE_CONFIG_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( taitons1 )
	ROM_REGION( 0x200000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "taitonostalgia1.u3", 0x000000, 0x200000, CRC(25bd8c67) SHA1(a109cd2da6aa4596e3ca3abd1afce2d0001a473f) )
ROM_END

CONS( 2006, taitons1,  0,   0,  xavix,  xavix, xavix_state, 0, "Bandai / SSD Company LTD / Taito", "Let's! TV Play Classic - Taito Nostalgia 1", MACHINE_IS_SKELETON )
