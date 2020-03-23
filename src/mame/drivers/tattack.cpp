// license:BSD-3-Clause
// copyright-holders:Tomasz Slanina
/****************************************************************************
    Time Attacker

    driver by Tomasz Slanina analog[at]op.pl

    Z80A,
    xtal 8MHz,
    dipsw 8-position x2,
    volume pots x6,
    2114 ram x5,
    7910CQ + NE555P sound section,
    SN74198N shifter
    no proms

    TODO:
    - colors
    - sound (requires Epson 7910 Multi-Melody emulation)
    - game logic

    Connector pinout from manual

          Solder Side      Parts Side
          -----------      ----------
                  GND   1  GND
                  GND   2  GND
                  GND   3  GND
              Speaker   4  Speaker
                 +12V   5  +12V
              Antenna   6  Antenna
                        7
                        8
                  +5V   9  +5V
                 VR a  10  VR b
         2P VR Center  11  1P VR Center
                 Coin  12  2P Serve
           Coin Alarm  13  1P Start
        Counter (Out)  14  2P Start
              TV Sync  15  1P Serve
      Counter (futei)  16  TV G
         Counter (In)  17  TV B
            Medal Out  18  TV R

****************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "screen.h"


class tattack_state : public driver_device
{
public:
	tattack_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_colorram(*this, "colorram"),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode") { }

	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_colorram;
	tilemap_t *m_tmap;
	DECLARE_DRIVER_INIT(tattack);
	TILE_GET_INFO_MEMBER(get_tile_info);
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(tattack);
	uint32_t screen_update_tattack(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
};



TILE_GET_INFO_MEMBER(tattack_state::get_tile_info)
{
	int code = m_videoram[tile_index];
	int color = m_colorram[tile_index];

	if((color&1 ) || (color>15) )
		logerror("COLOR %i\n",color);

	color>>=1;

	SET_TILE_INFO_MEMBER(0,
		code,
		color,
		0);
}

uint32_t tattack_state::screen_update_tattack(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_tmap->mark_all_dirty();
	m_tmap->draw(screen, bitmap, cliprect, 0,0);
	return 0;
}

void tattack_state::video_start()
{
	m_tmap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(tattack_state::get_tile_info),this),TILEMAP_SCAN_ROWS,8,8,32,32 );
}

static ADDRESS_MAP_START( mem, AS_PROGRAM, 8, tattack_state )
	AM_RANGE(0x0000, 0x0fff) AM_ROM
//  AM_RANGE(0x4000, 0x4000) AM_READNOP $315
	AM_RANGE(0x5000, 0x53ff) AM_RAM AM_SHARE("videoram")
	AM_RANGE(0x7000, 0x73ff) AM_RAM AM_SHARE("colorram")    // color map ? something else .. only bits 1-3 are used
	AM_RANGE(0x6000, 0x6000) AM_READ_PORT("DSW2")
	AM_RANGE(0xa000, 0xa000) AM_READ_PORT("DSW1")       // dsw ? something else ?
	AM_RANGE(0xc000, 0xc000) AM_READ_PORT("INPUTS") AM_WRITENOP
	AM_RANGE(0xc001, 0xc002) AM_WRITENOP                // bit 7 = strobe ($302)
	AM_RANGE(0xc005, 0xc007) AM_WRITENOP
	AM_RANGE(0xe000, 0xe3ff) AM_RAM
ADDRESS_MAP_END

static INPUT_PORTS_START( tattack )
	PORT_START("INPUTS")
	PORT_DIPNAME( 0x01, 0x00, "1-01" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "1-02" )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "1-03" )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "1-04" )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON3 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON1 )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON2 )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_COIN1 )

	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x00, "DSW1 1" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Coin_A ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_2C ) )

	PORT_DIPNAME( 0x04, 0x00, "Time" )
	PORT_DIPSETTING(    0x04, "112" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x08, 0x00, "DSW1 4" )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "DSW1 5" )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, "DSW1 6" )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, "DSW1 7" )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, "DSW1 8" )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x01, 0x00, "DSW2 1" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "DSW2 2" )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "DSW2 3" )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "DSW2 4" )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "DSW2 5" )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, "DSW2 6" )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, "DSW2 7" )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, "DSW2 8" )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END


static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,1),
	1,
	{ 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7},
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8
};



static GFXDECODE_START( tattack )
	GFXDECODE_ENTRY( "gfx1", 0     , charlayout,  0, 8 )
GFXDECODE_END

PALETTE_INIT_MEMBER(tattack_state, tattack)
{
	int i,r,g,b;
	for(i=0;i<8;i++)
	{
		if(i)
		{
			r=(i&1)?0xff:0;
			g=(i&2)?0xff:0;
			b=(i&4)?0xff:0;
		}
		else
			r=g=b=128;

		palette.set_pen_color(2*i,rgb_t(0x00,0x00,0x00));
		palette.set_pen_color(2*i+1,rgb_t(r,g,b));
	}
}


static MACHINE_CONFIG_START( tattack )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, 8000000 / 2)   /* 4 MHz ? */
	MCFG_CPU_PROGRAM_MAP(mem)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", tattack_state,  irq0_line_hold)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500) /* not accurate */)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 0*8, 32*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(tattack_state, screen_update_tattack)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", tattack)
	MCFG_PALETTE_ADD("palette", 16)
	MCFG_PALETTE_INIT_OWNER(tattack_state, tattack)

	/* sound hardware */
	/* Discrete ???? */

MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( tattack )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "rom.9a",     0x0000, 0x1000, CRC(47120994) SHA1(b6e90abbc50cba77df4c0aaf50d1f97b99e33b6d) )

	ROM_REGION( 0x1000, "gfx1", 0 )
	ROM_LOAD( "rom.6c",     0x0000, 0x1000, CRC(88ce45cf) SHA1(c7a43bfc9e9c2aeb75a98f723558bc88e53401a7) )

ROM_END

DRIVER_INIT_MEMBER(tattack_state,tattack)
{
	uint8_t *rom = memregion("maincpu")->base();

	rom[0x1b4]=0;
	rom[0x1b5]=0;

	rom[0x262]=0;
	rom[0x263]=0;
	rom[0x264]=0;

	rom[0x32a]=0;
	rom[0x32b]=0;
	rom[0x32c]=0;

/*
    possible jumps to 0 (protection checks?)

    rom[0x8a]=0;
    rom[0x8b]=0;
    rom[0x8c]=0;

    rom[0x99]=0;
    rom[0x9a]=0;
    rom[0x9b]=0;

    rom[0xd5]=0;
    rom[0xd6]=0;
    rom[0xd7]=0;

    rom[0x65a]=0;
    rom[0x65b]=0;
    rom[0x65c]=0;
*/

}

GAME( 198?, tattack, 0, tattack, tattack, tattack_state, tattack, ROT270, "Shonan", "Time Attacker", MACHINE_NO_SOUND | MACHINE_IMPERFECT_COLORS | MACHINE_NOT_WORKING)
