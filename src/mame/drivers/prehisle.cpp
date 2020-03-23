// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
/***************************************************************************

    Prehistoric Isle in 1930 (World)        (c) 1989 SNK
    Prehistoric Isle in 1930 (USA)          (c) 1989 SNK
    Genshi-Tou 1930's (Japan)               (c) 1989 SNK

    Emulation by Bryan McPhail, mish@tendril.co.uk

***************************************************************************/

#include "emu.h"
#include "includes/prehisle.h"

#include "cpu/z80/z80.h"
#include "cpu/m68000/m68000.h"
#include "sound/3812intf.h"
#include "screen.h"
#include "speaker.h"


/******************************************************************************/

WRITE16_MEMBER(prehisle_state::soundcmd_w)
{
	m_soundlatch->write(space, 0, data & 0xff);
	m_audiocpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

/*******************************************************************************/

static ADDRESS_MAP_START( prehisle_map, AS_PROGRAM, 16, prehisle_state )
	AM_RANGE(0x000000, 0x03ffff) AM_ROM
	AM_RANGE(0x070000, 0x073fff) AM_RAM
	AM_RANGE(0x090000, 0x0907ff) AM_RAM_WRITE(tx_vram_w) AM_SHARE("tx_vram")
	AM_RANGE(0x0a0000, 0x0a07ff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x0b0000, 0x0b3fff) AM_RAM_WRITE(fg_vram_w) AM_SHARE("fg_vram")
	AM_RANGE(0x0d0000, 0x0d07ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x0e0000, 0x0e00ff) AM_READ(control_r)
	AM_RANGE(0x0f0070, 0x0ff071) AM_WRITE(soundcmd_w)
	AM_RANGE(0x0f0000, 0x0ff0ff) AM_WRITE(control_w)
ADDRESS_MAP_END

/******************************************************************************/

WRITE8_MEMBER(prehisle_state::D7759_write_port_0_w)
{
	m_upd7759->port_w(space, 0, data);
	m_upd7759->start_w(0);
	m_upd7759->start_w(1);
}

WRITE8_MEMBER(prehisle_state::D7759_upd_reset_w)
{
	m_upd7759->reset_w(data & 0x80);
}

static ADDRESS_MAP_START( prehisle_sound_map, AS_PROGRAM, 8, prehisle_state )
	AM_RANGE(0x0000, 0xefff) AM_ROM
	AM_RANGE(0xf000, 0xf7ff) AM_RAM
	AM_RANGE(0xf800, 0xf800) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
	AM_RANGE(0xf800, 0xf800) AM_WRITENOP    // ???
ADDRESS_MAP_END

static ADDRESS_MAP_START( prehisle_sound_io_map, AS_IO, 8, prehisle_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x00) AM_DEVREADWRITE("ymsnd", ym3812_device, status_port_r, control_port_w)
	AM_RANGE(0x20, 0x20) AM_DEVWRITE("ymsnd", ym3812_device, write_port_w)
	AM_RANGE(0x40, 0x40) AM_WRITE(D7759_write_port_0_w)
	AM_RANGE(0x80, 0x80) AM_WRITE(D7759_upd_reset_w)
ADDRESS_MAP_END

/******************************************************************************/

static INPUT_PORTS_START( prehisle )
	PORT_START("P1")    /* Player 1 controls */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START1 )

	PORT_START("P2")    /* Player 2 controls */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START2  )

	PORT_START("COIN")  /* coin */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_SERVICE_NO_TOGGLE( 0x08, IP_ACTIVE_LOW )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0xe0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW0")  /* Dip switches */
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Flip_Screen ) )  PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Level_Select ) ) PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Bonus_Life ) )   PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x04, "Only Twice" )
	PORT_DIPSETTING(    0x00, "Always" )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )      PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Coinage ) )      PORT_DIPLOCATION("SW1:5,6")
	PORT_DIPSETTING(    0x00, "A 4C/1C B 1C/4C" )
	PORT_DIPSETTING(    0x10, "A 3C/1C B 1C/3C" )
	PORT_DIPSETTING(    0x20, "A 2C/1C B 1C/2C" )
	PORT_DIPSETTING(    0x30, DEF_STR( 1C_1C ) )
	PORT_DIPNAME( 0xc0, 0xc0, DEF_STR( Lives ) )        PORT_DIPLOCATION("SW1:7,8")
	PORT_DIPSETTING(    0x80, "2" )
	PORT_DIPSETTING(    0xc0, "3" )
	PORT_DIPSETTING(    0x40, "4" )
	PORT_DIPSETTING(    0x00, "5" )

	PORT_START("DSW1")
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Difficulty ) )   PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x02, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x03, DEF_STR( Standard ) )
	PORT_DIPSETTING(    0x01, "Middle" )
	PORT_DIPSETTING(    0x00, DEF_STR( Difficult ) )
	PORT_DIPNAME( 0x0c, 0x0c, "Game Mode" )             PORT_DIPLOCATION("SW2:3,4")
	PORT_DIPSETTING(    0x08, "Demo Sounds Off" )
	PORT_DIPSETTING(    0x0c, "Demo Sounds On" )
	PORT_DIPSETTING(    0x00, "Freeze" )
	PORT_DIPSETTING(    0x04, "Infinite Lives" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Bonus_Life ) )   PORT_DIPLOCATION("SW2:5,6")
	PORT_DIPSETTING(    0x30, "100K 200K" )
	PORT_DIPSETTING(    0x20, "150K 300K" )
	PORT_DIPSETTING(    0x10, "300K 500K" )
	PORT_DIPSETTING(    0x00, DEF_STR( None ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SW2:7")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Yes ) )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_CUSTOM ) PORT_VBLANK("screen")
INPUT_PORTS_END

/******************************************************************************/

static const gfx_layout charlayout =
{
	8,8,    /* 8*8 characters */
	1024,
	4,      /* 4 bits per pixel */
	{ 0, 1, 2, 3 },
	{ 0, 4, 8, 12, 16, 20, 24, 28},
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	32*8    /* every char takes 32 consecutive bytes */
};

static const gfx_layout tilelayout =
{
	16,16,  /* 16*16 sprites */
	0x800,
	4,  /* 4 bits per pixel */
	{ 0, 1, 2, 3 },
	{ 0,4,8,12,16,20,24,28,
		0+64*8,4+64*8,8+64*8,12+64*8,16+64*8,20+64*8,24+64*8,28+64*8 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
		8*32, 9*32, 10*32, 11*32, 12*32, 13*32, 14*32, 15*32 },
	128*8   /* every sprite takes 64 consecutive bytes */
};

static const gfx_layout spritelayout =
{
	16,16,  /* 16*16 sprites */
	5120,
	4,  /* 4 bits per pixel */
	{ 0, 1, 2, 3 },
	{ 0,4,8,12,16,20,24,28,
		0+64*8,4+64*8,8+64*8,12+64*8,16+64*8,20+64*8,24+64*8,28+64*8 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
		8*32, 9*32, 10*32, 11*32, 12*32, 13*32, 14*32, 15*32 },
	128*8   /* every sprite takes 64 consecutive bytes */
};

static GFXDECODE_START( prehisle )
	GFXDECODE_ENTRY( "chars",   0, charlayout,  0, 16 )
	GFXDECODE_ENTRY( "bgtiles", 0, tilelayout, 768, 16 )
	GFXDECODE_ENTRY( "fgtiles", 0, tilelayout, 512, 16 )
	GFXDECODE_ENTRY( "sprites", 0, spritelayout, 256, 16 )
GFXDECODE_END

/******************************************************************************/

static MACHINE_CONFIG_START( prehisle )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, XTAL_18MHz/2)   /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(prehisle_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", prehisle_state,  irq4_line_hold)

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_4MHz)    /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(prehisle_sound_map)
	MCFG_CPU_IO_MAP(prehisle_sound_io_map)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	// the screen parameters are guessed but should be accurate. They
	// give a theoretical refresh rate of 59.1856Hz while the measured
	// rate on a snk68.c with very similar hardware board is 59.16Hz.
	MCFG_SCREEN_RAW_PARAMS(XTAL_24MHz/4, 384, 0, 256, 264, 16, 240)
	MCFG_SCREEN_UPDATE_DRIVER(prehisle_state, screen_update_prehisle)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", prehisle)
	MCFG_PALETTE_ADD("palette", 1024)
	MCFG_PALETTE_FORMAT(RRRRGGGGBBBBxxxx)


	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ymsnd", YM3812, XTAL_4MHz)  /* verified on pcb */
	MCFG_YM3812_IRQ_HANDLER(INPUTLINE("audiocpu", 0))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_SOUND_ADD("upd", UPD7759, UPD7759_STANDARD_CLOCK)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.90)
MACHINE_CONFIG_END

/******************************************************************************/

ROM_START( prehisle )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "gt-e2.2h", 0x00000, 0x20000, CRC(7083245a) SHA1(c4f72440e3fb130c8c44224c958bf70c61e8c34e) ) /* red "E" stamped on printed label */
	ROM_LOAD16_BYTE( "gt-e3.3h", 0x00001, 0x20000, CRC(6d8cdf58) SHA1(0078e54db899132d2b1244aed0b974173717f82e) ) /* red "E" stamped on printed label */

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* Sound CPU */
	ROM_LOAD( "gt1.1",  0x000000, 0x10000, CRC(80a4c093) SHA1(abe59e43259eb80b504bd5541f58cd0e5eb998ab) )

	ROM_REGION( 0x008000, "chars", 0 )
	ROM_LOAD( "gt15.b15",   0x000000, 0x08000, CRC(ac652412) SHA1(916c04c3a8a7bfb961313ab73c0a27d7f5e48de1) )

	ROM_REGION( 0x040000, "bgtiles", 0 )
	ROM_LOAD( "pi8914.b14", 0x000000, 0x40000, CRC(207d6187) SHA1(505dfd1424b894e7b898f91b89f021ddde433c48) )

	ROM_REGION( 0x040000, "fgtiles", 0 )
	ROM_LOAD( "pi8916.h16", 0x000000, 0x40000, CRC(7cffe0f6) SHA1(aba08617964fc425418b098be5167021768bd47c) )

	ROM_REGION( 0x0a0000, "sprites", 0 )
	ROM_LOAD( "pi8910.k14", 0x000000, 0x80000, CRC(5a101b0b) SHA1(9645ab1f8d058cf2c6c42ccb4ce92a9b5db10c51) )
	ROM_LOAD( "gt5.5",      0x080000, 0x20000, CRC(3d3ab273) SHA1(b5706ada9eb2c22fcc0ac8ede2d2ee02ee853191) )

	ROM_REGION( 0x10000, "bgtilemap", 0 )    /* background tilemaps */
	ROM_LOAD( "gt11.11",  0x000000, 0x10000, CRC(b4f0fcf0) SHA1(b81cc0b6e3e6f5616789bb3e77807dc0ef718a38) )

	ROM_REGION( 0x20000, "upd", 0 ) /* ADPCM samples */
	ROM_LOAD( "gt4.4",  0x000000, 0x20000, CRC(85dfb9ec) SHA1(78c865e7ccffddb71dcddccab358fa945f521f25) )
ROM_END

ROM_START( prehisleu )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "gt-u2.2h", 0x00000, 0x20000, CRC(a14f49bb) SHA1(6b39a894c3d3862be349a58c748d2d763d5a269c) ) /* red "U" stamped on printed label */
	ROM_LOAD16_BYTE( "gt-u3.3h", 0x00001, 0x20000, CRC(f165757e) SHA1(26cf369fed1713deec182852d76fe014ed46d6ac) ) /* red "U" stamped on printed label */

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* Sound CPU */
	ROM_LOAD( "gt1.1",  0x000000, 0x10000, CRC(80a4c093) SHA1(abe59e43259eb80b504bd5541f58cd0e5eb998ab) )

	ROM_REGION( 0x008000, "chars", 0 )
	ROM_LOAD( "gt15.b15",   0x000000, 0x08000, CRC(ac652412) SHA1(916c04c3a8a7bfb961313ab73c0a27d7f5e48de1) )

	ROM_REGION( 0x040000, "bgtiles", 0 )
	ROM_LOAD( "pi8914.b14", 0x000000, 0x40000, CRC(207d6187) SHA1(505dfd1424b894e7b898f91b89f021ddde433c48) )

	ROM_REGION( 0x040000, "fgtiles", 0 )
	ROM_LOAD( "pi8916.h16", 0x000000, 0x40000, CRC(7cffe0f6) SHA1(aba08617964fc425418b098be5167021768bd47c) )

	ROM_REGION( 0x0a0000, "sprites", 0 )
	ROM_LOAD( "pi8910.k14", 0x000000, 0x80000, CRC(5a101b0b) SHA1(9645ab1f8d058cf2c6c42ccb4ce92a9b5db10c51) )
	ROM_LOAD( "gt5.5",      0x080000, 0x20000, CRC(3d3ab273) SHA1(b5706ada9eb2c22fcc0ac8ede2d2ee02ee853191) )

	ROM_REGION( 0x10000, "bgtilemap", 0 )    /* background tilemaps */
	ROM_LOAD( "gt11.11",  0x000000, 0x10000, CRC(b4f0fcf0) SHA1(b81cc0b6e3e6f5616789bb3e77807dc0ef718a38) )

	ROM_REGION( 0x20000, "upd", 0 ) /* ADPCM samples */
	ROM_LOAD( "gt4.4",  0x000000, 0x20000, CRC(85dfb9ec) SHA1(78c865e7ccffddb71dcddccab358fa945f521f25) )
ROM_END

ROM_START( prehislek )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "gt-k2.2h", 0x00000, 0x20000, CRC(f2d3544d) SHA1(28d41a81ac12ef951610ba0aa70945c069d69d75) ) /* red "K" stamped on printed label */
	ROM_LOAD16_BYTE( "gt-k3.3h", 0x00001, 0x20000, CRC(ebf7439b) SHA1(76fcad47bc8ae371ecf265fd378e2c4856d39c7f) ) /* red "K" stamped on printed label */

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* Sound CPU */
	ROM_LOAD( "gt1.1",  0x000000, 0x10000, CRC(80a4c093) SHA1(abe59e43259eb80b504bd5541f58cd0e5eb998ab) )

	ROM_REGION( 0x008000, "chars", 0 )
	ROM_LOAD( "gt15.b15",   0x000000, 0x08000, BAD_DUMP CRC(ac652412) SHA1(916c04c3a8a7bfb961313ab73c0a27d7f5e48de1) ) // not dumped, missing korean text

	ROM_REGION( 0x040000, "bgtiles", 0 )
	ROM_LOAD( "pi8914.b14", 0x000000, 0x40000, CRC(207d6187) SHA1(505dfd1424b894e7b898f91b89f021ddde433c48) )

	ROM_REGION( 0x040000, "fgtiles", 0 )
	ROM_LOAD( "pi8916.h16", 0x000000, 0x40000, CRC(7cffe0f6) SHA1(aba08617964fc425418b098be5167021768bd47c) )

	ROM_REGION( 0x0a0000, "sprites", 0 )
	ROM_LOAD( "pi8910.k14", 0x000000, 0x80000, CRC(5a101b0b) SHA1(9645ab1f8d058cf2c6c42ccb4ce92a9b5db10c51) )
	ROM_LOAD( "gt5.5",      0x080000, 0x20000, CRC(3d3ab273) SHA1(b5706ada9eb2c22fcc0ac8ede2d2ee02ee853191) )

	ROM_REGION( 0x10000, "bgtilemap", 0 )    /* background tilemaps */
	ROM_LOAD( "gt11.11",  0x000000, 0x10000, CRC(b4f0fcf0) SHA1(b81cc0b6e3e6f5616789bb3e77807dc0ef718a38) )

	ROM_REGION( 0x20000, "upd", 0 ) /* ADPCM samples */
	ROM_LOAD( "gt4.4",  0x000000, 0x20000, CRC(85dfb9ec) SHA1(78c865e7ccffddb71dcddccab358fa945f521f25) )
ROM_END

ROM_START( gensitou )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "gt-j2.2h", 0x00000, 0x20000, CRC(a2da0b6b) SHA1(d102118f83b96094fd4ea4b3468713c4946c949d) ) /* red "J" stamped on printed label */
	ROM_LOAD16_BYTE( "gt-j3.3h", 0x00001, 0x20000, CRC(c1a0ae8e) SHA1(2c9643abfd71edf8612e63d69cea4fbc19aad19d) ) /* red "J" stamped on printed label */

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* Sound CPU */
	ROM_LOAD( "gt1.1",  0x000000, 0x10000, CRC(80a4c093) SHA1(abe59e43259eb80b504bd5541f58cd0e5eb998ab) )

	ROM_REGION( 0x008000, "chars", 0 )
	ROM_LOAD( "gt15.b15",   0x000000, 0x08000, CRC(ac652412) SHA1(916c04c3a8a7bfb961313ab73c0a27d7f5e48de1) )

	ROM_REGION( 0x040000, "bgtiles", 0 )
	ROM_LOAD( "pi8914.b14", 0x000000, 0x40000, CRC(207d6187) SHA1(505dfd1424b894e7b898f91b89f021ddde433c48) )

	ROM_REGION( 0x040000, "fgtiles", 0 )
	ROM_LOAD( "pi8916.h16", 0x000000, 0x40000, CRC(7cffe0f6) SHA1(aba08617964fc425418b098be5167021768bd47c) )

	ROM_REGION( 0x0a0000, "sprites", 0 )
	ROM_LOAD( "pi8910.k14", 0x000000, 0x80000, CRC(5a101b0b) SHA1(9645ab1f8d058cf2c6c42ccb4ce92a9b5db10c51) )
	ROM_LOAD( "gt5.5",      0x080000, 0x20000, CRC(3d3ab273) SHA1(b5706ada9eb2c22fcc0ac8ede2d2ee02ee853191) )

	ROM_REGION( 0x10000, "bgtilemap", 0 )    /* background tilemaps */
	ROM_LOAD( "gt11.11",  0x000000, 0x10000, CRC(b4f0fcf0) SHA1(b81cc0b6e3e6f5616789bb3e77807dc0ef718a38) )

	ROM_REGION( 0x20000, "upd", 0 ) /* ADPCM samples */
	ROM_LOAD( "gt4.4",  0x000000, 0x20000, CRC(85dfb9ec) SHA1(78c865e7ccffddb71dcddccab358fa945f521f25) )
ROM_END

// world bootleg using 64k*8 UVEPROMs, program and sound unchanged, sprites and background tilemaps altered
ROM_START( prehisleb )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "u_h1.bin", 0x00000, 0x10000, CRC(04c1703b) SHA1(36aa9b8cd11621faa094af4dd2fb5a0e59318a5e) )
	ROM_LOAD16_BYTE( "u_h3.bin", 0x00001, 0x10000, CRC(62f04cd1) SHA1(9506e18a7847362128e06781e783fdb1f562e502) )
	ROM_LOAD16_BYTE( "u_j2.bin", 0x20000, 0x10000, CRC(7b12501d) SHA1(678d32f70d86807449ffe617c7c6e257d308d8af) )
	ROM_LOAD16_BYTE( "u_j3.bin", 0x20001, 0x10000, CRC(2a86f7c4) SHA1(5bca393f6edfcd41e1803ea1062497752fd400a9) )

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* Sound CPU */
	ROM_LOAD( "u_e12.bin", 0x00000, 0x10000, CRC(80a4c093) SHA1(abe59e43259eb80b504bd5541f58cd0e5eb998ab) )

	ROM_REGION( 0x008000, "chars", 0 )
	ROM_LOAD( "l_a17.bin", 0x00000, 0x08000, CRC(ac652412) SHA1(916c04c3a8a7bfb961313ab73c0a27d7f5e48de1) )

	ROM_REGION( 0x040000, "bgtiles", 0 )
	ROM_LOAD( "l_b17.bin", 0x00000, 0x10000, CRC(65a22ffc) SHA1(0122c15f9c948bd6a12d44f71178d0a8b7f38c2e) )
	ROM_LOAD( "l_b16.bin", 0x10000, 0x10000, CRC(b1e1f527) SHA1(07d88d4a1f198bd5e37dcb1521904c5f8d851f4d) )
	ROM_LOAD( "l_b14.bin", 0x20000, 0x10000, CRC(28e94d40) SHA1(e77187040b5c5c7354088aa1173b23493cf26b78) )
	ROM_LOAD( "l_b13.bin", 0x30000, 0x10000, CRC(4dbb557a) SHA1(af07074dae121264018f2f6f3489cce243bfd3c0) )

	ROM_REGION( 0x040000, "fgtiles", 0 )
	ROM_LOAD( "l_h17.bin", 0x00000, 0x10000, CRC(79c42316) SHA1(7a1c72c9146ce50d9c24ec4f3ae210103a95c2eb) )
	ROM_LOAD( "l_h15.bin", 0x10000, 0x10000, CRC(50e31fb0) SHA1(043181041354e3af07b6b32fc6192aae9e49d869) )
	ROM_LOAD( "l_f17.bin", 0x20000, 0x10000, CRC(2af1739d) SHA1(e17b88ee525247100b038f2200ad5a1ce4e71cb2) )
	ROM_LOAD( "l_f15.bin", 0x30000, 0x10000, CRC(cac11327) SHA1(c0feb6f3d9b8bba1dab66142fa44269bda579443) )

	ROM_REGION( 0x0a0000, "sprites", 0 )
	ROM_LOAD( "u_k12.bin", 0x00000, 0x10000, CRC(4b0215f0) SHA1(340e68e9b9603829a200ad1ff7c0b373d39ca4dc) )
	ROM_LOAD( "u_k13.bin", 0x10000, 0x10000, CRC(68b8a698) SHA1(ff87c47cb600bacdb50b2e8ad87090a0e0146d12) )
	ROM_LOAD( "u_j4.bin",  0x20000, 0x10000, CRC(06ce7b57) SHA1(d19f35405b34bb43a2ca341c020c14de4c8474d6) )
	ROM_LOAD( "u_j5.bin",  0x30000, 0x10000, CRC(2ee8b401) SHA1(6f4a3ff75daae790872477a600c9e61332f74a46) )
	ROM_LOAD( "u_j7.bin",  0x40000, 0x10000, CRC(35656cbc) SHA1(bed0b2bfb9bd8487718a14d5388c61740d0e0e3a) )
	ROM_LOAD( "u_j8.bin",  0x50000, 0x10000, CRC(1e7e9336) SHA1(28b13ab7e9a0bb806af8fe3dbc2b100b93b29c5c) )
	ROM_LOAD( "u_j10.bin", 0x60000, 0x10000, CRC(785bf046) SHA1(5ab3f883643de6c59e764775b11b275989437fa2) )
	ROM_LOAD( "u_j11.bin", 0x70000, 0x10000, CRC(c306b9fa) SHA1(58c8d64dd7ae80b5d21d289757de442ac8e9264c) )
	ROM_LOAD( "u_j12.bin" ,0x80000, 0x10000, CRC(5ba5bbed) SHA1(6af3503e0277a926815afb973d67c4ad7a0427d1) )
	ROM_LOAD( "u_j13.bin", 0x90000, 0x10000, CRC(007dee47) SHA1(e45ce52a471783864cc2704b3b0462c32ddf7e52) ) // modified by bootleggers

	ROM_REGION( 0x10000, "bgtilemap", 0 )    /* background tilemaps */
	ROM_LOAD( "l_a6.bin",  0x00000, 0x10000, CRC(e2b9a44b) SHA1(4a1be44c19a724727218bbdc120bafbbe095747a) ) // modified by bootleggers

	ROM_REGION( 0x20000, "upd", 0 ) /* ADPCM samples */
	ROM_LOAD( "u_f14.bin", 0x00000, 0x10000, CRC(2fb32933) SHA1(2cea86dfe9a6a0b2de34c3c952c625ad30a7ebea) )
	ROM_LOAD( "u_j14.bin", 0x10000, 0x10000, CRC(32d5f7c9) SHA1(23abc82f83296c62320a047b9f63032a7f07bf6d) )
ROM_END

/******************************************************************************/


GAME( 1989, prehisle,  0,        prehisle, prehisle, prehisle_state, 0, ROT0, "SNK",                  "Prehistoric Isle in 1930 (World)",          MACHINE_SUPPORTS_SAVE )
GAME( 1989, prehisleu, prehisle, prehisle, prehisle, prehisle_state, 0, ROT0, "SNK",                  "Prehistoric Isle in 1930 (US)",             MACHINE_SUPPORTS_SAVE )
GAME( 1989, prehislek, prehisle, prehisle, prehisle, prehisle_state, 0, ROT0, "SNK (Victor license)", "Prehistoric Isle in 1930 (Korea)",          MACHINE_SUPPORTS_SAVE )
GAME( 1989, gensitou,  prehisle, prehisle, prehisle, prehisle_state, 0, ROT0, "SNK",                  "Genshi-Tou 1930's",                         MACHINE_SUPPORTS_SAVE )
GAME( 1989, prehisleb, prehisle, prehisle, prehisle, prehisle_state, 0, ROT0, "bootleg",              "Prehistoric Isle in 1930 (World, bootleg)", MACHINE_SUPPORTS_SAVE )
