// license:BSD-3-Clause
// copyright-holders:David Haywood, Pierpaolo Prazzoli
/*************************************************************************
    Quiz Panicuru Fantasy
    (c) 1993 NMK

    Driver by David Haywood and Pierpaolo Prazzoli

    PCB No: QZ93094
    CPU   : TMP68000P-12
    SOUND : Oki M6295
    OSC   : 16.000MHz, 10.000MHz
    RAM   : 62256 (x2), 6116 (x2), 6264 (x4)
    DIPSW : 8 position (x2)
    CUSTOM: NMK112 (QFP64, near ROMs 31,32,4, M6295 sample ROM banking)
            NMK111 (QFP64, 1x input-related near JAMMA, 2x gfx related near ROMs 11,12,21,22)
            NMK903 (QFP44, x2, near ROMs 11,12,21,22)
            NMK005 (QFP64, near DIPs, possible MCU?)

    ROMs  :
            93094-51.127    27c4002     near 68000
            93094-52.126    27c4001     near 68000
            93094-53.125    27c1001     near 68000
            93090-4.56      8M Mask     oki samples
            93090-31.58     8M Mask     oki samples
            93090-32.57     8M Mask     oki samples
            93090-21.10     8M Mask     gfx
            93090-22.9      8M Mask     gfx
            93090-11.2      8M Mask     gfx
            93090-12.1      8M Mask     gfx
            QZ6.88          82s129      prom
            QZ7.99          82s129      prom
            QZ8.121         82s135      prom


Stephh's notes (based on the games M68000 code and some tests) :

  - I'm unsure that DSW1 bit 0 is related to screen flipping, but if you look at code
    at 0x004d9c, there is a write to 0x100014.w before writes to the scroll registers.
  - I can't explain why there is a possibility to have an alternate buttons layout.
    It may be to run the game on other (NMK ?) boards.
  - Press START1 + START2 on startup to enter sort of "test mode" (code at 0x003b0c).
    It shall display system and players inputs, but text seems to be displayed
    with a black color on a black background, so you can't see anything :(
    Furthermore, there is no wait after the values are displayed, and after the 'rts'
    instruction at 0x0083a4, PC branches back to 0x000000 ! Leftover from another game ?

*************************************************************************/

#include "emu.h"
#include "includes/quizpani.h"

#include "cpu/m68000/m68000.h"
#include "sound/okim6295.h"
#include "screen.h"
#include "speaker.h"


static ADDRESS_MAP_START( quizpani_map, AS_PROGRAM, 16, quizpani_state )
	AM_RANGE(0x000000, 0x07ffff) AM_ROM
	AM_RANGE(0x100000, 0x100001) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x100002, 0x100003) AM_READ_PORT("P1_P2")
	AM_RANGE(0x100008, 0x100009) AM_READ_PORT("DSW1")
	AM_RANGE(0x10000a, 0x10000b) AM_READ_PORT("DSW2")
	AM_RANGE(0x100014, 0x100015) AM_WRITENOP /* screen flipping? */
	AM_RANGE(0x100016, 0x100017) AM_WRITENOP /* IRQ enable? */
	AM_RANGE(0x100018, 0x100019) AM_WRITE(tilesbank_w)
	AM_RANGE(0x104000, 0x104001) AM_DEVREADWRITE8("oki", okim6295_device, read, write, 0x00ff)
	AM_RANGE(0x104020, 0x104027) AM_DEVWRITE8("nmk112", nmk112_device, okibank_w, 0x00ff)
	AM_RANGE(0x108000, 0x1083ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x108400, 0x1085ff) AM_WRITENOP
	AM_RANGE(0x10c000, 0x10c007) AM_RAM AM_SHARE("scrollreg")
	AM_RANGE(0x10c008, 0x10c403) AM_WRITENOP
	AM_RANGE(0x110000, 0x113fff) AM_RAM_WRITE(bg_videoram_w) AM_SHARE("bg_videoram")
	AM_RANGE(0x11c000, 0x11ffff) AM_RAM_WRITE(txt_videoram_w) AM_SHARE("txt_videoram")
	AM_RANGE(0x180000, 0x18ffff) AM_RAM
	AM_RANGE(0x200000, 0x33ffff) AM_ROM
ADDRESS_MAP_END


/* verified from M68000 code */
static INPUT_PORTS_START( quizpani )
	PORT_START("SYSTEM")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P1_P2")
	/* "Standard" buttons layout */
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0002)
	/* "Alternate" buttons layout */
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_UNKNOWN ) /* dup. B1 P1 */   PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_UNKNOWN ) /* dup. B1 P1 */   PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_UNKNOWN ) /* dup. B1 P1 */   PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(1)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_UNKNOWN ) /* dup. B1 P2 */   PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_UNKNOWN ) /* dup. B1 P2 */   PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_UNKNOWN ) /* dup. B1 P2 */   PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(2)     PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )                    PORT_CONDITION("DSW1",0x0002,EQUALS,0x0000)

	PORT_START("DSW1")
	PORT_DIPNAME( 0x0001, 0x0001, DEF_STR( Flip_Screen ) )  /* guess, but check code at 0x004d9c and write to 0x100014 */
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "Buttons Layout" )        /* check code at 0x005d80 */
	PORT_DIPSETTING(      0x0002, DEF_STR( Standard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Alternate ) )
	PORT_DIPNAME( 0x0004, 0x0018, DEF_STR( Difficulty ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x0018, DEF_STR( Medium ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0030, 0x0030, DEF_STR( Lives ) )
	PORT_DIPSETTING(      0x0000, "1" )
	PORT_DIPSETTING(      0x0020, "2" )
	PORT_DIPSETTING(      0x0030, "3" )
	PORT_DIPSETTING(      0x0010, "4" )
	PORT_DIPUNKNOWN( 0x0040, 0x0040 )                       /* check code at 0x0083a6 - never called ? */
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Demo_Sounds ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( On ) )

	PORT_START("DSW2")
	PORT_DIPUNUSED( 0x0001, 0x0001 )
	PORT_DIPNAME( 0x0002, 0x0002, "Sounds/Music" )          /* check code at 0x003ab4 */
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( On ) )
	PORT_DIPNAME( 0x001c, 0x001c, DEF_STR( Coin_B ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0018, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x001c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x000c, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0014, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ) )
	PORT_DIPNAME( 0x00e0, 0x00e0, DEF_STR( Coin_A ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x00c0, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x00e0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x0060, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x00a0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ) )
INPUT_PORTS_END

static const gfx_layout tilelayout =
{
	16,16,
	RGN_FRAC(1,1),
	4,
	{ 0, 1, 2, 3 },
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4,
			16*32+0*4, 16*32+1*4, 16*32+2*4, 16*32+3*4, 16*32+4*4, 16*32+5*4, 16*32+6*4, 16*32+7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
			8*32, 9*32, 10*32, 11*32, 12*32, 13*32, 14*32, 15*32 },
	32*32
};

static GFXDECODE_START( quizpani )
	GFXDECODE_ENTRY( "gfx1", 0, tilelayout, 0x100, 16 ) /* Background */
	GFXDECODE_ENTRY( "gfx2", 0, tilelayout, 0x000, 16 ) /* Text */
GFXDECODE_END


static MACHINE_CONFIG_START( quizpani )
	MCFG_CPU_ADD("maincpu", M68000, 10000000)
	MCFG_CPU_PROGRAM_MAP(quizpani_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", quizpani_state,  irq4_line_hold)
	MCFG_CPU_PERIODIC_INT_DRIVER(quizpani_state, irq1_line_hold, 164) // music tempo

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", quizpani)
	MCFG_PALETTE_ADD("palette", 0x200)
	MCFG_PALETTE_FORMAT(RRRRGGGGBBBBRGBx)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(64*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 48*8-1, 0*8, 28*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(quizpani_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")


	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_OKIM6295_ADD("oki", 16000000/4, PIN7_LOW)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_DEVICE_ADD("nmk112", NMK112, 0)
	MCFG_NMK112_ROM0("oki")
MACHINE_CONFIG_END

ROM_START( quizpani )
	ROM_REGION( 0x340000, "maincpu", 0 ) /* 68000 Code */
	ROM_LOAD16_WORD_SWAP( "93094-51.127", 0x000000, 0x080000, CRC(2b7a29d4) SHA1(f87b875e69410745ee46d5d94b6c28e5417afb0d) )
	/* No EVEN rom */
	ROM_LOAD16_BYTE( "93094-52.126",      0x200001, 0x080000, CRC(0617524e) SHA1(91ab5cb8a605c37c92632cf007ddb67172cc9863) )
	/* No EVEN rom */
	ROM_LOAD16_BYTE( "93094-53.125",      0x300001, 0x020000, CRC(7e0ab49c) SHA1(dd10f723ef74f3153e04b1a271b8761585799aa6) )

	ROM_REGION( 0x200000, "gfx1", 0 )
	ROM_LOAD( "93090-11.2",  0x000000, 0x100000, CRC(4b3ab155) SHA1(fc1210853ca262c42b927689cb8f04aca15de7d6) )
	ROM_LOAD( "93090-12.1",  0x100000, 0x100000, CRC(3f2ebfa5) SHA1(1c935d566f3980483356264a9216f9bf298bb815) )

	ROM_REGION( 0x200000, "gfx2", 0 )
	ROM_LOAD( "93090-21.10", 0x000000, 0x100000, CRC(63754242) SHA1(3698b89d8515b45b9bc0fff87ca94ab5c2b3d53a) )
	ROM_LOAD( "93090-22.9",  0x100000, 0x100000, CRC(93382cd3) SHA1(6527e92f696c21aae65d008bb237231eaba7a105) )

	ROM_REGION( 0x340000, "oki", 0 )
	ROM_LOAD( "93090-31.58", 0x040000, 0x100000, CRC(1cce0e13) SHA1(43816762e7907a8ff4b5a7b8da9f799b5baa64d5) )
	ROM_LOAD( "93090-32.57", 0x140000, 0x100000, CRC(5d38f62e) SHA1(22fe95de6e1de1be0cec73b8163ab4283f2b8186) )
	ROM_LOAD( "93090-4.56",  0x240000, 0x100000, CRC(ee370ed6) SHA1(9b1edfada5805014aa23d28d0c70227728b0e04f) )

	ROM_REGION( 0x300, "proms", 0 )
	ROM_LOAD( "qz6.88",  0x000, 0x100, CRC(19dbbad2) SHA1(ebf7950d1869ca3bc1e72228505fbc17d095746a) ) /* unknown */
	ROM_LOAD( "qz7.99",  0x100, 0x100, CRC(1f802af1) SHA1(617bb7e5105ac202b5a8cf83c8c66178b91099e0) ) /* unknown */
	ROM_LOAD( "qz8.121", 0x200, 0x100, CRC(b4c19741) SHA1(a6d3686bad6ef2336463b89bc2d249003d9b4bcc) ) /* unknown */
ROM_END

GAME( 1993, quizpani, 0, quizpani, quizpani, quizpani_state, 0, ROT0, "NMK", "Quiz Panicuru Fantasy", MACHINE_SUPPORTS_SAVE )
