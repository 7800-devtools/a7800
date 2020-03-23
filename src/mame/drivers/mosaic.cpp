// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
/***************************************************************************

    Mosaic (c) 1990 Space

    Notes:
    - the ROM OK / RAM OK message in service mode is fake: ROM and RAM are not tested.

***************************************************************************/

#include "emu.h"
#include "includes/mosaic.h"

#include "cpu/z180/z180.h"
#include "sound/2203intf.h"
#include "screen.h"
#include "speaker.h"


WRITE8_MEMBER(mosaic_state::protection_w)
{
	if (!BIT(data, 7))
	{
		/* simply increment given value */
		m_prot_val = (data + 1) << 8;
	}
	else
	{
		static const int jumptable[] =
		{
			0x02be, 0x0314, 0x0475, 0x0662, 0x0694, 0x08f3, 0x0959, 0x096f,
			0x0992, 0x09a4, 0x0a50, 0x0d69, 0x0eee, 0x0f98, 0x1040, 0x1075,
			0x10d8, 0x18b4, 0x1a27, 0x1a4a, 0x1ac6, 0x1ad1, 0x1ae2, 0x1b68,
			0x1c95, 0x1fd5, 0x20fc, 0x212d, 0x213a, 0x21b6, 0x2268, 0x22f3,
			0x231a, 0x24bb, 0x286b, 0x295f, 0x2a7f, 0x2fc6, 0x3064, 0x309f,
			0x3118, 0x31e1, 0x32d0, 0x35f7, 0x3687, 0x38ea, 0x3b86, 0x3c9a,
			0x411f, 0x473f
		};

		m_prot_val = jumptable[data & 0x7f];
	}
}

READ8_MEMBER(mosaic_state::protection_r)
{
	int res = (m_prot_val >> 8) & 0xff;

	logerror("%06x: protection_r %02x\n", space.device().safe_pc(), res);

	m_prot_val <<= 8;

	return res;
}

WRITE8_MEMBER(mosaic_state::gfire2_protection_w)
{
	logerror("%06x: protection_w %02x\n", space.device().safe_pc(), data);

	switch(data)
	{
		case 0x01:
			/* written repeatedly; no effect?? */
			break;
		case 0x02:
			m_prot_val = 0x0a10;
			break;
		case 0x04:
			m_prot_val = 0x0a15;
			break;
		case 0x06:
			m_prot_val = 0x80e3;
			break;
		case 0x08:
			m_prot_val = 0x0965;
			break;
		case 0x0a:
			m_prot_val = 0x04b4;
			break;
	}
}

READ8_MEMBER(mosaic_state::gfire2_protection_r)
{
	int res = m_prot_val & 0xff;

	m_prot_val >>= 8;

	return res;
}



static ADDRESS_MAP_START( mosaic_map, AS_PROGRAM, 8, mosaic_state )
	AM_RANGE(0x00000, 0x0ffff) AM_ROM
	AM_RANGE(0x20000, 0x21fff) AM_RAM
	AM_RANGE(0x22000, 0x22fff) AM_RAM_WRITE(bgvideoram_w) AM_SHARE("bgvideoram")
	AM_RANGE(0x23000, 0x23fff) AM_RAM_WRITE(fgvideoram_w) AM_SHARE("fgvideoram")
	AM_RANGE(0x24000, 0x241ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
ADDRESS_MAP_END

static ADDRESS_MAP_START( gfire2_map, AS_PROGRAM, 8, mosaic_state )
	AM_RANGE(0x00000, 0x0ffff) AM_ROM
	AM_RANGE(0x10000, 0x17fff) AM_RAM
	AM_RANGE(0x22000, 0x22fff) AM_RAM_WRITE(bgvideoram_w) AM_SHARE("bgvideoram")
	AM_RANGE(0x23000, 0x23fff) AM_RAM_WRITE(fgvideoram_w) AM_SHARE("fgvideoram")
	AM_RANGE(0x24000, 0x241ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
ADDRESS_MAP_END

static ADDRESS_MAP_START( mosaic_io_map, AS_IO, 8, mosaic_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x3f) AM_WRITENOP    /* Z180 internal registers */
	AM_RANGE(0x30, 0x30) AM_READNOP /* Z180 internal registers */
	AM_RANGE(0x70, 0x71) AM_DEVREADWRITE("ymsnd", ym2203_device, read, write)
	AM_RANGE(0x72, 0x72) AM_READWRITE(protection_r, protection_w)
	AM_RANGE(0x74, 0x74) AM_READ_PORT("P1")
	AM_RANGE(0x76, 0x76) AM_READ_PORT("P2")
ADDRESS_MAP_END

static ADDRESS_MAP_START( gfire2_io_map, AS_IO, 8, mosaic_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x3f) AM_WRITENOP    /* Z180 internal registers */
	AM_RANGE(0x30, 0x30) AM_READNOP /* Z180 internal registers */
	AM_RANGE(0x70, 0x71) AM_DEVREADWRITE("ymsnd", ym2203_device, read, write)
	AM_RANGE(0x72, 0x72) AM_READWRITE(gfire2_protection_r, gfire2_protection_w)
	AM_RANGE(0x74, 0x74) AM_READ_PORT("P1")
	AM_RANGE(0x76, 0x76) AM_READ_PORT("P2")
ADDRESS_MAP_END


static INPUT_PORTS_START( mosaic )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN1 )

	PORT_START("P2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN2 )

	PORT_START("DSW")
	PORT_SERVICE( 0x80, IP_ACTIVE_LOW )
	PORT_DIPNAME( 0x40, 0x00, "Bombs" )
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPSETTING(    0x40, "5" )
	PORT_DIPNAME( 0x20, 0x20, "Speed" )
	PORT_DIPSETTING(    0x20, DEF_STR( Low ) )
	PORT_DIPSETTING(    0x00, DEF_STR( High ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Demo_Sounds ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Coinage ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_2C ) )
	PORT_DIPNAME( 0x02, 0x00, "Music" )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x01, 0x00, "Sound" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( gfire2 )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN1 )

	PORT_START("P2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW")
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Language ) )
	PORT_DIPSETTING(    0x00, DEF_STR( English ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Korean ) )
	PORT_DIPNAME( 0x60, 0x60, DEF_STR( Coinage ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x60, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 1C_2C ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unused ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Difficulty ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Hard ) )
//  PORT_DIPSETTING(    0x00, DEF_STR( Hard ) )
	PORT_DIPNAME( 0x02, 0x02, "Bonus Time" )
	PORT_DIPSETTING(    0x00, "*2 +30" )
	PORT_DIPSETTING(    0x02, "*2 +50" )
	PORT_SERVICE( 0x01, IP_ACTIVE_LOW )
INPUT_PORTS_END



static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,4),
	8,
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{   RGN_FRAC(3,4)+0, RGN_FRAC(2,4)+0, RGN_FRAC(1,4)+0, RGN_FRAC(0,4)+0,
		RGN_FRAC(3,4)+8, RGN_FRAC(2,4)+8, RGN_FRAC(1,4)+8, RGN_FRAC(0,4)+8 },
	{ 0*16, 1*16, 2*16, 3*16, 4*16, 5*16, 6*16, 7*16 },
	16*8
};

static GFXDECODE_START( mosaic )
	GFXDECODE_ENTRY( "gfx1", 0, charlayout, 0, 1 )
	GFXDECODE_ENTRY( "gfx2", 0, charlayout, 0, 1 )
GFXDECODE_END

void mosaic_state::machine_start()
{
	save_item(NAME(m_prot_val));
}

void mosaic_state::machine_reset()
{
	m_prot_val = 0;
}

static MACHINE_CONFIG_START( mosaic )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z180, 7000000)  /* ??? */
	MCFG_CPU_PROGRAM_MAP(mosaic_map)
	MCFG_CPU_IO_MAP(mosaic_io_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", mosaic_state,  irq0_line_hold)


	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(64*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(8*8, 48*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(mosaic_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", mosaic)
	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT(xRRRRRGGGGGBBBBB)


	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ymsnd", YM2203, 3000000)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( gfire2, mosaic )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(gfire2_map)
	MCFG_CPU_IO_MAP(gfire2_io_map)
MACHINE_CONFIG_END



/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( mosaic )
	ROM_REGION( 0x100000, "maincpu", 0 )    /* 1024k for Z180 address space */
	ROM_LOAD( "mosaic.9", 0x00000, 0x10000, CRC(5794dd39) SHA1(28784371f4ca561e3c0fb74d1f0a204f58ccdd3a) )

	ROM_REGION( 0x40000, "gfx1", 0 )
	ROM_LOAD( "mosaic.1", 0x00000, 0x10000, CRC(05f4cc70) SHA1(367cfa716b5d24663efcd98a4a80bf02ef28f2f8) )
	ROM_LOAD( "mosaic.2", 0x10000, 0x10000, CRC(78907875) SHA1(073b90e0303f7812e7e8f66bb798a7734cb36bb9) )
	ROM_LOAD( "mosaic.3", 0x20000, 0x10000, CRC(f81294cd) SHA1(9bce627bbe3940769776121fb4296f92ac4c7d1a) )
	ROM_LOAD( "mosaic.4", 0x30000, 0x10000, CRC(fff72536) SHA1(4fc5d0a79128dd49275bc4c4cc2dd7c587096fd8) )

	ROM_REGION( 0x40000, "gfx2", 0 )
	ROM_LOAD( "mosaic.5", 0x00000, 0x10000, CRC(28513fbf) SHA1(e69051206cc3df470e7b2358c51cbbed294795f5) )
	ROM_LOAD( "mosaic.6", 0x10000, 0x10000, CRC(1b8854c4) SHA1(d49df2565d9ccda403fafb9e219d3603776e3d34) )
	ROM_LOAD( "mosaic.7", 0x20000, 0x10000, CRC(35674ac2) SHA1(6422a81034b6d34aefc8ca5d2926d3d3c3d7ff77) )
	ROM_LOAD( "mosaic.8", 0x30000, 0x10000, CRC(6299c376) SHA1(eb64b20268c06c97c4201c8004a759b6de42fab6) )
ROM_END

ROM_START( mosaica )
	ROM_REGION( 0x100000, "maincpu", 0 )    /* 1024k for Z180 address space */
	ROM_LOAD( "mosaic_9.a02", 0x00000, 0x10000, CRC(ecb4f8aa) SHA1(e45c074bac92d1d079cf1bcc0a6a081beb3dbb8e) )

	ROM_REGION( 0x40000, "gfx1", 0 )
	ROM_LOAD( "mosaic.1", 0x00000, 0x10000, CRC(05f4cc70) SHA1(367cfa716b5d24663efcd98a4a80bf02ef28f2f8) )
	ROM_LOAD( "mosaic.2", 0x10000, 0x10000, CRC(78907875) SHA1(073b90e0303f7812e7e8f66bb798a7734cb36bb9) )
	ROM_LOAD( "mosaic.3", 0x20000, 0x10000, CRC(f81294cd) SHA1(9bce627bbe3940769776121fb4296f92ac4c7d1a) )
	ROM_LOAD( "mosaic.4", 0x30000, 0x10000, CRC(fff72536) SHA1(4fc5d0a79128dd49275bc4c4cc2dd7c587096fd8) )

	ROM_REGION( 0x40000, "gfx2", 0 )
	ROM_LOAD( "mosaic.5", 0x00000, 0x10000, CRC(28513fbf) SHA1(e69051206cc3df470e7b2358c51cbbed294795f5) )
	ROM_LOAD( "mosaic.6", 0x10000, 0x10000, CRC(1b8854c4) SHA1(d49df2565d9ccda403fafb9e219d3603776e3d34) )
	ROM_LOAD( "mosaic.7", 0x20000, 0x10000, CRC(35674ac2) SHA1(6422a81034b6d34aefc8ca5d2926d3d3c3d7ff77) )
	ROM_LOAD( "mosaic.8", 0x30000, 0x10000, CRC(6299c376) SHA1(eb64b20268c06c97c4201c8004a759b6de42fab6) )
ROM_END

ROM_START( gfire2 )
	ROM_REGION( 0x100000, "maincpu", 0 )    /* 1024k for Z180 address space */
	ROM_LOAD( "goldf2_i.7e",         0x00000, 0x10000, CRC(a102f7d0) SHA1(cfde51d0e9e69e9653fdfd70d4e4f4649b662005) )

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "goldf2_a.1k",         0x00000, 0x40000, CRC(1f086472) SHA1(c776a734869b6bab317627bd15457a9fb18e1159) )
	ROM_LOAD( "goldf2_b.1j",         0x40000, 0x40000, CRC(edb0d40c) SHA1(624a71b42a2e6c7c55cf455395aa0ad9b3eaeb9e) )
	ROM_LOAD( "goldf2_c.1i",         0x80000, 0x40000, CRC(d0ebd486) SHA1(ff2bfc84bc622b437913e1861f7acb373c7844c8) )
	ROM_LOAD( "goldf2_d.1h",         0xc0000, 0x40000, CRC(2b56ae2c) SHA1(667f9093ed28ba1804583fb201c7e3b37f1a9927) )

	ROM_REGION( 0x80000, "gfx2", 0 )
	ROM_LOAD( "goldf2_e.1e",         0x00000, 0x20000, CRC(61b8accd) SHA1(d6317b8b7ab33a2a78d388b87ddb8946e6c6df29) )
	ROM_LOAD( "goldf2_f.1d",         0x20000, 0x20000, CRC(49f77e53) SHA1(6e7c8f86cb368bf1a32f02f72e7b418684c847dc) )
	ROM_LOAD( "goldf2_g.1b",         0x40000, 0x20000, CRC(aa79f3bf) SHA1(c0b62f5de7e36ce1ef1de92ee6f63d8286815566) )
	ROM_LOAD( "goldf2_h.1a",         0x60000, 0x20000, CRC(a3519259) SHA1(9e1edb50ade4a4ddcd628a897f6fa712075a888b) )
ROM_END



GAME( 1990, mosaic,  0,      mosaic, mosaic, mosaic_state, 0, ROT0, "Space",                 "Mosaic",         MACHINE_SUPPORTS_SAVE )
GAME( 1990, mosaica, mosaic, mosaic, mosaic, mosaic_state, 0, ROT0, "Space (Fuuki license)", "Mosaic (Fuuki)", MACHINE_SUPPORTS_SAVE )
GAME( 1992, gfire2,  0,      gfire2, gfire2, mosaic_state, 0, ROT0, "Topis Corp",            "Golden Fire II", MACHINE_SUPPORTS_SAVE )
