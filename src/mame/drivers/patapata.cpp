// license:BSD-3-Clause
// copyright-holders:David Haywood

/*

RW93085 (C) NTC CO.      on main PCB

RW93085-SUB (c)NTC CO.LTD 1993 MADE IN JAPAN     on sub (ROM) PCB

various NMK customs including NMK005 near dipswitches, difficult to read the rest from PCB pics
possible MCU or just IO extender?

developed by NTC on NMK hardware?

video is probably just 2 tilemap layers
maybe close to jalmah.cpp?

*/

#include "emu.h"
#include "machine/nmk112.h"

#include "cpu/m68000/m68000.h"
#include "sound/okim6295.h"

#include "screen.h"
#include "speaker.h"


class patapata_state : public driver_device
{
public:
	patapata_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_bg_videoram(*this, "bg_videoram"),
		m_fg_videoram(*this, "fg_videoram"),
		m_vregs(*this, "videoregs"),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode")
	{ }

	DECLARE_WRITE16_MEMBER(bg_videoram_w);
	DECLARE_WRITE16_MEMBER(fg_videoram_w);
	DECLARE_WRITE8_MEMBER(flipscreen_w);

	TIMER_DEVICE_CALLBACK_MEMBER(scanline);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	virtual void video_start() override;

private:
	/* memory pointers */
	required_shared_ptr<uint16_t> m_bg_videoram;
	required_shared_ptr<uint16_t> m_fg_videoram;
	required_shared_ptr<uint16_t> m_vregs;

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;

	/* video-related */
	TILEMAP_MAPPER_MEMBER(pagescan);
	tilemap_t    *m_bg_tilemap;
	tilemap_t    *m_fg_tilemap;
	int m_fg_bank;

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
};

WRITE16_MEMBER(patapata_state::bg_videoram_w)
{
	COMBINE_DATA(&m_bg_videoram[offset]);
	m_bg_tilemap->mark_tile_dirty(offset);
}

TILE_GET_INFO_MEMBER(patapata_state::get_bg_tile_info)
{
	int tileno = m_bg_videoram[tile_index];
	int pal = tileno>>12;
	SET_TILE_INFO_MEMBER(0, tileno&0x1fff, pal, 0);
}

WRITE16_MEMBER(patapata_state::fg_videoram_w)
{
	COMBINE_DATA(&m_fg_videoram[offset]);
	m_fg_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(patapata_state::flipscreen_w)
{
	flip_screen_set(data & 0x01);
}

TILE_GET_INFO_MEMBER(patapata_state::get_fg_tile_info)
{
	int bank = m_fg_bank;

	int tileno = m_fg_videoram[tile_index];
	int pal = tileno>>12;

	SET_TILE_INFO_MEMBER(1, (tileno&0x0fff)+(bank*0x1000), pal, 0);
}

TILEMAP_MAPPER_MEMBER(patapata_state::pagescan)
{
	return (col &0xff) * (num_rows>>1) + (row & 0xf) + ((row & 0x10)<<8) + ((col & 0x300) << 5);
//  return (col &0xff) * (num_rows>>1) + (row & 0xf) + ((row & 0x10)<<8) + ((col & 0x100) << 5); // see comment with tilemap creation
}


void patapata_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(patapata_state::get_bg_tile_info),this), tilemap_mapper_delegate(FUNC(patapata_state::pagescan),this), 16, 16, 1024,16*2);
	m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(patapata_state::get_fg_tile_info),this), tilemap_mapper_delegate(FUNC(patapata_state::pagescan),this), 16, 16, 1024,16*2);

// 2nd half of the ram seems unused, maybe it's actually a mirror meaning this would be the correct tilemap sizes
// m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(patapata_state::get_bg_tile_info),this), tilemap_mapper_delegate(FUNC(patapata_state::pagescan),this), 16, 16, 1024/2,16*2);
// m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(patapata_state::get_fg_tile_info),this), tilemap_mapper_delegate(FUNC(patapata_state::pagescan),this), 16, 16, 1024/2,16*2);

	m_fg_tilemap->set_transparent_pen(0xf);

	save_item(NAME(m_fg_bank));
}

uint32_t patapata_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	// vregs
	// 0/1 are fg scroll?  0x0ff0 , 0x07b0  is no scroll
	// 2/3 are bg scroll?
	// 4 is fg bank

	int scrollx = (m_vregs[2]-0xff0)&0xfff;
	int scrolly = (m_vregs[3]-0x7b0)&0xfff;
	if (scrolly&0x200) scrollx += 0x1000;
	scrolly&=0x1ff;

	m_bg_tilemap->set_scrollx(0, scrollx);
	m_bg_tilemap->set_scrolly(0, scrolly);

	scrollx = (m_vregs[0]-0xff0)&0xfff;
	scrolly = (m_vregs[1]-0x7b0)&0xfff;
	if (scrolly&0x200) scrollx += 0x1000;
	scrolly&=0x1ff;

	m_fg_tilemap->set_scrollx(0, scrollx);
	m_fg_tilemap->set_scrolly(0, scrolly);


	if ((m_vregs[4]&0x3) != m_fg_bank)
	{
		m_fg_bank = m_vregs[4]&0x3;
		m_fg_tilemap->mark_all_dirty();
	}


	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 0, 0);

	/*
	popmessage("%04x %04x %04x %04x\n%04x %04x %04x %04x",
	    m_vregs[0], m_vregs[1],
	    m_vregs[2], m_vregs[3],
	    m_vregs[4], m_vregs[5],
	    m_vregs[6], m_vregs[7]);
	*/
	return 0;
}

/*

also

[:maincpu] ':maincpu' (00A284): unmapped program memory write to 110400 = 0000 & FFFF
[:maincpu] ':maincpu' (00A284): unmapped program memory write to 110402 = 0000 & FFFF


*/

static ADDRESS_MAP_START( main_map, AS_PROGRAM, 16, patapata_state )
	AM_RANGE(0x000000, 0x07ffff) AM_ROM

	AM_RANGE(0x100000, 0x100001) AM_READ_PORT("IN0")
	AM_RANGE(0x100002, 0x100003) AM_READ_PORT("IN1")
	AM_RANGE(0x100008, 0x100009) AM_READ_PORT("DSW1")
	AM_RANGE(0x10000a, 0x10000b) AM_READ_PORT("DSW2")
	AM_RANGE(0x100014, 0x100015) AM_WRITE8(flipscreen_w, 0x00ff)
	AM_RANGE(0x110000, 0x1103ff) AM_RAM AM_SHARE("videoregs")
	AM_RANGE(0x120000, 0x1205ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x130000, 0x13ffff) AM_RAM_WRITE(fg_videoram_w) AM_SHARE("fg_videoram")
	AM_RANGE(0x140000, 0x14ffff) AM_RAM_WRITE(bg_videoram_w) AM_SHARE("bg_videoram")
	AM_RANGE(0x150000, 0x150001) AM_DEVREADWRITE8("oki1", okim6295_device, read, write, 0x00ff)
	AM_RANGE(0x150010, 0x150011) AM_DEVREADWRITE8("oki2", okim6295_device, read, write, 0x00ff)
	AM_RANGE(0x150020, 0x15002f) AM_DEVWRITE8("nmk112", nmk112_device, okibank_w, 0x00ff)
	AM_RANGE(0x180000, 0x18ffff) AM_RAM // mainram?
ADDRESS_MAP_END

static INPUT_PORTS_START( patapata )
	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Reset")
	PORT_SERVICE( 0x20, IP_ACTIVE_LOW )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("IN1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICKLEFT_DOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICKLEFT_UP )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICKRIGHT_DOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICKRIGHT_UP )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Hopper")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN1 )

	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Flip_Screen ) )  PORT_DIPLOCATION("DSW1:8")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPUNKNOWN_DIPLOC( 0x02, 0x02, "DSW1:7" )
	PORT_DIPUNKNOWN_DIPLOC( 0x04, 0x04, "DSW1:6" )
	PORT_DIPUNKNOWN_DIPLOC( 0x08, 0x08, "DSW1:5" )
	PORT_DIPUNKNOWN_DIPLOC( 0x10, 0x10, "DSW1:4" )
	PORT_DIPUNKNOWN_DIPLOC( 0x20, 0x20, "DSW1:3" )
	PORT_DIPUNKNOWN_DIPLOC( 0x40, 0x40, "DSW1:2" )
	PORT_DIPUNKNOWN_DIPLOC( 0x80, 0x80, "DSW1:1" )

	PORT_START("DSW2")
	PORT_DIPUNKNOWN_DIPLOC( 0x01, 0x01, "DSW2:8" )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION("DSW2:7")
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPNAME( 0x1c, 0x1c, DEF_STR( Coin_B ) ) PORT_DIPLOCATION("DSW2:6,5,4")
	PORT_DIPSETTING(    0x1c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x14, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_7C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_8C ) )
	PORT_DIPNAME( 0xe0, 0xe0, DEF_STR( Coin_A ) ) PORT_DIPLOCATION("DSW2:3,2,1")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0xe0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x60, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0xa0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 1C_4C ) )
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

static GFXDECODE_START( patapata )
	GFXDECODE_ENTRY( "tilesa", 0, tilelayout, 0x000, 16 )
	GFXDECODE_ENTRY( "tilesb", 0, tilelayout, 0x100, 16 )
GFXDECODE_END

TIMER_DEVICE_CALLBACK_MEMBER(patapata_state::scanline)
{
	// reads inputs (half-frame interrupt like NMK16?)
	if (param==128) m_maincpu->set_input_line(1, HOLD_LINE);
}

static MACHINE_CONFIG_START( patapata )

	MCFG_CPU_ADD("maincpu", M68000, XTAL_16MHz) // 16 MHz XTAL, 16 MHz CPU
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", patapata_state,  irq4_line_hold) // 1 + 4 valid? (4 main VBL)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", patapata_state, scanline, "screen", 0, 1)

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", patapata)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(64*8, 64*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 60*8-1, 0*8, 44*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(patapata_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 0x600/2)
	MCFG_PALETTE_FORMAT(RRRRGGGGBBBBRGBx)

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_OKIM6295_ADD("oki1", XTAL_16MHz / 4, PIN7_LOW) // not verified
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.40)

	MCFG_OKIM6295_ADD("oki2", XTAL_16MHz / 4, PIN7_LOW) // not verified
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.40)

	MCFG_DEVICE_ADD("nmk112", NMK112, 0) // or 212? difficult to read (maybe 212 is 2* 112?)
	MCFG_NMK112_ROM0("oki1")
	MCFG_NMK112_ROM1("oki2")
MACHINE_CONFIG_END

ROM_START( patapata )
	ROM_REGION( 0x80000, "maincpu", 0 )  /* 68000 code */
	ROM_LOAD16_WORD_SWAP( "rw-93085-7.u132",  0x00000, 0x80000, CRC(f8084e30) SHA1(8ca19fb3d348affbcb89fb4fef0be4614edd14f7) )

	ROM_REGION( 0x080000, "tilesa", 0 ) // bg layer - 1 bank
	ROM_LOAD16_BYTE( "rw-93085-9.u5",    0x000001, 0x040000, CRC(c2e243ff) SHA1(492e25ac1f85ac6f815409ce11de9a1fabab6fc1) )
	ROM_LOAD16_BYTE( "rw-93085-10.u15",  0x000000, 0x040000, CRC(546be459) SHA1(f96b139a1b7c021cd9752d626330ffd6201d7441) )

	ROM_REGION( 0x180000, "tilesb", 0 ) // fg layer - 3 banks
	ROM_LOAD16_BYTE( "rw-93085-17.u9",  0x000001, 0x080000, CRC(e19afa04) SHA1(0511ac94faa549706d729678b4f26b738cf19059) )
	ROM_LOAD16_BYTE( "rw-93085-18.u19", 0x000000, 0x080000, CRC(5cf4582e) SHA1(98a5a274589aa048fa5809d5bb38326e287e6905) )
	ROM_LOAD16_BYTE( "rw-93085-19.u19", 0x100001, 0x040000, CRC(dfd7bdcf) SHA1(02e46da9a8c938daa180a57f4aca04b2fd655ee0) )
	ROM_LOAD16_BYTE( "rw-93085-20.u20", 0x100000, 0x040000, CRC(dd821f74) SHA1(a63e9979db30d130449f689cc6ba8b4c7d25085a) )

	ROM_REGION( 0x100000+0x40000, "oki1", 0 ) /* OKIM6295 samples */
	ROM_LOAD( "rw-93085-5.u22", 0x000000+0x40000, 0x080000, CRC(0c0d2835) SHA1(dc14ebea5f4e0d3f2f8e7bc05e16b8d0f92ce588) )
	ROM_LOAD( "rw-93085-6.u23", 0x080000+0x40000, 0x080000, CRC(882c25d0) SHA1(9cbf21bd5940240440025b4481d96e3db45a676c) )

	ROM_REGION( 0x100000+0x40000, "oki2", 0 ) /* OKIM6295 samples */
	ROM_LOAD( "rw-93085-1.u3",  0x000000+0x40000, 0x080000, CRC(d9776d50) SHA1(06e4d2184f687af8380fcb49ce48ce8ec8091050) )
	ROM_LOAD( "rw-93085-2.u4",  0x080000+0x40000, 0x080000, CRC(3698fafa) SHA1(3de54a990478621271285254544f5382d6fd9ca9) )

	ROM_REGION( 0x0200, "proms", 0 )
	ROM_LOAD( "n82s131n.u119", 0x000, 0x200, CRC(33f63fc8) SHA1(24c4a1a7c06e546571c77c7dc7bd87c57aa088d7) )
	ROM_LOAD( "n82s135n.u137", 0x000, 0x100, CRC(cb90eedc) SHA1(6577cb1999a90b9209b150cbedde11de9ac30018) )
ROM_END

// cabinet shows Atlus logo, though there's no copyright on the title screen and PCB is NTC / NMK
GAME( 1993, patapata, 0, patapata, patapata, patapata_state, 0, ROT0,  "Atlus", "Pata Pata Panic", MACHINE_SUPPORTS_SAVE )
