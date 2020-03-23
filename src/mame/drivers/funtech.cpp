// license:BSD-3-Clause
// copyright-holders:David Haywood
/*

program rom contains the following details

COMPANY:FUN TECH CORPORATION
PRODUCT-NAME:SUPER TWO IN ONE
PROJECTOR:TIEN YUAN CHIEN,NOMA
HARDWARE-DESIGNER:EN YU CHENG
SOFTWARE-DESIGNER:RANG CHANG LI,CHIH HNI HUANG,WEN CHANG LIN
PROGRAM-VERSION:1.0
PROGRAM-DATE:09/23/1993

8x8 tiles and 8x32 reels, likely going to be very similar to skylncr.cpp or goldstar.cpp (which are both very similar anyway)
palette addresses are the same as unkch in goldtar.cpp, but the io stuff is definitely different here

board has an M5255 for sound
and an unpopulated position for a YM2413 or UM3567

*/

#include "emu.h"

#include "cpu/z80/z80.h"
#include "sound/ay8910.h"
#include "machine/nvram.h"
#include "machine/ticket.h"
#include "screen.h"
#include "speaker.h"

#include "fts2in1.lh"



class fun_tech_corp_state : public driver_device
{
public:
	fun_tech_corp_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_fgram(*this, "fgram"),
		m_reel1_ram(*this, "reel1ram"),
		m_reel2_ram(*this, "reel2ram"),
		m_reel3_ram(*this, "reel3ram"),
		m_reel1_scroll(*this, "reel1_scroll"),
		m_reel2_scroll(*this, "reel2_scroll"),
		m_reel3_scroll(*this, "reel3_scroll"),
		m_reel1_alt_scroll(*this, "reel1_alt_scroll"),
		m_maincpu(*this, "maincpu"),
		m_hopper(*this, "hopper"),
		m_gfxdecode(*this, "gfxdecode") { }

	required_shared_ptr<uint8_t> m_fgram;
	required_shared_ptr<uint8_t> m_reel1_ram;
	required_shared_ptr<uint8_t> m_reel2_ram;
	required_shared_ptr<uint8_t> m_reel3_ram;
	required_shared_ptr<uint8_t> m_reel1_scroll;
	required_shared_ptr<uint8_t> m_reel2_scroll;
	required_shared_ptr<uint8_t> m_reel3_scroll;
	required_shared_ptr<uint8_t> m_reel1_alt_scroll;


	INTERRUPT_GEN_MEMBER(funtech_vblank_interrupt);

	DECLARE_WRITE8_MEMBER(funtech_lamps_w);
	DECLARE_WRITE8_MEMBER(funtech_coins_w);
	DECLARE_WRITE8_MEMBER(funtech_vreg_w);


	uint8_t m_vreg;

	tilemap_t *m_fg_tilemap;

	DECLARE_WRITE8_MEMBER(fgram_w);

	TILE_GET_INFO_MEMBER(get_fg_tile_info);

	tilemap_t *m_reel1_tilemap;
	tilemap_t *m_reel2_tilemap;
	tilemap_t *m_reel3_tilemap;

	DECLARE_WRITE8_MEMBER(reel1_ram_w);
	DECLARE_WRITE8_MEMBER(reel2_ram_w);
	DECLARE_WRITE8_MEMBER(reel3_ram_w);

	TILE_GET_INFO_MEMBER(get_reel1_tile_info);
	TILE_GET_INFO_MEMBER(get_reel2_tile_info);
	TILE_GET_INFO_MEMBER(get_reel3_tile_info);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_funtech(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	required_device<cpu_device> m_maincpu;
	required_device<ticket_dispenser_device> m_hopper;
	required_device<gfxdecode_device> m_gfxdecode;
};


TILE_GET_INFO_MEMBER(fun_tech_corp_state::get_fg_tile_info)
{
	int code = m_fgram[tile_index];
	int attr = m_fgram[tile_index+0x800];

	code |= (attr & 0x0f) << 8;

	if (m_vreg&1) code |= 0x1000;

	SET_TILE_INFO_MEMBER(0,
			code,
			attr>>4,
			0);
}


TILE_GET_INFO_MEMBER(fun_tech_corp_state::get_reel1_tile_info)
{
	int code = m_reel1_ram[tile_index];
	if (m_vreg & 0x4) code |= 0x100;
	if (m_vreg & 0x8) code |= 0x200;

	SET_TILE_INFO_MEMBER(1,
			code,
			0,
			0);
}

TILE_GET_INFO_MEMBER(fun_tech_corp_state::get_reel2_tile_info)
{
	int code = m_reel2_ram[tile_index];
	if (m_vreg & 0x4) code |= 0x100;
	if (m_vreg & 0x8) code |= 0x200;

	SET_TILE_INFO_MEMBER(1,
			code,
			0,
			0);
}


TILE_GET_INFO_MEMBER(fun_tech_corp_state::get_reel3_tile_info)
{
	int code = m_reel3_ram[tile_index];
	if (m_vreg & 0x4) code |= 0x100;
	if (m_vreg & 0x8) code |= 0x200;

	SET_TILE_INFO_MEMBER(1,
			code,
			0,
			0);
}


WRITE8_MEMBER(fun_tech_corp_state::reel1_ram_w)
{
	m_reel1_ram[offset] = data;
	m_reel1_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(fun_tech_corp_state::reel2_ram_w)
{
	m_reel2_ram[offset] = data;
	m_reel2_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(fun_tech_corp_state::reel3_ram_w)
{
	m_reel3_ram[offset] = data;
	m_reel3_tilemap->mark_tile_dirty(offset);
}


void fun_tech_corp_state::video_start()
{
	m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(fun_tech_corp_state::get_fg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 64, 32);
	m_fg_tilemap->set_transparent_pen(0);

	m_reel1_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(fun_tech_corp_state::get_reel1_tile_info),this),TILEMAP_SCAN_ROWS,8,32, 64, 8);
	m_reel2_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(fun_tech_corp_state::get_reel2_tile_info),this),TILEMAP_SCAN_ROWS,8,32, 64, 8);
	m_reel3_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(fun_tech_corp_state::get_reel3_tile_info),this),TILEMAP_SCAN_ROWS,8,32, 64, 8);

	m_reel1_tilemap->set_scroll_cols(64);
	m_reel2_tilemap->set_scroll_cols(64);
	m_reel3_tilemap->set_scroll_cols(64);

}

WRITE8_MEMBER(fun_tech_corp_state::fgram_w)
{
	m_fgram[offset] = data;
	m_fg_tilemap->mark_tile_dirty(offset&0x7ff);
}


uint32_t fun_tech_corp_state::screen_update_funtech(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);

	if (!(m_vreg & 0x40))
	{
		for (int i = 0; i < 64; i++)
		{
			m_reel1_tilemap->set_scrolly(i, m_reel1_scroll[i]);
			m_reel2_tilemap->set_scrolly(i, m_reel2_scroll[i]);
			m_reel3_tilemap->set_scrolly(i, m_reel3_scroll[i]);
		}

		const rectangle visible1(0 * 8, (14 + 48) * 8 - 1, 4 * 8, (4 + 7) * 8 - 1);
		const rectangle visible2(0 * 8, (14 + 48) * 8 - 1, 12 * 8, (12 + 7) * 8 - 1);
		const rectangle visible3(0 * 8, (14 + 48) * 8 - 1, 18 * 8, (18 + 7) * 8 - 1);

		m_reel1_tilemap->draw(screen, bitmap, visible1, 0, 0);
		m_reel2_tilemap->draw(screen, bitmap, visible2, 0, 0);
		m_reel3_tilemap->draw(screen, bitmap, visible3, 0, 0);
	}
	else
	{
		// this mode seems to draw reel1 as fullscreen using a different set of scroll regs
		for (int i = 0; i < 64; i++)
		{
			m_reel1_tilemap->set_scrolly(i, m_reel1_alt_scroll[i]);
		}

		m_reel1_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	}


	m_fg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}



INTERRUPT_GEN_MEMBER(fun_tech_corp_state::funtech_vblank_interrupt)
{
//  if (m_nmi_enable)
		device.execute().set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}



static ADDRESS_MAP_START( funtech_map, AS_PROGRAM, 8, fun_tech_corp_state )
	AM_RANGE(0x0000, 0xbfff) AM_ROM

	AM_RANGE(0xc000, 0xc1ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0xc800, 0xc9ff) AM_RAM_DEVWRITE("palette", palette_device, write_ext) AM_SHARE("palette_ext")

	AM_RANGE(0xd000, 0xd7ff) AM_ROM // maybe

	AM_RANGE(0xd800, 0xdfff) AM_RAM AM_SHARE("nvram")

	AM_RANGE(0xe000, 0xefff) AM_RAM_WRITE(fgram_w) AM_SHARE("fgram")
	AM_RANGE(0xf000, 0xf1ff) AM_RAM_WRITE(reel1_ram_w) AM_SHARE("reel1ram")
	AM_RANGE(0xf200, 0xf3ff) AM_RAM_WRITE(reel2_ram_w) AM_SHARE("reel2ram")
	AM_RANGE(0xf400, 0xf5ff) AM_RAM_WRITE(reel3_ram_w) AM_SHARE("reel3ram")
	AM_RANGE(0xf600, 0xf7ff) AM_RAM

	AM_RANGE(0xf840, 0xf87f) AM_RAM AM_SHARE("reel1_scroll")
	AM_RANGE(0xf880, 0xf8bf) AM_RAM AM_SHARE("reel2_scroll")
	AM_RANGE(0xf900, 0xf93f) AM_RAM AM_SHARE("reel3_scroll")

	AM_RANGE(0xf9c0, 0xf9ff) AM_RAM AM_SHARE("reel1_alt_scroll") // or a mirror, gets used in 'full screen' mode.
ADDRESS_MAP_END


WRITE8_MEMBER(fun_tech_corp_state::funtech_lamps_w)
{
	output().set_lamp_value(0, (data >> 0) & 1);
	output().set_lamp_value(1, (data >> 1) & 1);
	output().set_lamp_value(2, (data >> 2) & 1);
	output().set_lamp_value(3, (data >> 3) & 1);
	output().set_lamp_value(4, (data >> 4) & 1);
	output().set_lamp_value(5, (data >> 5) & 1);
	output().set_lamp_value(6, (data >> 6) & 1);
	output().set_lamp_value(7, (data >> 7) & 1);
}

WRITE8_MEMBER(fun_tech_corp_state::funtech_coins_w)
{
	if (data & 0x01) printf("funtech_coins_w %02x\n", data);

	// 80 = hopper motor?
	m_hopper->write(space, 0, data & 0x80);

	// 40 = ? sometimes

	// 20 = coin 3 counter
	machine().bookkeeping().coin_counter_w(2, data & 0x20 );

	// 10 = coin 2 counter
	machine().bookkeeping().coin_counter_w(1, data & 0x10 );

	// 08 = coin 4 counter
	machine().bookkeeping().coin_counter_w(3, data & 0x08 );

	// 04 = coin 1 counter
	machine().bookkeeping().coin_counter_w(0, data & 0x04 );

	// 02 = used when hopper is used (coin out counter?)
}

WRITE8_MEMBER(fun_tech_corp_state::funtech_vreg_w)
{
	if (data & 0xb2) printf("funtech_vreg_w %02x\n", data);

	// -x-- rr-t
	// t = text tile bank
	// r = reel tile bank
	// x = show reel 1 full screen?

	m_vreg = data;
	m_fg_tilemap->mark_all_dirty();
	m_reel1_tilemap->mark_all_dirty();
}



static ADDRESS_MAP_START( funtech_io_map, AS_IO, 8, fun_tech_corp_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	// lamps?
	AM_RANGE(0x00, 0x00) AM_WRITE(funtech_lamps_w)
	AM_RANGE(0x01, 0x01) AM_WRITE(funtech_coins_w)

	AM_RANGE(0x03, 0x03) AM_WRITE(funtech_vreg_w)

	AM_RANGE(0x04, 0x04) AM_READ_PORT("IN0")
	AM_RANGE(0x05, 0x05) AM_READ_PORT("IN1")
	AM_RANGE(0x06, 0x06) AM_READ_PORT("DSW1")
	AM_RANGE(0x07, 0x07) AM_READ_PORT("DSW2")

	AM_RANGE(0x10, 0x10) AM_READ_PORT("IN4")

	AM_RANGE(0x11, 0x11) AM_DEVWRITE("aysnd", ay8910_device, data_w)
	AM_RANGE(0x12, 0x12) AM_DEVWRITE("aysnd", ay8910_device, address_w)
ADDRESS_MAP_END

static INPUT_PORTS_START( funtech )
	PORT_START("IN0")
	// the buttons are all multi-purpose as it's a 2-in-1.
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_NAME("Hold 5, Bet")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Hold 1, Take, Odds, Stop 1")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("Hold 4, Double")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("Hold 3, Small, Stop 3")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("Hold 2, Big, Stop 2")

	PORT_START("IN1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_GAMBLE_BOOK )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_SPECIAL ) PORT_READ_LINE_DEVICE_MEMBER("hopper", ticket_dispenser_device, line_r)
	PORT_DIPNAME( 0x08, 0x08, "IN1-08" ) // some kind of key-out? reduces credits to 0
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, "IN1-10" ) // prevents start from working in the poker game at least
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_SERVICE_NO_TOGGLE(   0x20, IP_ACTIVE_LOW )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN4 ) // note?

	// the board contains 4 banks of 8 dipswitches, the code does not appear to read that many.
	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x01, "Reel Speed" )
	PORT_DIPSETTING(    0x01, "Low" )
	PORT_DIPSETTING(    0x00, "High" )
	PORT_DIPNAME( 0x06, 0x06, "Max Bet" )
	PORT_DIPSETTING(    0x00, "8" )
	PORT_DIPSETTING(    0x04, "16" )
	PORT_DIPSETTING(    0x02, "32" )
	PORT_DIPSETTING(    0x06, "64" )
	PORT_DIPNAME( 0x08, 0x08, "Min Bet for Bonus" )
	PORT_DIPSETTING(    0x00, "8" )
	PORT_DIPSETTING(    0x08, "16" )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) ) // unused?
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) ) // unused?
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) ) // unused?
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) ) // unused?
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x0f, 0x0f, "Main Game Percent" )
	PORT_DIPSETTING(    0x00, "50" )
	PORT_DIPSETTING(    0x08, "53" )
	PORT_DIPSETTING(    0x04, "56" )
	PORT_DIPSETTING(    0x0c, "59" )
	PORT_DIPSETTING(    0x02, "62" )
	PORT_DIPSETTING(    0x0a, "65" )
	PORT_DIPSETTING(    0x06, "68" )
	PORT_DIPSETTING(    0x0e, "71" )
	PORT_DIPSETTING(    0x01, "74" )
	PORT_DIPSETTING(    0x09, "77" )
	PORT_DIPSETTING(    0x05, "80" )
	PORT_DIPSETTING(    0x0d, "83" )
	PORT_DIPSETTING(    0x03, "86" )
	PORT_DIPSETTING(    0x0b, "89" )
	PORT_DIPSETTING(    0x07, "92" )
	PORT_DIPSETTING(    0x0f, "95" )
	PORT_DIPNAME( 0x10, 0x10, "Double Up Game Enable" )
	PORT_DIPSETTING(    0x00, "Disabled" )
	PORT_DIPSETTING(    0x10, "Enabled" )
	PORT_DIPNAME( 0x20, 0x20, "Double Up Percent" )
	PORT_DIPSETTING(    0x00, "80" )
	PORT_DIPSETTING(    0x20, "90" )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) ) // unused?
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) ) // unused?
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("IN4") // the dipswitch screen doesn't list these, maybe determined by the hopper?
	PORT_DIPNAME( 0x07, 0x07, "Coin 4 Value" )
	PORT_DIPSETTING(    0x00, "1" )
	PORT_DIPSETTING(    0x04, "2" )
	PORT_DIPSETTING(    0x02, "5" )
	PORT_DIPSETTING(    0x06, "10" )
	PORT_DIPSETTING(    0x01, "20" )
	PORT_DIPSETTING(    0x05, "25" )
	PORT_DIPSETTING(    0x03, "50" )
	PORT_DIPSETTING(    0x07, "100" )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x60, 0x00, "Payout Value" )
	PORT_DIPSETTING(    0x00, "1" )
	PORT_DIPSETTING(    0x40, "2" )
	PORT_DIPSETTING(    0x20, "10" )
	PORT_DIPSETTING(    0x60, "50" )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END


static const gfx_layout tiles8x32_layout =
{
	8,32,
	RGN_FRAC(1,1),
	8,
	{ 0, 1, 2, 3, 4 ,5, 6, 7 },
	{ 0, 8, 16, 24, 32, 40, 48, 56 },
	{ 0*64, 1*64, 2*64, 3*64, 4*64, 5*64, 6*64, 7*64, 8*64, 9*64, 10*64, 11*64, 12*64, 13*64, 14*64, 15*64, 16*64, 17*64, 18*64, 19*64, 20*64, 21*64, 22*64, 23*64, 24*64, 25*64, 26*64, 27*64, 28*64, 29*64, 30*64, 31*64 },
	32*64
};


static const gfx_layout tiles8x8_layout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ 0, 1, 2, 3 },
	{ 4, 0, 12, 8, 20, 16, 28, 24 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	32*8
};


static GFXDECODE_START( funtech )
	GFXDECODE_ENTRY( "gfx1", 0, tiles8x8_layout, 0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0, tiles8x32_layout, 0x100, 1 )
GFXDECODE_END




void fun_tech_corp_state::machine_start()
{
}

void fun_tech_corp_state::machine_reset()
{
}

static MACHINE_CONFIG_START( funtech )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80,4000000)         /* ? MHz */
	MCFG_CPU_PROGRAM_MAP(funtech_map)
	MCFG_CPU_IO_MAP(funtech_io_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", fun_tech_corp_state,  funtech_vblank_interrupt)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(512, 256)
	MCFG_SCREEN_VISIBLE_AREA(0, 512-1, 8, 256-8-1)
	MCFG_SCREEN_UPDATE_DRIVER(fun_tech_corp_state, screen_update_funtech)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", funtech)
	MCFG_PALETTE_ADD("palette", 0x200)
	MCFG_PALETTE_FORMAT(xBBBBBGGGGGRRRRR)

	MCFG_NVRAM_ADD_1FILL("nvram")

	MCFG_TICKET_DISPENSER_ADD("hopper", attotime::from_msec(50), TICKET_MOTOR_ACTIVE_HIGH, TICKET_STATUS_ACTIVE_HIGH)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("aysnd", AY8910, 1500000) /* M5255, ? MHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)
MACHINE_CONFIG_END



ROM_START( fts2in1 )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "u5.bin", 0x00000, 0x10000, CRC(ab19fd28) SHA1(a65ff732e0aaaec256cc63beff5f24419e691645) )

	ROM_REGION( 0x80000, "gfx1", 0 ) // crc printed on label matches half the data, even if chip was double size
	ROM_LOAD( "u18.bin", 0x00000, 0x80000, CRC(d1154aac) SHA1(dc03c4b7a4dfda2a30bfabaeb0ce053660961663) ) // 1ST AND 2ND HALF IDENTICAL

	ROM_REGION( 0x40000, "gfx2", 0 )
	ROM_LOAD16_BYTE( "u29.bin", 0x00000, 0x20000, CRC(ed6a1e2f) SHA1(2c72e764c7c8091a8fa1dfc257a84d61e2da0e4b) )
	ROM_LOAD16_BYTE( "u30.bin", 0x00001, 0x20000, CRC(d572bddc) SHA1(06499aeb47085a02af9eb4987ed987f9a3a397f7) )
ROM_END

GAMEL( 1993, fts2in1,  0,    funtech, funtech, fun_tech_corp_state,  0, ROT0, "Fun Tech Corporation", "Super Two In One", 0, layout_fts2in1 )
