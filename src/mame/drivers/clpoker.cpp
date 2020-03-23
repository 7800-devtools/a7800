// license:BSD-3-Clause
// copyright-holders:AJR

/*
Poker Genius (misspelled "Genuis" on title screen) by Chain Leisure

The following string is copied from the main CPU ROM into NVRAM:
"CLPOK GAME designed by FULL-LIFE  at 02-01-1994"


PCB is marked Chain Leisure CL-001

- 1x Z0840004PSC Z80 CPU
- 1x 12.000 XTAL (second XTAL location is unpopulated)
- 1x AY38910A/P sound chip
- 2x M5L8255AP-5
- 1x HM86171 RAMDAC
- 1x GM76C88L-15 SRAM (8,192 x 8 Bit)
- 1x HY6116ALP-12 CMOS SRAM (2,048 x 8 Bit)
- 1x MACH110-20JC CMOS
- 2x ATV2500H PLDs
- 1x PLSI1024-60LJ CPLD
- 1x GAL20V8A-15LNC
- 1x (should be 2x) bank of 8 dip-switches

There also are unpopulated locations that might fit a YM3812 and YM3014.
*/

#include "emu.h"
#include "screen.h"
#include "speaker.h"
#include "cpu/z80/z80.h"
#include "machine/i8255.h"
#include "machine/nvram.h"
#include "machine/ticket.h"
#include "sound/ay8910.h"
#include "video/ramdac.h"

class clpoker_state : public driver_device
{
public:
	clpoker_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_gfxdecode(*this, "gfxdecode")
		, m_hopper(*this, "hopper")
		, m_videoram(*this, "videoram")
	{
	}

	DECLARE_WRITE8_MEMBER(output_a_w);
	DECLARE_WRITE8_MEMBER(output_b_w);
	DECLARE_WRITE8_MEMBER(output_c_w);

	DECLARE_WRITE8_MEMBER(videoram_w);
	DECLARE_WRITE_LINE_MEMBER(vblank_w);

	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	virtual void video_start() override;

private:
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<ticket_dispenser_device> m_hopper;
	required_shared_ptr<u8> m_videoram;

	tilemap_t *m_bg_tilemap;
	tilemap_t *m_fg_tilemap;

	bool m_nmi_enable;
};

static ADDRESS_MAP_START( prg_map, AS_PROGRAM, 8, clpoker_state )
	AM_RANGE(0x0000, 0xbfff) AM_ROM
	AM_RANGE(0xc000, 0xdfff) AM_RAM_WRITE(videoram_w) AM_SHARE("videoram")
	AM_RANGE(0xe000, 0xe7ff) AM_RAM AM_SHARE("nvram")
	AM_RANGE(0xf000, 0xf000) AM_DEVWRITE("ramdac", ramdac_device, index_w)
	AM_RANGE(0xf001, 0xf001) AM_DEVWRITE("ramdac", ramdac_device, pal_w)
	AM_RANGE(0xf002, 0xf002) AM_DEVWRITE("ramdac", ramdac_device, mask_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( io_map, AS_IO, 8, clpoker_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x03) AM_DEVREADWRITE("ppi_outputs", i8255_device, read, write)
	AM_RANGE(0x10, 0x13) AM_DEVREADWRITE("ppi_inputs", i8255_device, read, write)
	AM_RANGE(0x30, 0x30) AM_DEVREAD("aysnd", ay8910_device, data_r)
	AM_RANGE(0x32, 0x32) AM_DEVWRITE("aysnd", ay8910_device, data_w)
	AM_RANGE(0x34, 0x34) AM_DEVWRITE("aysnd", ay8910_device, address_w)
	AM_RANGE(0xc0, 0xc0) AM_READNOP // mystery read at startup
ADDRESS_MAP_END

static ADDRESS_MAP_START( ramdac_map, 0, 8, clpoker_state )
	AM_RANGE(0x000, 0x3ff) AM_DEVREADWRITE("ramdac", ramdac_device, ramdac_pal_r, ramdac_rgb666_w)
ADDRESS_MAP_END

WRITE8_MEMBER(clpoker_state::output_a_w)
{
	if (data != 0xff)
	{
		machine().bookkeeping().coin_counter_w(0, BIT(data, 0));
		m_hopper->motor_w(BIT(data, 4));
	}
}

WRITE8_MEMBER(clpoker_state::output_b_w)
{
}

WRITE8_MEMBER(clpoker_state::output_c_w)
{
	m_nmi_enable = BIT(data, 1);
	if (!m_nmi_enable)
		m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
}

static INPUT_PORTS_START( clpoker )
	PORT_START("INA")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_KEYIN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_GAMBLE_KEYOUT )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_READ_LINE_DEVICE_MEMBER("hopper", ticket_dispenser_device, line_r)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_GAMBLE_BOOK )
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("INB")
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("INC")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 ) PORT_NAME("Start / Double Up")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_POKER_HOLD1 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_POKER_HOLD2 ) PORT_NAME("Hold 2 / Small")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_POKER_HOLD3 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_POKER_HOLD4 ) PORT_NAME("Hold 4 / Big")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_POKER_HOLD5 ) PORT_NAME("Hold 5 / Bet")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_POKER_CANCEL ) PORT_NAME("Cancel / Take Score")

	PORT_START("DSW1") // $E012
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Unknown ) ) // $E013
	PORT_DIPSETTING(    0x03, "0" )
	PORT_DIPSETTING(    0x02, "1" )
	PORT_DIPSETTING(    0x01, "2" )
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Coinage ) ) // $E014
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x08, "1 Coin/10 Credits" )
	PORT_DIPSETTING(    0x04, "1 Coin/20 Credits" )
	PORT_DIPSETTING(    0x00, "1 Coin/50 Credits" )
	PORT_DIPNAME( 0x30, 0x30, "Key In/Out" ) // $E015
	PORT_DIPSETTING(    0x30, "50 Credits" )
	PORT_DIPSETTING(    0x20, "100 Credits" )
	PORT_DIPSETTING(    0x10, "200 Credits" )
	PORT_DIPSETTING(    0x00, "500 Credits" )
	PORT_DIPNAME( 0xc0, 0xc0, "Max Bet" ) // $E016
	PORT_DIPSETTING(    0xc0, "20" )
	PORT_DIPSETTING(    0x80, "40" )
	PORT_DIPSETTING(    0x40, "60" )
	PORT_DIPSETTING(    0x00, "80" )

	PORT_START("DSW2") // $E017
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Unused ) ) // $E018
	PORT_DIPSETTING(    0x03, "0" )
	PORT_DIPSETTING(    0x02, "1" )
	PORT_DIPSETTING(    0x01, "2" )
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Unused ) ) // $E019
	PORT_DIPSETTING(    0x0c, "0" )
	PORT_DIPSETTING(    0x08, "1" )
	PORT_DIPSETTING(    0x04, "2" )
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Unused ) ) // $E01A
	PORT_DIPSETTING(    0x30, "0" )
	PORT_DIPSETTING(    0x20, "1" )
	PORT_DIPSETTING(    0x10, "2" )
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPNAME( 0xc0, 0xc0, DEF_STR( Unused ) ) // $E01B
	PORT_DIPSETTING(    0xc0, "0" )
	PORT_DIPSETTING(    0x80, "1" )
	PORT_DIPSETTING(    0x40, "2" )
	PORT_DIPSETTING(    0x00, "3" )
INPUT_PORTS_END


TILE_GET_INFO_MEMBER(clpoker_state::get_bg_tile_info)
{
	u16 tileno = (m_videoram[tile_index] << 8) | m_videoram[tile_index + 0x0800];
	SET_TILE_INFO_MEMBER(0, tileno, 0, 0);
}

TILE_GET_INFO_MEMBER(clpoker_state::get_fg_tile_info)
{
	u16 tileno = (m_videoram[tile_index + 0x1000] << 8) | m_videoram[tile_index + 0x1800];
	SET_TILE_INFO_MEMBER(0, tileno, 0, 0);
}

WRITE8_MEMBER(clpoker_state::videoram_w)
{
	m_videoram[offset] = data;
	(BIT(offset, 12) ? m_fg_tilemap : m_bg_tilemap)->mark_tile_dirty(offset & 0x07ff);
}

void clpoker_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(clpoker_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 64, 32);
	m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(clpoker_state::get_fg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 64, 32);
	m_fg_tilemap->set_transparent_pen(0);

	m_nmi_enable = false;
	save_item(NAME(m_nmi_enable));
}

WRITE_LINE_MEMBER(clpoker_state::vblank_w)
{
	if (m_nmi_enable)
		m_maincpu->set_input_line(INPUT_LINE_NMI, state);
}

u32 clpoker_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}


static const gfx_layout gfx_layout =
{
	8,8,
	RGN_FRAC(1,1),
	8,
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	{ 0*64, 1*64, 2*64, 3*64, 4*64, 5*64, 6*64, 7*64 },
	8*64,
};


static GFXDECODE_START( clpoker )
	GFXDECODE_ENTRY( "gfx1", 0, gfx_layout,   0x0, 1 )
GFXDECODE_END


static MACHINE_CONFIG_START( clpoker )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_12MHz / 3) // Z0840004PSC, divider not verified
	MCFG_CPU_PROGRAM_MAP(prg_map)
	MCFG_CPU_IO_MAP(io_map)

	MCFG_NVRAM_ADD_0FILL("nvram") // HY6116ALP-12

	MCFG_DEVICE_ADD("ppi_outputs", I8255, 0) // M5L8255AP-5
	MCFG_I8255_OUT_PORTA_CB(WRITE8(clpoker_state, output_a_w))
	MCFG_I8255_OUT_PORTB_CB(WRITE8(clpoker_state, output_b_w))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(clpoker_state, output_c_w))

	MCFG_DEVICE_ADD("ppi_inputs", I8255, 0) // M5L8255AP-5
	MCFG_I8255_IN_PORTA_CB(IOPORT("INA"))
	MCFG_I8255_IN_PORTB_CB(IOPORT("INB"))
	MCFG_I8255_IN_PORTC_CB(IOPORT("INC"))

	MCFG_TICKET_DISPENSER_ADD("hopper", attotime::from_msec(60), TICKET_MOTOR_ACTIVE_HIGH, TICKET_STATUS_ACTIVE_LOW)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60) // wrong
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))  // wrong
	MCFG_SCREEN_SIZE(64*8, 32*8) // wrong
	MCFG_SCREEN_VISIBLE_AREA(0*8, 64*8-1, 0*8, 32*8-1) // probably right
	MCFG_SCREEN_UPDATE_DRIVER(clpoker_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")
	MCFG_SCREEN_VBLANK_CALLBACK(WRITELINE(clpoker_state, vblank_w))

	MCFG_PALETTE_ADD("palette", 0x100)
	MCFG_RAMDAC_ADD("ramdac", ramdac_map, "palette") // HM86171

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", clpoker)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("aysnd", AY8910, XTAL_12MHz / 8) // AY38910A/P, divider not verified
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW1"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("DSW2"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.30)
MACHINE_CONFIG_END


ROM_START( clpoker )
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD( "5.u7", 0x00000, 0x10000, CRC(96b07104) SHA1(24ec1e44795add0db6215a7687ac2fd3b636980b) ) // 27512, 2nd half empty?

	ROM_REGION(0x80000, "gfx1", 0)
	ROM_LOAD32_BYTE( "1.u1", 0x00000, 0x20000, CRC(d1b5a3f1) SHA1(5a08be220b81d9502f1ed61966916384925ba569) ) // 27C010
	ROM_LOAD32_BYTE( "2.u2", 0x00001, 0x20000, CRC(00abb6b2) SHA1(3123c2e18d987895cb1d3359bf2765343289037b) ) // 27C010
	ROM_LOAD32_BYTE( "3.u3", 0x00002, 0x20000, CRC(fcccef5a) SHA1(a0bdba24a6a9ca8aa8b7fdfee10ace3cb17600b4) ) // 27C010
	ROM_LOAD32_BYTE( "4.u4", 0x00003, 0x20000, CRC(be707d36) SHA1(b1cb9dc387a54d895cfaedfbc015598151ddab38) ) // 27C010

	ROM_REGION(0x1000, "pld", 0)
	ROM_LOAD( "plsi1024-60lj.pl1", 0x00, 0x200,  NO_DUMP )
	ROM_LOAD( "atv2500h.pl2",      0x00, 0x200,  NO_DUMP )
	ROM_LOAD( "atv2500h.pl3",      0x00, 0x200,  NO_DUMP )
	ROM_LOAD( "mach110-20jc.pl4",  0x00, 0x200,  NO_DUMP )
	ROM_LOAD( "gal20v8a.pl5",      0x00, 0x157,  NO_DUMP )
ROM_END


GAME( 1994, clpoker,  0, clpoker, clpoker, clpoker_state, 0, ROT0, "Chain Leisure", "Poker Genius", MACHINE_SUPPORTS_SAVE ) // Year taken from string in main CPU ROM
