// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/**********************************************************************************************

M14 Hardware (c) 1979 Irem

driver by Angelo Salese

TODO:
- Actual discrete sound emulation;
- Colors might be not 100% accurate (needs screenshots from the real thing);
- What are the high 4 bits in the colorram for? They are used on the mahjong tiles only,
  left-over or something more important?
- I/Os are grossly mapped;
- ball and paddle drawing are a guesswork;

Notes:
- Unlike most Arcade games, if you call a ron but you don't have a legit hand you'll automatically
  lose the match. This is commonly named chombo in rii'chi mahjong rules;
- After getting a completed hand, press start 1 + ron + discard at the same time to go back
  into attract mode (!);

==============================================================================================
x (Mystery Rom)
(c)1978-1981? Irem?
PCB No. :M14S-2
    :M14L-2
CPU :NEC D8085AC
Sound   :?
OSC :6MHz x2

mgpb1.bin
mgpa2.bin
mgpa3.bin
mgpa4.bin
mgpa5.bin
mgpb6.bin
mgpa7.bin
mgpb8.bin

mgpa9.bin
mgpa10.bin


--- Team Japump!!! ---
Dumped by Chackn
01/30/2000

**********************************************************************************************/

#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "screen.h"
#include "sound/samples.h"
#include "speaker.h"


class m14_state : public driver_device
{
public:
	m14_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_video_ram(*this, "video_ram"),
		m_color_ram(*this, "color_ram"),
		m_samples(*this,"samples")
		{ }

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_shared_ptr<uint8_t> m_video_ram;
	required_shared_ptr<uint8_t> m_color_ram;
	required_device<samples_device> m_samples;

	DECLARE_WRITE8_MEMBER(m14_vram_w);
	DECLARE_WRITE8_MEMBER(m14_cram_w);
	DECLARE_READ8_MEMBER(m14_rng_r);
	DECLARE_WRITE8_MEMBER(output_w);
	DECLARE_WRITE8_MEMBER(ball_x_w);
	DECLARE_WRITE8_MEMBER(ball_y_w);
	DECLARE_WRITE8_MEMBER(paddle_x_w);
	DECLARE_WRITE8_MEMBER(sound_w);

	DECLARE_INPUT_CHANGED_MEMBER(left_coin_inserted);
	DECLARE_INPUT_CHANGED_MEMBER(right_coin_inserted);

	TILE_GET_INFO_MEMBER(m14_get_tile_info);
	void draw_ball_and_paddle(bitmap_ind16 &bitmap, const rectangle &cliprect);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(m14);
	uint32_t screen_update_m14(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(m14_irq);

private:
	/* video-related */
	tilemap_t  *m_m14_tilemap;

	/* input-related */
	//uint8_t m_hop_mux;
	uint8_t m_ballx,m_bally;
	uint8_t m_paddlex;
};


/*************************************
 *
 *  Video Hardware
 *
 *************************************/

PALETTE_INIT_MEMBER(m14_state, m14)
{
	int i;
	const rgb_t green_pen = rgb_t(pal1bit(0), pal1bit(1), pal1bit(0));

	for (i = 0; i < 0x20; i++)
	{
		rgb_t color;

		if (i & 0x01)
			color = rgb_t(pal1bit(i >> 1), pal1bit(i >> 2), pal1bit(i >> 3));
		else
			color = (i & 0x10) ? rgb_t::white() : green_pen;

		palette.set_pen_color(i, color);
	}
}

TILE_GET_INFO_MEMBER(m14_state::m14_get_tile_info)
{
	int code = m_video_ram[tile_index];
	int color = m_color_ram[tile_index] & 0x0f;

	/* colorram & 0xf0 used but unknown purpose*/

	SET_TILE_INFO_MEMBER(0,
			code,
			color,
			0);
}

void m14_state::video_start()
{
	m_m14_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(m14_state::m14_get_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 32, 32);


}

void m14_state::draw_ball_and_paddle(bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int xi,yi;
	//const rgb_t white_pen =  rgb_t::white();
	const uint8_t white_pen = 0x1f;
	const int xoffs = -8; // matches left-right wall bounces
	const int p_ybase = 184; // matches ball bounce to paddle
	int resx,resy;

	// draw ball
	for(xi=0;xi<4;xi++)
		for(yi=0;yi<4;yi++)
		{
			resx = flip_screen() ?  32*8-(m_ballx+xi+xoffs) : m_ballx+xi+xoffs;
			resy = flip_screen() ?  28*8-(m_bally+yi) :       m_bally+yi;

			if(cliprect.contains(resx,resy))
				bitmap.pix16(resy, resx) = m_palette->pen(white_pen);
		}

	// draw paddle
	for(xi=0;xi<16;xi++)
		for(yi=0;yi<4;yi++)
		{
			resx = flip_screen() ? 32*8-(m_paddlex+xi+xoffs) : (m_paddlex+xi+xoffs);
			resy = flip_screen() ? 28*8-(p_ybase+yi) :         p_ybase+yi;

			if(cliprect.contains(resx,resy))
				bitmap.pix16(resy, resx) = m_palette->pen(white_pen);
		}


}


uint32_t m14_state::screen_update_m14(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_m14_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	draw_ball_and_paddle(bitmap,cliprect);

	return 0;
}


WRITE8_MEMBER(m14_state::m14_vram_w)
{
	m_video_ram[offset] = data;
	m_m14_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(m14_state::m14_cram_w)
{
	m_color_ram[offset] = data;
	m_m14_tilemap->mark_tile_dirty(offset);
}

/*************************************
 *
 *  I/O
 *
 *************************************/

READ8_MEMBER(m14_state::m14_rng_r)
{
	/* graphic artifacts happens if this doesn't return random values. */
	/* guess directly tied to screen frame number */
	return (m_screen->frame_number() & 0x7f) | (ioport("IN1")->read() & 0x80);
}

WRITE8_MEMBER(m14_state::output_w)
{
	/* ---- x--- active after calling a winning hand */
	/* ---- --x- lamp? */
	/* ---- ---x flip screen */
	//m_hop_mux = data & 2;
	flip_screen_set(data & 1);
	//popmessage("%02x",data);
}

WRITE8_MEMBER(m14_state::ball_x_w)
{
	m_ballx = data;
}

WRITE8_MEMBER(m14_state::ball_y_w)
{
	m_bally = data;
}

WRITE8_MEMBER(m14_state::paddle_x_w)
{
	m_paddlex = data;
}

/*************************************
 *
 *  Sound section
 *
 *************************************/

static const char *const m14_sample_names[] =
{
	"*ptrmj",
	"wall_hit", // 1
	"tile_hit", // 2
	"tick",     // 0x40
	"ball_drop", // 8
	"paddle_hit",
	nullptr
};

WRITE8_MEMBER(m14_state::sound_w)
{
	switch(data)
	{
		case 1: // wall hit
			m_samples->start(0,0);
			break;
		case 2: // tile hit
			m_samples->start(1,1);
			break;
		case 8: // ball drop
			m_samples->start(3,3);
			break;
		case 0x40: // tick
			m_samples->start(2,2);
			break;
		case 0x80:
		case 0: // flips with previous, unknown purpose
			break;
		default:
			logerror("%s: Sound start with %02x param\n",m_samples->tag(),data);
			break;
	}
}


/*************************************
 *
 *  Memory Map
 *
 *************************************/

static ADDRESS_MAP_START( m14_map, AS_PROGRAM, 8, m14_state )
	AM_RANGE(0x0000, 0x1fff) AM_ROM
	AM_RANGE(0x2000, 0x23ff) AM_RAM
	AM_RANGE(0xe000, 0xe3ff) AM_RAM_WRITE(m14_vram_w) AM_SHARE("video_ram")
	AM_RANGE(0xe400, 0xe7ff) AM_RAM_WRITE(m14_cram_w) AM_SHARE("color_ram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( m14_io_map, AS_IO, 8, m14_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0xf8, 0xf8) AM_READ_PORT("AN_PADDLE") AM_WRITE(ball_x_w)
	AM_RANGE(0xf9, 0xf9) AM_READ_PORT("IN0") AM_WRITE(ball_y_w)
	AM_RANGE(0xfa, 0xfa) AM_READ(m14_rng_r) AM_WRITE(paddle_x_w)
	AM_RANGE(0xfb, 0xfb) AM_READ_PORT("DSW") AM_WRITE(output_w)
	AM_RANGE(0xfc, 0xfc) AM_WRITE(sound_w)
ADDRESS_MAP_END

/*************************************
 *
 *  Input Port Definitions
 *
 *************************************/

INPUT_CHANGED_MEMBER(m14_state::left_coin_inserted)
{
	/* left coin insertion causes a rst6.5 (vector 0x34) */
	if (newval)
		m_maincpu->set_input_line(I8085_RST65_LINE, HOLD_LINE);
}

INPUT_CHANGED_MEMBER(m14_state::right_coin_inserted)
{
	/* right coin insertion causes a rst5.5 (vector 0x2c) */
	if (newval)
		m_maincpu->set_input_line(I8085_RST55_LINE, HOLD_LINE);
}

static INPUT_PORTS_START( m14 )
	PORT_START("AN_PADDLE")
	PORT_BIT( 0xff, 0x00, IPT_PADDLE ) PORT_MINMAX(0,0xff) PORT_SENSITIVITY(10) PORT_KEYDELTA(10) PORT_CENTERDELTA(0) PORT_REVERSE

	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("P1 Fire / Discard")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("P1 Ron")
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Coinage ) )
	PORT_DIPSETTING(    0x04, u8"Coin A 5\u30de\u30a4 Coin B 1\u30de\u30a4" )
	PORT_DIPSETTING(    0x00, u8"Coin A & B 1\u30de\u30a4" )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_CUSTOM ) PORT_VBLANK("screen")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_START1 )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("IN1")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("P1 Reach")

	PORT_START("DSW") //this whole port is stored at work ram $2112.
	PORT_DIPNAME( 0x01, 0x01, "Show available tiles" ) // difficulty even
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Cabinet ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Cocktail ) )
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

	PORT_START("FAKE")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_COIN1 ) PORT_IMPULSE(1) PORT_CHANGED_MEMBER(DEVICE_SELF, m14_state,left_coin_inserted, 0) //coin x 5
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_COIN2 ) PORT_IMPULSE(1) PORT_CHANGED_MEMBER(DEVICE_SELF, m14_state,right_coin_inserted, 0) //coin x 1
INPUT_PORTS_END

static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,1),
	1,
	{ RGN_FRAC(0,1) },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8
};

static GFXDECODE_START( m14 )
	GFXDECODE_ENTRY( "gfx1", 0, charlayout,     0, 0x10 )
GFXDECODE_END

INTERRUPT_GEN_MEMBER(m14_state::m14_irq)
{
	device.execute().set_input_line(I8085_RST75_LINE, ASSERT_LINE);
	device.execute().set_input_line(I8085_RST75_LINE, CLEAR_LINE);
}

void m14_state::machine_start()
{
	//save_item(NAME(m_hop_mux));
	save_item(NAME(m_ballx));
	save_item(NAME(m_bally));
	save_item(NAME(m_paddlex));
}

void m14_state::machine_reset()
{
	//m_hop_mux = 0;
}


static MACHINE_CONFIG_START( m14 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",I8085A,6000000) //guess: 6 Mhz internally divided by 2
	MCFG_CPU_PROGRAM_MAP(m14_map)
	MCFG_CPU_IO_MAP(m14_io_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", m14_state, m14_irq)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) //not accurate
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 0*8, 28*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(m14_state, screen_update_m14)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", m14)
	MCFG_PALETTE_ADD("palette", 0x20)
	MCFG_PALETTE_INIT_OWNER(m14_state, m14)


	/* sound hardware */

	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("samples", SAMPLES, 0)
	MCFG_SAMPLES_CHANNELS(5)
	MCFG_SAMPLES_NAMES(m14_sample_names)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.6)

//  MCFG_SOUND_ADD("discrete", DISCRETE, 0)
//  MCFG_DISCRETE_INTF(m14)
//  MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/


ROM_START( ptrmj )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "mgpb1.bin",   0x0000, 0x0400, CRC(47c041b8) SHA1(e834c375e689f99a13964863fc9847a8e148ec91) )
	ROM_LOAD( "mgpa2.bin",   0x0400, 0x0400, CRC(cf8bfa23) SHA1(091055e803255f1b5520f50b31af7135d71d0a40) )
	ROM_LOAD( "mgpa3.bin",   0x0800, 0x0400, CRC(a07a3093) SHA1(5b86bb11e83c06f828956e7db6dd2c105b023b03) )
	ROM_LOAD( "mgpa4.bin",   0x0c00, 0x0400, CRC(a420241c) SHA1(7497d90014dabb49f9db1d5d8e3014c634045725) )
	ROM_LOAD( "mgpa5.bin",   0x1000, 0x0400, CRC(a2df92a3) SHA1(97a3d4b188d26f172881f8cf86bdd83d549f5b74) )
	ROM_LOAD( "mgpb6.bin",   0x1400, 0x0400, CRC(f5c0fcd4) SHA1(14e2d04be105caeb221dfc226f84cb1722ae2627) )
	ROM_LOAD( "mgpa7.bin",   0x1800, 0x0400, CRC(56ed7eb2) SHA1(acf954bf4daadb225475937dd3c36baa996f7b65) )
	ROM_LOAD( "mgpb8.bin",   0x1c00, 0x0400, CRC(3ecb8214) SHA1(8575bcb49f693aa8798b8a7d9a76392bfcc90e0e) )

	ROM_REGION( 0x0800, "gfx1", 0 )
	ROM_LOAD( "mgpa9.bin",   0x0000, 0x0400, CRC(cb68b4ec) SHA1(2cf596affb155ae38729fcd95cae424073faf74d) )
	ROM_LOAD( "mgpa10.bin",  0x0400, 0x0400, CRC(e1a4ebdc) SHA1(d9df42424ede17f0634d8d0a56c0374a33c55333) )
ROM_END

GAME( 1979, ptrmj,  0,       m14,  m14, m14_state,  0, ROT0, "Irem", "PT Reach Mahjong (Japan)", MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE ) // was already Irem according to the official flyer
