// license:BSD-3-Clause
// copyright-holders:David Haywood
/*

Sega M1 hardware (837-7571) (PCB)

Sega Bingo Multicart (837-10675) (Sticker on PCB)

used for redemption / gambling style machines in a satellite setup

based on Caribbean Boule the following hardware setup is used

One X-Board (segaxbd.cpp) drives a large rear-projection monitor which all players view to see the main game progress.

Multiple M1 boards ("satellite" board) for each player for them to view information privately.

One 'link' board which connects everything together.  The link board has audio hardware, a 68K, and a Z80 as
well as a huge bank of UARTS and toslink connectors, but no video. It's possible the main game logic runs
on the 'link' board.


Unfortunately we don't have any dumps of anything other than an M1 board right now.

---

is this related to (or a component of?) bingoc.cpp, the EPR numbers are much lower there tho
so it's probably an earlier version of the same thing or one of the 'link' boards?

uses s24 style tilemaps (ram based?)


*/


#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "cpu/z80/z80.h"
#include "machine/315_5296.h"
#include "machine/gen_latch.h"
#include "machine/i8251.h"
#include "machine/mb8421.h"
#include "sound/2612intf.h"
#include "video/segaic24.h"
#include "screen.h"
#include "speaker.h"



class segam1_state : public driver_device
{
public:
	segam1_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
		, m_palette(*this, "palette")
		, m_paletteram(*this, "paletteram")
		, m_tile(*this, "tile")
		, m_mixer(*this, "mixer")
		, m_ymsnd(*this, "ymsnd")
	{ }

	DECLARE_WRITE16_MEMBER(paletteram_w);
	DECLARE_WRITE8_MEMBER(sound_a0_bank_w);

	virtual void machine_start() override;
	virtual void video_start() override;

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_shared_ptr<u16> m_paletteram;
	required_device<segas24_tile_device> m_tile;
	required_device<segas24_mixer_device> m_mixer;
	required_device<ym3438_device> m_ymsnd;
};

void segam1_state::machine_start()
{
	membank("soundbank")->configure_entries(0x00, 0x10, memregion("audiocpu")->base(), 0x2000);
}

void segam1_state::video_start()
{
}

WRITE8_MEMBER(segam1_state::sound_a0_bank_w)
{
	membank("soundbank")->set_entry(data & 0x0f);
}

// 315-5242

WRITE16_MEMBER(segam1_state::paletteram_w)
{
	COMBINE_DATA(&m_paletteram[offset]);
	data = m_paletteram[offset];

	u16 r = (data & 0x00f) << 4;
	if(data & 0x1000)
		r |= 8;

	u16 g = data & 0x0f0;
	if(data & 0x2000)
		g |= 8;

	u16 b = (data & 0xf00) >> 4;
	if(data & 0x4000)
		b |= 8;

	r |= r >> 5;
	g |= g >> 5;
	b |= b >> 5;

	m_palette->set_pen_color(offset, rgb_t(r, g, b));

	if(data & 0x8000) {
		r = 255-0.6*(255-r);
		g = 255-0.6*(255-g);
		b = 255-0.6*(255-b);
	} else {
		r = 0.6*r;
		g = 0.6*g;
		b = 0.6*b;
	}
	m_palette->set_pen_color(offset+m_palette->entries()/2, rgb_t(r, g, b));
}


// copied from segas24.cpp
namespace {
	struct layer_sort {
		layer_sort(segas24_mixer_device *_mixer) { mixer = _mixer; }

		bool operator()(int l1, int l2) {
			static const int default_pri[12] = { 0, 1, 2, 3, 4, 5, 6, 7, -4, -3, -2, -1 };
			int p1 = mixer->get_reg(l1) & 7;
			int p2 = mixer->get_reg(l2) & 7;
			if(p1 != p2)
				return p1 - p2 < 0;
			return default_pri[l2] - default_pri[l1] < 0;
		}

		segas24_mixer_device *mixer;
	};
}

// copied from segas24.cpp
uint32_t segam1_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	if (m_mixer->get_reg(13) & 1)
	{
		bitmap.fill(m_palette->black_pen());
		return 0;
	}

	screen.priority().fill(0);
	bitmap.fill(0, cliprect);

	std::vector<int> order;
	order.resize(12);
	for(int i=0; i<12; i++)
		order[i] = i;

	std::sort(order.begin(), order.end(), layer_sort(m_mixer.target()));

	int level = 0;
	for(int i=0; i<12; i++)
		if(order[i] < 8)
			m_tile->draw(screen, bitmap, cliprect, order[i], level, 0);
		else
			level++;

	return 0;
}



static ADDRESS_MAP_START( segam1_map, AS_PROGRAM, 16, segam1_state )
	AM_RANGE(0x000000, 0x07ffff) AM_ROM
	AM_RANGE(0x340000, 0x340fff) AM_DEVREADWRITE8("dpram", mb8421_device, right_r, right_w, 0x00ff)
	AM_RANGE(0xb00000, 0xb0ffff) AM_DEVREADWRITE("tile", segas24_tile_device, tile_r, tile_w)
	AM_RANGE(0xb20000, 0xb20001) AM_WRITENOP        /* Horizontal split position (ABSEL) */
	AM_RANGE(0xb40000, 0xb40001) AM_WRITENOP        /* Scanline trigger position (XHOUT) */
	AM_RANGE(0xb60000, 0xb60001) AM_WRITENOP        /* Frame trigger position (XVOUT) */
	AM_RANGE(0xb70000, 0xb70001) AM_WRITENOP        /* Synchronization mode */
	AM_RANGE(0xb80000, 0xbfffff) AM_DEVREADWRITE("tile", segas24_tile_device, char_r, char_w)
	AM_RANGE(0xc00000, 0xc03fff) AM_RAM_WRITE(paletteram_w) AM_SHARE("paletteram")
	AM_RANGE(0xc04000, 0xc0401f) AM_DEVREADWRITE("mixer", segas24_mixer_device, read, write)
	AM_RANGE(0xe00000, 0xe0001f) AM_DEVREADWRITE8("io1", sega_315_5296_device, read, write, 0x00ff)
	AM_RANGE(0xe40000, 0xe40001) AM_READ_PORT("INX")
	AM_RANGE(0xe40002, 0xe40003) AM_READ_PORT("INY")
	AM_RANGE(0xe40004, 0xe40005) AM_DEVWRITE8("soundlatch", generic_latch_8_device, write, 0x00ff)
	AM_RANGE(0xe40006, 0xe40007) AM_WRITENOP
	AM_RANGE(0xe40008, 0xe40009) AM_READ_PORT("INZ")
	AM_RANGE(0xe80000, 0xe8001f) AM_DEVREADWRITE8("io2", sega_315_5296_device, read, write, 0x00ff)
	AM_RANGE(0xf00000, 0xf03fff) AM_MIRROR(0x0fc000) AM_RAM // NVRAM?
ADDRESS_MAP_END

static ADDRESS_MAP_START( segam1_sound_map, AS_PROGRAM, 8, segam1_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0xa000, 0xbfff) AM_ROMBANK("soundbank")
	AM_RANGE(0xf000, 0xffff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( unkm1_sound_map, AS_PROGRAM, 8, segam1_state )
	AM_RANGE(0xe000, 0xefff) AM_RAM
	AM_IMPORT_FROM(segam1_sound_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( segam1_sound_io_map, AS_IO, 8, segam1_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x80, 0x83) AM_DEVREADWRITE("ymsnd", ym3438_device, read, write)
	AM_RANGE(0xa0, 0xa0) AM_WRITE(sound_a0_bank_w)
	AM_RANGE(0xc0, 0xc0) AM_DEVREAD("soundlatch", generic_latch_8_device, read) AM_WRITENOP
ADDRESS_MAP_END

static ADDRESS_MAP_START( segam1_comms_map, AS_PROGRAM, 8, segam1_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x9fff) AM_RAM
	AM_RANGE(0xa000, 0xa7ff) AM_DEVREADWRITE("dpram", mb8421_device, left_r, left_w)
	AM_RANGE(0xc000, 0xc000) AM_DEVREADWRITE("uart", i8251_device, data_r, data_w)
	AM_RANGE(0xc001, 0xc001) AM_DEVREADWRITE("uart", i8251_device, status_r, control_w)
	AM_RANGE(0xe003, 0xe003) AM_WRITENOP // ???
ADDRESS_MAP_END


static INPUT_PORTS_START( segam1 )
	PORT_START("INA")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("INB")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("INC")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("IND")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("INE")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("INF")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("ING")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("INX")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INY")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INZ")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END




static MACHINE_CONFIG_START( segam1 )

	MCFG_CPU_ADD("maincpu", M68000, XTAL_20MHz/2)
	MCFG_CPU_PROGRAM_MAP(segam1_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", segam1_state, irq4_line_hold)

	MCFG_CPU_ADD("audiocpu", Z80, 4000000) // unknown clock
	MCFG_CPU_PROGRAM_MAP(segam1_sound_map)
	MCFG_CPU_IO_MAP(segam1_sound_io_map)

	MCFG_CPU_ADD("m1comm", Z80, 4000000) // unknown clock
	MCFG_CPU_PROGRAM_MAP(segam1_comms_map)

	MCFG_DEVICE_ADD("io1", SEGA_315_5296, 0) // unknown clock
	MCFG_315_5296_IN_PORTA_CB(IOPORT("INA"))
	MCFG_315_5296_IN_PORTB_CB(IOPORT("INB"))
	MCFG_315_5296_IN_PORTC_CB(IOPORT("INC"))
	MCFG_315_5296_IN_PORTD_CB(IOPORT("IND"))
	MCFG_315_5296_IN_PORTE_CB(IOPORT("INE"))
	MCFG_315_5296_IN_PORTF_CB(IOPORT("INF"))

	MCFG_DEVICE_ADD("io2", SEGA_315_5296, 0) // unknown clock
	MCFG_315_5296_IN_PORTG_CB(IOPORT("ING"))

	MCFG_DEVICE_ADD("uart", I8251, 4000000) // unknown clock

	MCFG_DEVICE_ADD("dpram", MB8421, 0)
	MCFG_MB8421_INTL_HANDLER(INPUTLINE("m1comm", 0))

	MCFG_S24TILE_DEVICE_ADD("tile", 0x3fff)
	MCFG_S24TILE_DEVICE_PALETTE("palette")
	MCFG_S24MIXER_DEVICE_ADD("mixer")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_VIDEO_ATTRIBUTES(VIDEO_UPDATE_AFTER_VBLANK)
	MCFG_SCREEN_RAW_PARAMS(16000000, 656, 0, 496, 424, 0, 384) // copied from segas24.cpp; may not be accurate
	MCFG_SCREEN_UPDATE_DRIVER(segam1_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 8192*2)

	// sound hardware
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")
	MCFG_GENERIC_LATCH_DATA_PENDING_CB(INPUTLINE("audiocpu", INPUT_LINE_NMI))

	MCFG_SOUND_ADD("ymsnd", YM3438, 8000000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.40)
	//MCFG_YM2612_IRQ_HANDLER(WRITELINE(segam1_state, ym3438_irq_handler))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( unkm1, segam1 )
	MCFG_CPU_MODIFY("audiocpu")
	MCFG_CPU_PROGRAM_MAP(unkm1_sound_map)

	MCFG_CPU_MODIFY("m1comm")
	MCFG_DEVICE_DISABLE() // not dumped yet
MACHINE_CONFIG_END


ROM_START( bingpty ) // 1994/05/01 string
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68000 Code */
	ROM_LOAD16_BYTE( "epr-16648b.bin", 0x00000, 0x20000, CRC(e4fceb4c) SHA1(0a248bb328d2f6d72d540baefbe62838f4b76585) )
	ROM_LOAD16_BYTE( "epr-16649b.bin", 0x00001, 0x20000, CRC(736d8bbd) SHA1(c359ad513d4a7693cbb1a27ce26f89849e894d05) )

	ROM_REGION( 0x20000, "audiocpu", 0 ) /* Z80 Code */
	ROM_LOAD( "epr-14845.bin", 0x00000, 0x20000, CRC(90d47101) SHA1(7bc002c104e3dbde1986aaec54112d5658eab523) )

	ROM_REGION( 0x8000, "m1comm", 0 ) /* Z80 Code */
	ROM_LOAD( "epr-14221a.bin", 0x00000, 0x8000, CRC(a13e67a4) SHA1(4cd269c7f04a64ae7806c8784f86bf6553a25d85) )

	// dumps of the X-Board part, and the LINK PCB are missing.
ROM_END

ROM_START( unkm1 ) // 1992.01.31 string
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68000 Code */
	ROM_LOAD16_BYTE( "epr-14427.ic8", 0x00000, 0x40000, CRC(2d904fc6) SHA1(7062f47d77d09906420118c85e1cb565bec345a7) )
	ROM_LOAD16_BYTE( "epr-14428.ic7", 0x00001, 0x40000, CRC(97a317f4) SHA1(19bc4cf6b6c580caa44f36c929b445ed94b2d9eb) )

	ROM_REGION( 0x20000, "audiocpu", 0 ) /* Z80 Code */
	ROM_LOAD( "epr-14429.ic104", 0x00000, 0x20000, CRC(1ff8262d) SHA1(fb90bd877b2dc65eb3e5495d6e21dee1f871fb44) )

	ROM_REGION( 0x8000, "m1comm", 0 )
	ROM_LOAD( "unkm1_comm.bin", 0x00000, 0x8000, NO_DUMP ) // CPU almost certainly exists, but not even type is confirmed

	ROM_REGION( 0x100, "plds", 0 )
	ROM_LOAD( "315-5472-01.ic22", 0x000, 0x0eb, CRC(828ee6e2) SHA1(f32dd0f6297cc8bd3049be4bca502c0f8ec738cf) )
	// dumps of the X-Board part, and the LINK PCB are missing.
ROM_END

GAME( 1994, bingpty,    0,        segam1,    segam1, segam1_state,    0, ROT0,  "Sega", "Bingo Party Multicart (Rev B) (M1 Satellite board)", MACHINE_NOT_WORKING )
GAME( 1992, unkm1,      0,        unkm1,     segam1, segam1_state,    0, ROT0,  "Sega", "Unknown Sega gambling game (M1 Satellite board)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
