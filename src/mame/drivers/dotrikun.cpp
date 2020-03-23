// license:BSD-3-Clause
// copyright-holders:Takahiro Nogi, hap
/***************************************************************************

Dottori Kun (Head On's mini game)
(c)1990 SEGA

Driver by Takahiro Nogi (nogi@kt.rim.or.jp) 1999/12/15 -


CPU   : Z-80 (4MHz)
SOUND : (none)

14479.MPR  ; PRG (FIRST VER)
14479A.MPR ; PRG (NEW VER)

* This game is only for the test of cabinet
* BackRaster = WHITE on the FIRST version.
* BackRaster = BLACK on the NEW version.
* On the NEW version, push COIN-SW as TEST MODE.
* 0000-3FFF:ROM 8000-85FF:VRAM(128x96) 8600-87FF:WORK-RAM


TODO:
- improve video timing more (see http://www.chrismcovell.com/dottorikun.html)

***************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "screen.h"

#include "dotrikun.lh"


class dotrikun_state : public driver_device
{
public:
	dotrikun_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_vram(*this, "vram"),
		m_interrupt_timer(*this, "interrupt"),
		m_scanline_off_timer(*this, "scanline_off")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_shared_ptr<uint8_t> m_vram;
	required_device<timer_device> m_interrupt_timer;
	required_device<timer_device> m_scanline_off_timer;

	uint8_t m_vram_latch;
	uint8_t m_color;

	DECLARE_WRITE8_MEMBER(vram_w);
	DECLARE_WRITE8_MEMBER(color_w);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(scanline_off);
	TIMER_DEVICE_CALLBACK_MEMBER(scanline_on);

	virtual void machine_start() override;
	virtual void machine_reset() override;
};

TIMER_DEVICE_CALLBACK_MEMBER(dotrikun_state::interrupt)
{
	generic_pulse_irq_line(*m_maincpu, 0, 1);
}

TIMER_DEVICE_CALLBACK_MEMBER(dotrikun_state::scanline_off)
{
	m_maincpu->set_unscaled_clock(XTAL_4MHz);
}

TIMER_DEVICE_CALLBACK_MEMBER(dotrikun_state::scanline_on)
{
	// on vram fetch(every 8 pixels during active display), z80 is stalled for 2 clocks
	if (param < 192)
	{
		m_maincpu->set_unscaled_clock(XTAL_4MHz * 0.75);
		m_scanline_off_timer->adjust(m_screen->time_until_pos(param, 128));
	}

	// vblank interrupt
	if (param == 191)
		m_interrupt_timer->adjust(m_screen->time_until_pos(param, 128+64));
}


/*************************************
 *
 *  Video emulation
 *
 *************************************/

WRITE8_MEMBER(dotrikun_state::vram_w)
{
	m_screen->update_now();
	m_vram[offset] = data;
}

WRITE8_MEMBER(dotrikun_state::color_w)
{
	// d0-d2: fg palette
	// d3-d5: bg palette
	// d6,d7: N/C
	m_screen->update_now();
	m_color = data & 0x3f;
}


uint32_t dotrikun_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		for (int x = cliprect.min_x; x <= cliprect.max_x; x++)
		{
			// vram fetch
			if ((x & 7) == 0)
				m_vram_latch = m_vram[x >> 3 | y >> 1 << 4];

			bitmap.pix16(y, x) = (m_vram_latch >> (~x & 7) & 1) ? m_color & 7 : m_color >> 3;
		}
	}

	return 0;
}


/*************************************
 *
 *  Address maps
 *
 *************************************/

static ADDRESS_MAP_START( dotrikun_map, AS_PROGRAM, 8, dotrikun_state )
	AM_RANGE(0x0000, 0x3fff) AM_ROM
	AM_RANGE(0x8000, 0x85ff) AM_RAM_WRITE(vram_w) AM_SHARE("vram")
	AM_RANGE(0x8600, 0x87ff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( io_map, AS_IO, 8, dotrikun_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x00) AM_MIRROR(0xff) AM_READ_PORT("INPUTS") AM_WRITE(color_w)
ADDRESS_MAP_END


/*************************************
 *
 *  Input ports
 *
 *************************************/

static INPUT_PORTS_START( dotrikun )
	PORT_START("INPUTS")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN1 )
INPUT_PORTS_END


/*************************************
 *
 *  Machine driver
 *
 *************************************/

void dotrikun_state::machine_start()
{
	save_item(NAME(m_vram_latch));
	save_item(NAME(m_color));
}

void dotrikun_state::machine_reset()
{
	m_vram_latch = 0;
	m_color = 0;
}

static MACHINE_CONFIG_START( dotrikun )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_4MHz)
	MCFG_CPU_PROGRAM_MAP(dotrikun_map)
	MCFG_CPU_IO_MAP(io_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scanline_on", dotrikun_state, scanline_on, "screen", 0, 1)
	MCFG_TIMER_DRIVER_ADD("scanline_off", dotrikun_state, scanline_off)
	MCFG_TIMER_DRIVER_ADD("interrupt", dotrikun_state, interrupt)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_4MHz, 128+128, 0, 128, 192+64, 0, 192)
	MCFG_SCREEN_UPDATE_DRIVER(dotrikun_state, screen_update)
	MCFG_SCREEN_VIDEO_ATTRIBUTES(VIDEO_ALWAYS_UPDATE)
	MCFG_SCREEN_PALETTE("palette")
	MCFG_PALETTE_ADD_3BIT_RGB("palette")

	/* no sound hardware */
MACHINE_CONFIG_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( dotrikun )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "14479a.mpr", 0x0000, 0x4000, CRC(b77a50db) SHA1(2a5d812d39f0f58f5c3e1b46f80aca75aa225115) )
ROM_END

ROM_START( dotrikun2 )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "14479.mpr",  0x0000, 0x4000, CRC(a6aa7fa5) SHA1(4dbea33fb3541fdacf2195355751078a33bb30d5) )
ROM_END

ROM_START( dotriman )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "14479a.mpr", 0x0000, 0x4000, CRC(4ba6d2f5) SHA1(db805e9121ecbd41fac4593b58d7f071e7dbc720) )
ROM_END


GAMEL(1990, dotrikun,  0,        dotrikun, dotrikun, dotrikun_state, 0, ROT0, "Sega", "Dottori Kun (new version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW, layout_dotrikun )
GAMEL(1990, dotrikun2, dotrikun, dotrikun, dotrikun, dotrikun_state, 0, ROT0, "Sega", "Dottori Kun (old version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW, layout_dotrikun )
GAMEL(2016, dotriman,  dotrikun, dotrikun, dotrikun, dotrikun_state, 0, ROT0, "hack (Chris Covell)", "Dottori-Man Jr.", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW, layout_dotrikun )
