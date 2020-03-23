// license:BSD-3-Clause
// copyright-holders:Robbbert
/***************************************************************************************************

    Skeleton driver for Memorex 2178

    Chips: Z80A, N8X305N, 2x B8452A, 4x D4016C-3, 2x HD468A50P, HD46505SP-1
    Crystal: 18.8696MHz
    There is a large piezo-beeper.

    TODO:
    - Connect up the beeper
    - RS232 not working (6850 parameters are /64, 8 bit, 2 stop bit, IRQ when key pressed)
    - Unknown port i/o 80
    - Unknown memory i/o C000, 4000
    - Need schematic / tech manual
    - Gets stuck waiting for E011 to become zero somehow. If you skip that, a status line appears.
    - Doesn't seem to be any dips, looks like all settings and modes are controlled by keystrokes.

***************************************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "cpu/z80/z80.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/keyboard.h"
#include "video/mc6845.h"
#include "screen.h"


class mx2178_state : public driver_device
{
public:
	mx2178_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_palette(*this, "palette")
		, m_p_videoram(*this, "videoram")
		, m_maincpu(*this, "maincpu")
		, m_acia(*this, "acia")
		, m_p_chargen(*this, "chargen")
	{
	}

	DECLARE_READ8_MEMBER(keyin_r);
	void kbd_put(u8 data);
	DECLARE_WRITE_LINE_MEMBER(write_acia_clock);
	MC6845_UPDATE_ROW(crtc_update_row);

private:
	uint8_t m_term_data;
	virtual void machine_reset() override;
	required_device<palette_device> m_palette;
	required_shared_ptr<uint8_t> m_p_videoram;
	required_device<z80_device> m_maincpu;
	required_device<acia6850_device> m_acia;
	required_region_ptr<u8> m_p_chargen;
};

static ADDRESS_MAP_START(mx2178_mem, AS_PROGRAM, 8, mx2178_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x1fff) AM_ROM AM_REGION("roms", 0)
	AM_RANGE(0x2000, 0x27ff) AM_RAM AM_SHARE("videoram")
	AM_RANGE(0x6000, 0x6fff) AM_RAM
	AM_RANGE(0xe000, 0xe7ff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START(mx2178_io, AS_IO, 8, mx2178_state)
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x00) AM_DEVWRITE("crtc", mc6845_device, address_w)
	AM_RANGE(0x01, 0x01) AM_DEVREADWRITE("crtc", mc6845_device, register_r, register_w)
	//AM_RANGE(0xa0, 0xa0) AM_DEVREADWRITE("acia", acia6850_device, status_r, control_w)
	//AM_RANGE(0xa1, 0xa1) AM_DEVREADWRITE("acia", acia6850_device, data_r, data_w)
	AM_RANGE(0xa0, 0xa1) AM_READ(keyin_r)
ADDRESS_MAP_END


/* Input ports */
static INPUT_PORTS_START( mx2178 )
INPUT_PORTS_END

READ8_MEMBER( mx2178_state::keyin_r )
{
	if (offset)
	{
		uint8_t ret = m_term_data;
		m_term_data = 0;
		return ret;
	}
	else
		return (m_term_data) ? 0x83 : 0x82;
}

void mx2178_state::kbd_put(u8 data)
{
	m_term_data = data;
	m_maincpu->set_input_line(0, HOLD_LINE);
}

MC6845_UPDATE_ROW( mx2178_state::crtc_update_row )
{
	const rgb_t *pens = m_palette->palette()->entry_list_raw();
	uint8_t chr,gfx;
	uint16_t mem,x;
	uint32_t *p = &bitmap.pix32(y);

	for (x = 0; x < x_count; x++)
	{
		mem = (ma + x) & 0x7ff;
		chr = m_p_videoram[mem];

		/* get pattern of pixels for that character scanline */
		gfx = m_p_chargen[(chr<<4) | ra] ^ ((x == cursor_x) ? 0xff : 0);

		/* Display a scanline of a character (8 pixels) */
		*p++ = pens[BIT(gfx, 7)];
		*p++ = pens[BIT(gfx, 6)];
		*p++ = pens[BIT(gfx, 5)];
		*p++ = pens[BIT(gfx, 4)];
		*p++ = pens[BIT(gfx, 3)];
		*p++ = pens[BIT(gfx, 2)];
		*p++ = pens[BIT(gfx, 1)];
		*p++ = pens[BIT(gfx, 0)];
	}
}

/* F4 Character Displayer */
static const gfx_layout mx2178_charlayout =
{
	8, 16,                  /* 8 x 16 characters */
	256,                    /* 256 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8, 12*8, 13*8 },
	8*16                    /* every char takes 8 bytes */
};

static GFXDECODE_START( mx2178 )
	GFXDECODE_ENTRY( "chargen", 0x0000, mx2178_charlayout, 0, 1 )
GFXDECODE_END

void mx2178_state::machine_reset()
{
}

WRITE_LINE_MEMBER(mx2178_state::write_acia_clock)
{
	m_acia->write_txc(state);
	m_acia->write_rxc(state);
}

static MACHINE_CONFIG_START( mx2178 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, 18869600/5) // guess
	MCFG_CPU_PROGRAM_MAP(mx2178_mem)
	MCFG_CPU_IO_MAP(mx2178_io)

	/* video hardware */
	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::green())
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not correct
	MCFG_SCREEN_UPDATE_DEVICE("crtc", mc6845_device, screen_update)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", mx2178)
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	/* Devices */
	MCFG_MC6845_ADD("crtc", MC6845, "screen", 18869600 / 8) // clk unknown
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(mx2178_state, crtc_update_row)

	MCFG_DEVICE_ADD("acia", ACIA6850, 0)

	/// TODO: hook up acia to keyboard and memory map

	MCFG_DEVICE_ADD("keyboard", GENERIC_KEYBOARD, 0)
	MCFG_GENERIC_KEYBOARD_CB(PUT(mx2178_state, kbd_put))

	MCFG_DEVICE_ADD("acia_clock", CLOCK, 614400)
	MCFG_CLOCK_SIGNAL_HANDLER(WRITELINE(mx2178_state, write_acia_clock))

MACHINE_CONFIG_END

/* ROM definition */
ROM_START( mx2178 )
	ROM_REGION(0x2000, "roms", 0)
	ROM_LOAD( "96274139.bin", 0x000000, 0x002000, CRC(eb471a27) SHA1(433abefd1a72653d0bf35bcaaeccf9943b96260b) )

	ROM_REGION(0x1000, "chargen", 0)
	ROM_LOAD( "96273883.bin", 0x000000, 0x001000, CRC(8311fadd) SHA1(573bbad23e893ad9374edc929642dc1cba3452d2) )
ROM_END

/* Driver */

//    YEAR  NAME    PARENT  COMPAT   MACHINE    INPUT   STATE         INIT  COMPANY    FULLNAME        FLAGS
COMP( 1984, mx2178, 0,      0,       mx2178,    mx2178, mx2178_state, 0,    "Memorex", "Memorex 2178", MACHINE_IS_SKELETON )
