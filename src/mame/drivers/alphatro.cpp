// license:BSD-3-Clause
// copyright-holders:Barry Rodewald, Robbbert, R. Belmont, Carl
/***************************************************************************

    Triumph-Adler (or Royal) Alphatronic PC

    Driver by Barry Rodewald, Robbbert, R. Belmont and Carl

    z80 + HD46505SP as a CRTC

    Other chips: 8251, 8257, 8259
    Crystals: 16MHz, 17.73447MHz
    Floppy format: 13cm, 2 sides, 40 tracks, 16 sectors, 256 bytes = 320kb.
    FDC (uPD765) is in a plug-in module, there is no ROM on the module.

    A configuration switch determines if the FDC is present.

    Has a socket for monochrome (to the standard amber monitor),
    and another for RGB. We emulate this with a configuration switch.

    ToDo:
    - Newer ROM set from Team Europe and try to work out the graphics expansion
    - uPD765 oddness that prevents Disk BASIC from loading

***************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "imagedev/cassette.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "machine/ram.h"
#include "machine/bankdev.h"
#include "machine/upd765.h"
#include "machine/i8257.h"
#include "sound/beep.h"
#include "sound/wave.h"
#include "video/mc6845.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#define MAIN_CLOCK XTAL_4MHz


class alphatro_state : public driver_device
{
public:
	enum
	{
		TIMER_SYSTEM
	};

	alphatro_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_ram(*this, RAM_TAG)
		, m_p_videoram(*this, "videoram")
		, m_screen(*this, "screen")
		, m_p_chargen(*this, "chargen")
		, m_maincpu(*this, "maincpu")
		, m_crtc(*this, "crtc")
		, m_usart(*this, "usart")
		, m_cass(*this, "cassette")
		, m_beep(*this, "beeper")
		, m_palette(*this, "palette")
		, m_lowbank(*this, "lowbank")
		, m_cartbank(*this, "cartbank")
		, m_monbank(*this, "monbank")
		, m_fdc(*this, "fdc")
		, m_dmac(*this, "dmac")
		, m_config(*this, "CONFIG")
	{ }

	DECLARE_READ8_MEMBER (ram0000_r);
	DECLARE_WRITE8_MEMBER(ram0000_w);
	DECLARE_READ8_MEMBER (ram6000_r);
	DECLARE_WRITE8_MEMBER(ram6000_w);
	DECLARE_READ8_MEMBER (rama000_r);
	DECLARE_WRITE8_MEMBER(rama000_w);
	DECLARE_READ8_MEMBER (rame000_r);
	DECLARE_WRITE8_MEMBER(rame000_w);
	DECLARE_READ8_MEMBER(port10_r);
	DECLARE_WRITE8_MEMBER(port10_w);
	DECLARE_WRITE8_MEMBER(port20_w);
	DECLARE_READ8_MEMBER(port30_r);
	DECLARE_WRITE8_MEMBER(port30_w);
	DECLARE_READ8_MEMBER(portf0_r);
	DECLARE_WRITE8_MEMBER(portf0_w);
	DECLARE_INPUT_CHANGED_MEMBER(alphatro_break);
	DECLARE_WRITE_LINE_MEMBER(txdata_callback);
	DECLARE_WRITE_LINE_MEMBER(write_usart_clock);
	DECLARE_WRITE_LINE_MEMBER(hrq_w);
	DECLARE_WRITE_LINE_MEMBER(fdc_irq_w);
	DECLARE_PALETTE_INIT(alphatro);
	TIMER_DEVICE_CALLBACK_MEMBER(timer_c);
	TIMER_DEVICE_CALLBACK_MEMBER(timer_p);
	MC6845_UPDATE_ROW(crtc_update_row);

private:
	uint8_t *m_ram_ptr;
	required_device<ram_device> m_ram;
	required_shared_ptr<u8> m_p_videoram;
	required_device<screen_device> m_screen;
	u8 m_flashcnt;
	u8 m_cass_data[4];
	u8 m_port_10, m_port_20, m_port_30, m_port_f0;
	bool m_cass_state;
	bool m_cassold, m_fdc_irq;
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	void update_banking();
	required_region_ptr<u8> m_p_chargen;
	required_device<cpu_device> m_maincpu;
	required_device<mc6845_device> m_crtc;
	required_device<i8251_device> m_usart;
	required_device<cassette_image_device> m_cass;
	required_device<beep_device> m_beep;
	required_device<palette_device> m_palette;
	required_device<address_map_bank_device> m_lowbank, m_cartbank, m_monbank;
	required_device<upd765a_device> m_fdc;
	required_device<i8257_device> m_dmac;
	required_ioport m_config;
};

void alphatro_state::update_banking()
{
	if (m_port_10 & 0x80)   // RAM at 0000?
	{
		m_lowbank->set_bank(1);
	}
	else
	{
		m_lowbank->set_bank(0);
	}

	if (m_port_10 & 0x40)   // ROM cartridge at A000
	{
		m_cartbank->set_bank(0);
	}
	else
	{
		m_cartbank->set_bank(1);
	}

	if (m_port_20 & 0x08)    // VRAM at F000?
	{
		m_monbank->set_bank(0);
	}
	else
	{
		if (m_port_20 & 0x40)   // IPL or Monitor at F000?
		{
			m_monbank->set_bank(2);
		}
		else
		{
			m_monbank->set_bank(1);
		}
	}
}

READ8_MEMBER (alphatro_state::ram0000_r)
{
	if (offset < 0xf000)
	{
		return m_ram_ptr[offset];
	}

	return m_p_videoram[offset & 0xfff];
}

WRITE8_MEMBER(alphatro_state::ram0000_w)
{
	if (offset < 0xf000)
	{
		m_ram_ptr[offset] = data;
	}
	else
	{
		m_p_videoram[offset & 0xfff] = data;
	}
}

READ8_MEMBER (alphatro_state::ram6000_r) { return m_ram_ptr[offset+0x6000]; }
WRITE8_MEMBER(alphatro_state::ram6000_w) { m_ram_ptr[offset+0x6000] = data; }
READ8_MEMBER (alphatro_state::rama000_r) { return m_ram_ptr[offset+0xa000]; }
WRITE8_MEMBER(alphatro_state::rama000_w) { m_ram_ptr[offset+0xa000] = data; }
READ8_MEMBER (alphatro_state::rame000_r) { return m_ram_ptr[offset+0xe000]; }
WRITE8_MEMBER(alphatro_state::rame000_w) { m_ram_ptr[offset+0xe000] = data; }

READ8_MEMBER( alphatro_state::port10_r )
{
// Bit 0 -> 1 = FDC is installed, 0 = not
// Bit 1 -> 1 = Graphic Board is installed, 0 = not
// Bits 2-4 = Country select: 0 = Intl, 1 = German, 2 = US
// Bit 5 -> 1 = BASIC LPRINT is RS-232, 0 = BASIC LPRINT is Centronics
// Bit 6 -> 1 = NTSC, 0 = PAL
// Bit 7 -> 1 = vblank or hblank, 0 = active display area

	u8 retval = 0x40;

	// we'll get "FDC present" and "graphics expansion present" from the config switches
	retval |= (m_config->read() & 3);

	if ((m_screen->vblank()) || (m_screen->hblank()))
	{
		retval |= 0x80;
	}

	return retval;
}

WRITE8_MEMBER( alphatro_state::port10_w )
{
// Bit 0 -> 0 = 40 cols; 1 = 80 cols
// Bit 1 -> 0 = display enable, 1 = display inhibit
// Bit 2 -> 0 = USART is connected to cassette, 1 = RS232 port
// Bit 3 -> 0 = cassette motor off, 1 = cassette motor on
// Bit 4 -> 0 = beeper off, 1 = beeper on
// Bit 5 -> always 0
// Bit 6 -> 1 = select ROM pack at A000, 0 = RAM at A000
// Bit 7 -> 0 = ROM enabled at 0, 1 = RAM enabled

	m_port_10 = data;

	data &= 0xfe;

	m_beep->set_state(BIT(data, 4));

	m_cass->change_state( BIT(data, 3) ? CASSETTE_MOTOR_ENABLED : CASSETTE_MOTOR_DISABLED, CASSETTE_MASK_MOTOR);

	if (BIT(data,2))
		m_cass_state = 1;

	update_banking();
}

WRITE8_MEMBER( alphatro_state::port20_w )
{
// Bit 0 -> 0 = CRTC reset release, 1 = CRTC reset enable
// Bit 1 -> 0 = Centronics reset release, 1 = Centronics reset enable
// Bit 2 -> 0 = no Centronics strobe, 1 = Centronics strobe active
// Bit 3 -> 0 = Monitor ROM at F000, 1 = VRAM at F000
// Bit 4 -> 0 = Graphic LED off, 1 = Graphic LED on
// Bit 5 -> 0 = Shift Lock LED off, 1 = Shift Lock LED on
// Bit 6 -> 0 = Lower 4K of Monitor at F000, 1 = Upper 4K of Monitor at F000
// Bit 7 -> N/A

	m_port_20 = data;

	update_banking();
}

READ8_MEMBER( alphatro_state::port30_r )
{
// Bit 0 -> SIOC
// Bit 1 -> 1 = vsync, 0 = not
// Bit 2 -> 1 = Centronics ACK, 0 = not
// Bit 3 -> 1 = Centronics BUSY, 0 = not

	u8 retval = 0;

	if (m_screen->vblank()) retval |= 0x02;

	return retval;
}

WRITE8_MEMBER( alphatro_state::port30_w )
{
	m_port_30 = data;
}

READ8_MEMBER( alphatro_state::portf0_r )
{
	return m_fdc_irq << 6;
}

WRITE8_MEMBER( alphatro_state::portf0_w)
{
	if ((data & 0x1) && !(m_port_f0))
	{
		m_fdc->reset();

		floppy_connector *con = machine().device<floppy_connector>("fdc:0");
		floppy_image_device *floppy = con ? con->get_device() : nullptr;
		if (floppy)
		{
			floppy->mon_w(0);
			m_fdc->set_rate(250000);
		}
		con = machine().device<floppy_connector>("fdc:1");
		floppy = con ? con->get_device() : nullptr;
		if (floppy)
		{
			floppy->mon_w(0);
		}
	}

	m_port_f0 = data;
}

void alphatro_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch(id)
	{
	default:
		assert_always(false, "Unknown id in alphatro_state::device_timer");
	}
}

WRITE_LINE_MEMBER( alphatro_state::txdata_callback )
{
	m_cass_state = state;
}

WRITE_LINE_MEMBER( alphatro_state::write_usart_clock )
{
	m_usart->write_txc(state);
	m_usart->write_rxc(state);
}

MC6845_UPDATE_ROW( alphatro_state::crtc_update_row )
{
	const rgb_t *pens = m_palette->palette()->entry_list_raw();
	bool palette = BIT(m_config->read(), 5);
	if (y==0) m_flashcnt++;
	bool inv;
	u8 chr,gfx,attr,bg,fg;
	u16 mem,x;
	u32 *p = &bitmap.pix32(y);

	for (x = 0; x < x_count; x++)
	{
		inv = (x == cursor_x);
		mem = (ma + x) & 0x7ff;
		chr = m_p_videoram[mem];
		attr = m_p_videoram[mem | 0x800];
		bg = (palette) ? 8 : attr & 7; // amber or RGB
		fg = (palette) ? 0 : (attr & 0x38) >> 3;

		if (BIT(attr, 7)) // reverse video
		{
			inv ^= 1;
			chr &= 0x7f;
		}

		if (BIT(attr, 6) & BIT(m_flashcnt, 4)) // flashing
		{
			inv ^= 1;
		}

		/* get pattern of pixels for that character scanline */
		gfx = m_p_chargen[(chr<<4) | ra];

		if (inv)
		{
			u8 t = bg;
			bg = fg;
			fg = t;
		}

		/* Display a scanline of a character (8 pixels) */
		*p++ = pens[BIT(gfx, 7) ? fg : bg];
		*p++ = pens[BIT(gfx, 6) ? fg : bg];
		*p++ = pens[BIT(gfx, 5) ? fg : bg];
		*p++ = pens[BIT(gfx, 4) ? fg : bg];
		*p++ = pens[BIT(gfx, 3) ? fg : bg];
		*p++ = pens[BIT(gfx, 2) ? fg : bg];
		*p++ = pens[BIT(gfx, 1) ? fg : bg];
		*p++ = pens[BIT(gfx, 0) ? fg : bg];
	}
}

INPUT_CHANGED_MEMBER( alphatro_state::alphatro_break )
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, HOLD_LINE);
}

static ADDRESS_MAP_START( alphatro_map, AS_PROGRAM, 8, alphatro_state )
	AM_RANGE(0x0000, 0x5fff) AM_DEVICE("lowbank", address_map_bank_device, amap8)
	AM_RANGE(0x6000, 0x9fff) AM_READWRITE(ram6000_r, ram6000_w)
	AM_RANGE(0xa000, 0xdfff) AM_DEVICE("cartbank", address_map_bank_device, amap8)
	AM_RANGE(0xe000, 0xefff) AM_READWRITE(rame000_r, rame000_w)
	AM_RANGE(0xf000, 0xffff) AM_DEVICE("monbank", address_map_bank_device, amap8)

ADDRESS_MAP_END

static ADDRESS_MAP_START( rombank_map, AS_PROGRAM, 8, alphatro_state )
	AM_RANGE(0x0000, 0x5fff) AM_ROM AM_REGION("roms", 0x0000) AM_WRITE(ram0000_w)
	AM_RANGE(0x6000, 0xbfff) AM_READWRITE(ram0000_r, ram0000_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( cartbank_map, AS_PROGRAM, 8, alphatro_state )
	AM_RANGE(0x0000, 0x3fff) AM_ROM AM_REGION("cart", 0x0000)
	AM_RANGE(0x4000, 0x7fff) AM_READWRITE(rama000_r, rama000_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( monbank_map, AS_PROGRAM, 8, alphatro_state )
	AM_RANGE(0x0000, 0x0fff) AM_RAM AM_SHARE("videoram")
	AM_RANGE(0x1000, 0x1fff) AM_ROM AM_REGION("roms", 0x8000)
	AM_RANGE(0x2000, 0x2fff) AM_ROM AM_REGION("roms", 0x9000)
ADDRESS_MAP_END

static ADDRESS_MAP_START( alphatro_io, AS_IO, 8, alphatro_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x10, 0x10) AM_READWRITE(port10_r, port10_w)
	AM_RANGE(0x20, 0x20) AM_READ_PORT("X0") AM_WRITE(port20_w)
	AM_RANGE(0x21, 0x21) AM_READ_PORT("X1")
	AM_RANGE(0x22, 0x22) AM_READ_PORT("X2")
	AM_RANGE(0x23, 0x23) AM_READ_PORT("X3")
	AM_RANGE(0x24, 0x24) AM_READ_PORT("X4")
	AM_RANGE(0x25, 0x25) AM_READ_PORT("X5")
	AM_RANGE(0x26, 0x26) AM_READ_PORT("X6")
	AM_RANGE(0x27, 0x27) AM_READ_PORT("X7")
	AM_RANGE(0x28, 0x28) AM_READ_PORT("X8")
	AM_RANGE(0x29, 0x29) AM_READ_PORT("X9")
	AM_RANGE(0x2a, 0x2a) AM_READ_PORT("XA")
	AM_RANGE(0x2b, 0x2b) AM_READ_PORT("XB")
	AM_RANGE(0x30, 0x30) AM_READWRITE(port30_r, port30_w)
	// USART for cassette reading and writing
	AM_RANGE(0x40, 0x40) AM_DEVREADWRITE("usart", i8251_device, data_r, data_w)
	AM_RANGE(0x41, 0x41) AM_DEVREADWRITE("usart", i8251_device, status_r, control_w)
	// CRTC - HD46505 / HD6845SP
	AM_RANGE(0x50, 0x50) AM_DEVWRITE("crtc", mc6845_device, address_w)
	AM_RANGE(0x51, 0x51) AM_DEVREADWRITE("crtc", mc6845_device, register_r, register_w)
	// 8257 DMAC
	AM_RANGE(0x60, 0x68) AM_DEVREADWRITE("dmac", i8257_device, read, write)
	// 8259 PIT
	//AM_RANGE(0x70, 0x72) AM_DEVREADWRITE("
	AM_RANGE(0xf0, 0xf0) AM_READ(portf0_r) AM_WRITE(portf0_w)
	AM_RANGE(0xf8, 0xf8) AM_DEVREADWRITE("fdc", upd765a_device, fifo_r, fifo_w)
	AM_RANGE(0xf9, 0xf9) AM_DEVREAD("fdc", upd765a_device, msr_r)
ADDRESS_MAP_END

static INPUT_PORTS_START( alphatro )
	PORT_START("X0")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 0") PORT_CODE(KEYCODE_0_PAD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 1") PORT_CODE(KEYCODE_1_PAD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD))
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 2") PORT_CODE(KEYCODE_2_PAD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD))
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 3") PORT_CODE(KEYCODE_3_PAD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD))
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 4") PORT_CODE(KEYCODE_4_PAD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 5") PORT_CODE(KEYCODE_5_PAD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD))
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 6") PORT_CODE(KEYCODE_6_PAD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD))
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 7") PORT_CODE(KEYCODE_7_PAD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD))

	PORT_START("X1")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 8") PORT_CODE(KEYCODE_8_PAD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad 9") PORT_CODE(KEYCODE_9_PAD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD))
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad .") PORT_CODE(KEYCODE_DEL_PAD)
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad - /") PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad =") PORT_CODE(KEYCODE_ASTERISK) // keypad equals
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Keypad + x") PORT_CODE(KEYCODE_PLUS_PAD)

	PORT_START("X2")
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("A") PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("B") PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("C") PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("D") PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("E") PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F") PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("G") PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G')

	PORT_START("X3")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("H") PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("I") PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("J") PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("K") PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("L") PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("M") PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("N") PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("O") PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O')

	PORT_START("X4")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("P") PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Q") PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("R") PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("S") PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("T") PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("U") PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("V") PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("W") PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W')

	PORT_START("X5")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("X") PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Y") PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Z") PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z')

	PORT_START("X6")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("0") PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR('_')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("1") PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("2") PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('\"')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("3") PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR(0xa3)
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("4") PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("5") PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("6") PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("7") PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('\'')

	PORT_START("X7")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("8") PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("9") PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("- =") PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('=')
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("^ ~") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('^') PORT_CHAR('~')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("@ `") PORT_CODE(KEYCODE_DEL) PORT_CHAR('@') PORT_CHAR('`')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("[ {") PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("] }") PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME(": *") PORT_CODE(KEYCODE_QUOTE) PORT_CHAR(':') PORT_CHAR('*')

	PORT_START("X8")
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Space") PORT_CODE(KEYCODE_SPACE) PORT_CHAR(32)
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("\\ |") PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME(", <") PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME(". >") PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("/ ?") PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("; +") PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR('+')

	PORT_START("X9")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN))
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(UP))
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT))
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP))
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Lock") PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)

	PORT_START("XA")
	PORT_BIT(0x02,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("INS DEL") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Tab") PORT_CODE(KEYCODE_TAB) PORT_CHAR(9)
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13)
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Graph") PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Clear Home") PORT_CODE(KEYCODE_HOME)
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("ESC") PORT_CODE(KEYCODE_ESC) PORT_CHAR(27)

	PORT_START("XB")
	PORT_BIT(0x04,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F6") PORT_CODE(KEYCODE_F6)
	PORT_BIT(0x08,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x10,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4)
	PORT_BIT(0x20,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x40,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x80,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)

	PORT_START("other")
	PORT_BIT(0x01,IP_ACTIVE_HIGH,IPT_KEYBOARD) PORT_NAME("Break") PORT_CODE(KEYCODE_ESC) PORT_CHANGED_MEMBER(DEVICE_SELF,alphatro_state,alphatro_break,0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x20, 0x00, "Monitor")
	PORT_CONFSETTING(    0x00, "RGB")
	PORT_CONFSETTING(    0x20, "Amber")

	PORT_CONFNAME(0x01, 0x00, "Floppy disk installed")
	PORT_CONFSETTING(0x00, "Not present")
	PORT_CONFSETTING(0x01, "Installed")
INPUT_PORTS_END

static const gfx_layout charlayout =
{
	8,16,
	RGN_FRAC(1,1),
	1,
	{ RGN_FRAC(0,1) },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },
	8*16
};

static GFXDECODE_START( alphatro )
	GFXDECODE_ENTRY( "chargen", 0, charlayout, 0, 4 )
GFXDECODE_END

void alphatro_state::machine_start()
{
	save_item(NAME(m_port_10));
	save_item(NAME(m_port_20));
	save_item(NAME(m_cass_data));
	save_item(NAME(m_cass_state));
	save_item(NAME(m_cassold));
}

void alphatro_state::machine_reset()
{
	m_ram_ptr = m_ram->pointer();
	m_port_10 = m_port_20 = 0;
	update_banking();

	m_cass_data[0] = m_cass_data[1] = m_cass_data[2] = m_cass_data[3] = 0;
	m_cass_state = 0;
	m_cassold = 0;
	m_fdc_irq = 0;
	m_usart->write_rxd(0);
	m_beep->set_state(0);
}

PALETTE_INIT_MEMBER(alphatro_state, alphatro)
{
	// RGB colours
	palette.set_pen_color(0, 0x00, 0x00, 0x00);
	palette.set_pen_color(1, 0x00, 0x00, 0xff);
	palette.set_pen_color(2, 0xff, 0x00, 0x00);
	palette.set_pen_color(3, 0xff, 0x00, 0xff);
	palette.set_pen_color(4, 0x00, 0xff, 0x00);
	palette.set_pen_color(5, 0x00, 0xff, 0xff);
	palette.set_pen_color(6, 0xff, 0xff, 0x00);
	palette.set_pen_color(7, 0xff, 0xff, 0xff);
	// Amber
	palette.set_pen_color(8, 0xf7, 0xaa, 0x00);
}


TIMER_DEVICE_CALLBACK_MEMBER(alphatro_state::timer_c)
{
	m_cass_data[3]++;

	if (m_cass_state != m_cassold)
	{
		m_cass_data[3] = 0;
		m_cassold = m_cass_state;
	}

	if (m_cass_state)
		m_cass->output(BIT(m_cass_data[3], 0) ? -1.0 : +1.0); // 2400Hz
	else
		m_cass->output(BIT(m_cass_data[3], 1) ? -1.0 : +1.0); // 1200Hz
}

TIMER_DEVICE_CALLBACK_MEMBER(alphatro_state::timer_p)
{
	/* cassette - turn 1200/2400Hz to a bit */
	m_cass_data[1]++;
	u8 cass_ws = (m_cass->input() > +0.03) ? 1 : 0;

	if (cass_ws != m_cass_data[0])
	{
		m_cass_data[0] = cass_ws;
		m_usart->write_rxd((m_cass_data[1] < 12) ? 1 : 0);
		m_cass_data[1] = 0;
	}
}

WRITE_LINE_MEMBER(alphatro_state::fdc_irq_w)
{
	m_fdc_irq = state ? false : true;
}

WRITE_LINE_MEMBER(alphatro_state::hrq_w)
{
	m_maincpu->set_input_line(INPUT_LINE_HALT, state);
	m_dmac->hlda_w(state);
}

static SLOT_INTERFACE_START( alphatro_floppies )
	SLOT_INTERFACE( "525dd", FLOPPY_525_DD )
SLOT_INTERFACE_END

static MACHINE_CONFIG_START( alphatro )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",Z80,MAIN_CLOCK)
	MCFG_CPU_PROGRAM_MAP(alphatro_map)
	MCFG_CPU_IO_MAP(alphatro_io)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not correct
	MCFG_SCREEN_UPDATE_DEVICE("crtc", mc6845_device, screen_update)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", alphatro)
	MCFG_PALETTE_ADD("palette", 9) // 8 colours + amber
	MCFG_PALETTE_INIT_OWNER(alphatro_state, alphatro)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("beeper", BEEP, 950) /* piezo-device needs to be measured */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)
	MCFG_SOUND_WAVE_ADD(WAVE_TAG, "cassette")
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)

	/* Devices */
	MCFG_UPD765A_ADD("fdc", true, true)
	MCFG_UPD765_INTRQ_CALLBACK(WRITELINE(alphatro_state, fdc_irq_w))
	MCFG_UPD765_DRQ_CALLBACK(DEVWRITELINE("dmac", i8257_device, dreq2_w))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", alphatro_floppies, "525dd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", alphatro_floppies, "525dd", floppy_image_device::default_floppy_formats)

	MCFG_DEVICE_ADD("dmac" , I8257, MAIN_CLOCK)
	MCFG_I8257_OUT_HRQ_CB(WRITELINE(alphatro_state, hrq_w))
	MCFG_I8257_IN_MEMR_CB(READ8(alphatro_state, ram0000_r))
	MCFG_I8257_OUT_MEMW_CB(WRITE8(alphatro_state, ram0000_w))
	MCFG_I8257_IN_IOR_2_CB(DEVREAD8("fdc", upd765a_device, mdma_r))
	MCFG_I8257_OUT_IOW_2_CB(DEVWRITE8("fdc", upd765a_device, mdma_w))
	MCFG_I8257_OUT_TC_CB(DEVWRITELINE("fdc", upd765a_device, tc_line_w))

	MCFG_MC6845_ADD("crtc", MC6845, "screen", XTAL_12_288MHz / 8) // clk unknown
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(alphatro_state, crtc_update_row)

	MCFG_DEVICE_ADD("usart", I8251, 0)
	MCFG_I8251_TXD_HANDLER(WRITELINE(alphatro_state, txdata_callback))

	MCFG_DEVICE_ADD("usart_clock", CLOCK, 19218) // 19218 to load a real tape, 19222 to load a tape made by this driver
	MCFG_CLOCK_SIGNAL_HANDLER(WRITELINE(alphatro_state, write_usart_clock))

	MCFG_CASSETTE_ADD("cassette")
	MCFG_CASSETTE_DEFAULT_STATE(CASSETTE_PLAY | CASSETTE_MOTOR_ENABLED | CASSETTE_SPEAKER_ENABLED)
	MCFG_CASSETTE_INTERFACE("alphatro_cass")

	MCFG_TIMER_DRIVER_ADD_PERIODIC("timer_c", alphatro_state, timer_c, attotime::from_hz(4800))
	MCFG_TIMER_DRIVER_ADD_PERIODIC("timer_p", alphatro_state, timer_p, attotime::from_hz(40000))

	MCFG_RAM_ADD("ram")
	MCFG_RAM_DEFAULT_SIZE("64K")

	/* 0000 banking */
	MCFG_DEVICE_ADD("lowbank", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(rombank_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_BIG)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x6000)

	/* A000 banking */
	MCFG_DEVICE_ADD("cartbank", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(cartbank_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_BIG)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x4000)

	/* F000 banking */
	MCFG_DEVICE_ADD("monbank", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(monbank_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_BIG)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x1000)

	// software list
	MCFG_SOFTWARE_LIST_ADD("flop_list", "alphatro_flop")
MACHINE_CONFIG_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( alphatro )
	ROM_REGION( 0xa000, "roms", ROMREGION_ERASE00)
	ROM_LOAD( "613256.ic-1058", 0x0000, 0x6000, CRC(ceea4cb3) SHA1(b332dea0a2d3bb2978b8422eb0723960388bb467) )
	ROM_LOAD( "2764.ic-1038",   0x8000, 0x2000, CRC(e337db3b) SHA1(6010bade6a21975636383179903b58a4ca415e49) )

	ROM_REGION( 0x4000, "cart", ROMREGION_ERASE00)

	ROM_REGION( 0x1000, "chargen", 0 )
	ROM_LOAD( "2732.ic-1067",   0x0000, 0x1000, CRC(61f38814) SHA1(35ba31c58a10d5bd1bdb202717792ca021dbe1a8) )
ROM_END

COMP( 1983, alphatro,   0,       0,    alphatro,   alphatro, alphatro_state,  0,  "Triumph-Adler", "Alphatronic PC", MACHINE_SUPPORTS_SAVE )
