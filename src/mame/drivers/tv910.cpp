// license:BSD-3-Clause
// copyright-holders: R. Belmont
/***************************************************************************

    TeleVideo TV-910 / 910 Plus
    Preliminary driver by R. Belmont

    Hardware:
    6502 CPU
    6545 CRTC
    6551 ACIA

    IRQ = ACIA wire-OR CRTC VBlank
    NMI = AY-5-3600 keyboard char present

    TODO:
        - Character attributes: how is that even possible?  (Esc-V brings up test screen)
        - DIP switches don't all appear to have the expected effects
        - Keyboard hookup isn't quite right

****************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "cpu/m6502/m6502.h"
#include "machine/kb3600.h"
#include "machine/mos6551.h"
#include "video/mc6845.h"
#include "screen.h"

#define ACIA_TAG    "acia1"
#define CRTC_TAG    "crtc"
#define RS232_TAG   "rs232"
#define KBDC_TAG    "ay3600"

#define MASTER_CLOCK (13608000)

class tv910_state : public driver_device
{
public:
	tv910_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_crtc(*this, CRTC_TAG)
		, m_vram(*this, "vram")
		, m_ay3600(*this, KBDC_TAG)
		, m_kbdrom(*this, "keyboard")
		, m_kbspecial(*this, "keyb_special")
	{ }

	virtual void machine_start() override;
	virtual void machine_reset() override;

	MC6845_UPDATE_ROW(crtc_update_row);
	MC6845_ON_UPDATE_ADDR_CHANGED(crtc_update_addr);

	DECLARE_READ8_MEMBER(charset_r);
	DECLARE_READ8_MEMBER(kbd_ascii_r);
	DECLARE_READ8_MEMBER(kbd_flags_r);

	DECLARE_WRITE8_MEMBER(vbl_ack_w);
	DECLARE_WRITE8_MEMBER(nmi_ack_w);
	DECLARE_WRITE8_MEMBER(control_w);

	DECLARE_WRITE_LINE_MEMBER(vbl_w);
	DECLARE_WRITE_LINE_MEMBER(acia_irq_w);

	DECLARE_READ_LINE_MEMBER(ay3600_shift_r);
	DECLARE_READ_LINE_MEMBER(ay3600_control_r);
	DECLARE_WRITE_LINE_MEMBER(ay3600_data_ready_w);
	DECLARE_WRITE_LINE_MEMBER(ay3600_ako_w);

	required_device<m6502_device> m_maincpu;
	required_device<r6545_1_device> m_crtc;
	required_shared_ptr<uint8_t> m_vram;
	required_device<ay3600_device> m_ay3600;
	required_memory_region m_kbdrom;
	required_ioport m_kbspecial;

private:
	uint8_t *m_vramptr, *m_chrrom;

	uint16_t m_lastchar, m_strobe;
	uint8_t m_transchar;
	bool m_anykeydown;
	int m_repeatdelay;

	uint8_t m_control;

	bool m_vbl_irq, m_aica_irq;
};

static ADDRESS_MAP_START(tv910_mem, AS_PROGRAM, 8, tv910_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x03ff) AM_RAM
	AM_RANGE(0x4000, 0x47ff) AM_RAM AM_SHARE("vram") // VRAM
	AM_RANGE(0x8010, 0x801f) AM_READ(charset_r)
	AM_RANGE(0x8020, 0x8020) AM_DEVREADWRITE(CRTC_TAG, r6545_1_device, status_r, address_w)
	AM_RANGE(0x8021, 0x8021) AM_DEVREADWRITE(CRTC_TAG, r6545_1_device, register_r, register_w)
	AM_RANGE(0x8030, 0x8033) AM_DEVREADWRITE(ACIA_TAG, mos6551_device, read, write)
	AM_RANGE(0x8040, 0x804f) AM_WRITE(vbl_ack_w)
	AM_RANGE(0x8050, 0x805f) AM_WRITE(nmi_ack_w)
	AM_RANGE(0x8060, 0x806f) AM_READ(kbd_ascii_r)
	AM_RANGE(0x8070, 0x807f) AM_READ(kbd_flags_r)
	AM_RANGE(0x9000, 0x9000) AM_WRITE(control_w)
	AM_RANGE(0x9001, 0x9001) AM_READ_PORT("DSW1")   // S2 in the operator's manual
	AM_RANGE(0x9002, 0x9002) AM_READ_PORT("DSW2")   // S1 in the operator's manual
	AM_RANGE(0xf000, 0xffff) AM_ROM AM_REGION("maincpu", 0)
ADDRESS_MAP_END

WRITE8_MEMBER(tv910_state::control_w)
{
	m_control = data;
	#if 0
	printf("%02x to control (%c%c%c%c%c)\n",
		data,
		(data & 0x10) ? 'U' : 'B',
		(data & 0x8) ? '6' : '5',
		(data & 0x4) ? 'X' : ' ',
		(data & 0x2) ? 'C' : 'c',
		(data & 1) ? 'B' : ' ');
	#endif
}

READ8_MEMBER(tv910_state::charset_r)
{
	return 0;   // 0 = US (TODO: make this configurable)
}

WRITE8_MEMBER(tv910_state::nmi_ack_w)
{
	m_maincpu->set_input_line(M6502_NMI_LINE, CLEAR_LINE);
	m_strobe = 0;
}

READ8_MEMBER(tv910_state::kbd_ascii_r)
{
	return m_transchar;
}

READ8_MEMBER(tv910_state::kbd_flags_r)
{
	uint8_t rv = 0;

	//rv |= m_strobe ? 0x40 : 0;
	//rv |= (m_kbspecial->read() & 0x01) ? 0x00 : 0x40; // caps lock
	rv |= 0x40; // must be set for keyboard reads to work, but disagrees with docs?

	return rv;
}

READ_LINE_MEMBER(tv910_state::ay3600_shift_r)
{
	// either shift key
	if (m_kbspecial->read() & 0x06)
	{
		return ASSERT_LINE;
	}

	return CLEAR_LINE;
}

READ_LINE_MEMBER(tv910_state::ay3600_control_r)
{
	if (m_kbspecial->read() & 0x08)
	{
		return ASSERT_LINE;
	}

	return CLEAR_LINE;
}

WRITE_LINE_MEMBER(tv910_state::ay3600_data_ready_w)
{
	if (state == ASSERT_LINE)
	{
		uint8_t *decode = m_kbdrom->base();

		m_lastchar = m_ay3600->b_r();
		m_transchar = decode[m_lastchar];
		m_strobe = 1;

		m_maincpu->set_input_line(M6502_NMI_LINE, ASSERT_LINE);
		//printf("new char = %04x (%02x)\n", m_lastchar, m_transchar);
	}
}

WRITE_LINE_MEMBER(tv910_state::ay3600_ako_w)
{
	m_anykeydown = (state == ASSERT_LINE) ? true : false;

	if (m_anykeydown)
	{
		m_repeatdelay = 10;
	}
}

/* Input ports */

/*
Keyboard matrix (thanks to Al Kossow!)

   X0     X1     X2     X3     X4     X5     X6     X7
Y8                             BKTAB  FN19   FN18   FN17
Y7  B     3      E      F      [      RET    `      {
Y6  .     7      U      K      BRK    SPACE  BS     HOME
Y5  V     2      W      D      P      ENTER  0      DEL
Y4  ,     6      Y      J      '      /      9      DOWN
Y3  C     1      Q      S      O      ;      =      CLRSP
Y2  M     5      T      H             LEFT   8      UP
Y1  X     ESC    TAB    A      I      L      -      Z
Y0  N     4      R      G      LF     PRNT   \      RIGHT
*/

static INPUT_PORTS_START( tv910 )
	PORT_START("X0")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)  PORT_CHAR('N') PORT_CHAR('n')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)  PORT_CHAR('X') PORT_CHAR('x')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)  PORT_CHAR('M') PORT_CHAR('m')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)  PORT_CHAR('C') PORT_CHAR('c')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)  PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)  PORT_CHAR('V') PORT_CHAR('v')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)   PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)  PORT_CHAR('B') PORT_CHAR('b')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("X1")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4)  PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Esc")      PORT_CODE(KEYCODE_ESC) PORT_CHAR(27)
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5)  PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)  PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6)  PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)  PORT_CHAR('2') PORT_CHAR('\"')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7)  PORT_CHAR('7') PORT_CHAR('\'')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3)  PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("X2")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)  PORT_CHAR('R') PORT_CHAR('r')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tab")      PORT_CODE(KEYCODE_TAB)      PORT_CHAR(9)
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)  PORT_CHAR('T') PORT_CHAR('t')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)  PORT_CHAR('Q') PORT_CHAR('q')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)  PORT_CHAR('Y') PORT_CHAR('y')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)  PORT_CHAR('W') PORT_CHAR('w')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)  PORT_CHAR('U') PORT_CHAR('u')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)  PORT_CHAR('E') PORT_CHAR('e')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("X3")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)  PORT_CHAR('G') PORT_CHAR('g')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)          PORT_CHAR('A') PORT_CHAR('a')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)  PORT_CHAR('H') PORT_CHAR('h')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)  PORT_CHAR('S') PORT_CHAR('s')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)  PORT_CHAR('J') PORT_CHAR('j')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)  PORT_CHAR('D') PORT_CHAR('d')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)  PORT_CHAR('K') PORT_CHAR('k')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)  PORT_CHAR('F') PORT_CHAR('f')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("X4")
	/// 001 - LF
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)  PORT_CHAR('I') PORT_CHAR('i')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)  PORT_CHAR('O') PORT_CHAR('o')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)  PORT_CHAR('\'') PORT_CHAR('\"')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)  PORT_CHAR('P') PORT_CHAR('p')
	/// 040 - BRK
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('[')
	/// 100 - BACKTAB

	PORT_START("X5")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PRTSCR) PORT_CHAR(UCHAR_MAMEKEY(PRTSCR))
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)  PORT_CHAR('L') PORT_CHAR('l')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_LEFT)      PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)      PORT_CHAR(';') PORT_CHAR(':')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)  PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER_PAD)   PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD))
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)  PORT_CHAR(' ')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Return")   PORT_CODE(KEYCODE_ENTER)    PORT_CHAR(13)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_F19) PORT_CHAR(UCHAR_MAMEKEY(F19))

	PORT_START("X6")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH)  PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)  PORT_CHAR('-') PORT_CHAR('_')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)  PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('=') PORT_CHAR('+')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)  PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)      PORT_CHAR('0') PORT_CHAR(')')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Backspace")   PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE)      PORT_CHAR('`') PORT_CHAR('~')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_F18) PORT_CHAR(UCHAR_MAMEKEY(F18))

	PORT_START("X7")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_RIGHT)     PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)  PORT_CHAR('Z') PORT_CHAR('z')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_UP)        PORT_CODE(KEYCODE_UP)
/// 008 - CLRSP
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_DOWN)      PORT_CODE(KEYCODE_DOWN)     PORT_CHAR(10)      // E0 47
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('{')

	PORT_START("X8")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("keyb_special")
	PORT_BIT( 0x01, IP_ACTIVE_LOW,  IPT_KEYBOARD) PORT_NAME("Caps Lock")    PORT_CODE(KEYCODE_CAPSLOCK) PORT_TOGGLE
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left Shift")   PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right Shift")  PORT_CODE(KEYCODE_RSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Control")      PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)


	PORT_START("DSW2")  // "S1" in the Operator's Manual
	PORT_DIPNAME( 0x0f, 0x00, "Baud rate" )
	PORT_DIPSETTING(    0x0, "9600" )
	PORT_DIPSETTING(    0x1, "50" )
	PORT_DIPSETTING(    0x2, "75" )
	PORT_DIPSETTING(    0x3, "110" )
	PORT_DIPSETTING(    0x4, "135" )
	PORT_DIPSETTING(    0x5, "150" )
	PORT_DIPSETTING(    0x6, "300" )
	PORT_DIPSETTING(    0x7, "600" )
	PORT_DIPSETTING(    0x8, "1200" )
	PORT_DIPSETTING(    0x9, "1800" )
	PORT_DIPSETTING(    0xa, "2400" )
	PORT_DIPSETTING(    0xb, "3600" )
	PORT_DIPSETTING(    0xc, "4800" )
	PORT_DIPSETTING(    0xd, "7200" )
	PORT_DIPSETTING(    0xe, "9600" )
	PORT_DIPSETTING(    0xf, "19200" )

	PORT_DIPNAME( 0x10, 0x00, "Data bits" )
	PORT_DIPSETTING( 0x00, "8 bits" )
	PORT_DIPSETTING( 0x10, "7 bits" )

	PORT_DIPNAME( 0x20, 0x00, "Parity" )
	PORT_DIPSETTING( 0x00, "No Parity" )
	PORT_DIPSETTING( 0x20, "Send Parity" )

	PORT_DIPNAME( 0x40, 0x00, "Parity Type" )
	PORT_DIPSETTING( 0x00, "Odd Parity" )
	PORT_DIPSETTING( 0x40, "Even Parity" )

	PORT_DIPNAME( 0x80, 0x00, "Stop Bits" )
	PORT_DIPSETTING( 0x00, "1 stop bit" )
	PORT_DIPSETTING( 0x80, "2 stop bits" )

	PORT_START("DSW1")  // "S2" in the Operator's Manual
	PORT_DIPNAME( 0x03, 0x00, "Term Char")
	PORT_DIPSETTING(    0x0, "0" )
	PORT_DIPSETTING(    0x1, "1" )
	PORT_DIPSETTING(    0x2, "2" )
	PORT_DIPSETTING(    0x3, "3" )

	PORT_DIPNAME( 0x0c, 0x00, "Emulation" )
	PORT_DIPSETTING(    0x0, "Standard 910" )
	PORT_DIPSETTING(    0x4, "ADM-3A/5" )
	PORT_DIPSETTING(    0x8, "ADDS 25" )
	PORT_DIPSETTING(    0xc, "Hazeltine 1410" )

	PORT_DIPNAME( 0x10, 0x00, "Refresh rate" )
	PORT_DIPSETTING( 0x00, "60 Hz" )
	PORT_DIPSETTING( 0x10, "50 Hz" )

	PORT_DIPNAME( 0x60, 0x00, "Cursor type" )
	PORT_DIPSETTING(    0x00, "Blinking block" )
	PORT_DIPSETTING(    0x20, "Blinking underline" )
	PORT_DIPSETTING(    0x40, "Steady block" )
	PORT_DIPSETTING(    0x60, "Steady underline" )

	PORT_DIPNAME( 0x80, 0x00, "Duplex" )
	PORT_DIPSETTING( 0x00, "Half Duplex" )
	PORT_DIPSETTING( 0x80, "Full Duplex" )
#if 0
	PORT_DIPNAME( 0x100, 0x000, "Colors" )
	PORT_DIPSETTING( 0x00, "Black characters on white screen" )
	PORT_DIPSETTING( 0x100, "White characters on black screen" )

	PORT_DIPNAME( 0x200, 0x200, "Data Set Ready" )
	PORT_DIPSETTING( 0x00, "DSR connected" )
	PORT_DIPSETTING( 0x200, "DSR disconnected" )
#endif
INPUT_PORTS_END

void tv910_state::machine_start()
{
	m_vramptr = m_vram.target();
	m_chrrom = memregion("graphics")->base();
}

void tv910_state::machine_reset()
{
}

MC6845_ON_UPDATE_ADDR_CHANGED( tv910_state::crtc_update_addr )
{
}

WRITE_LINE_MEMBER(tv910_state::vbl_w)
{
	// this is ACKed by vbl_ack_w, state going 0 here doesn't ack the IRQ
	if (state)
	{
		m_vbl_irq = true;
		m_maincpu->set_input_line(M6502_IRQ_LINE, ASSERT_LINE);
	}
}

WRITE_LINE_MEMBER(tv910_state::acia_irq_w)
{
	m_aica_irq = (state == ASSERT_LINE) ? true : false;

	if (m_aica_irq || m_vbl_irq)
	{
		m_maincpu->set_input_line(M6502_IRQ_LINE, ASSERT_LINE);
	}
	else
	{
		m_maincpu->set_input_line(M6502_IRQ_LINE, CLEAR_LINE);
	}
}

WRITE8_MEMBER(tv910_state::vbl_ack_w)
{
	m_vbl_irq = false;

	if (m_aica_irq || m_vbl_irq)
	{
		m_maincpu->set_input_line(M6502_IRQ_LINE, ASSERT_LINE);
	}
	else
	{
		m_maincpu->set_input_line(M6502_IRQ_LINE, CLEAR_LINE);
	}
}

MC6845_UPDATE_ROW( tv910_state::crtc_update_row )
{
	static const uint32_t palette[2] = { 0, 0x00ff00 };
	uint32_t  *p = &bitmap.pix32(y);
	uint16_t  chr_base = ra;
	int i;

	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ma + i ) & 0x7ff;
		uint8_t chr = m_vramptr[ offset ];
		uint8_t data = m_chrrom[ chr_base + chr * 8 ];
		uint8_t fg = 1;
		uint8_t bg = 0;

		if ( i == cursor_x )
		{
			if (m_control & 2)
			{
				data = 0xFF;
			}
		}

		if ((y % 10) >= 8)
		{
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
			*p++ = palette[0];
		}
		else
		{
			*p = palette[( data & 0x80 ) ? fg : bg]; p++;
			*p = palette[( data & 0x40 ) ? fg : bg]; p++;
			*p = palette[( data & 0x20 ) ? fg : bg]; p++;
			*p = palette[( data & 0x10 ) ? fg : bg]; p++;
			*p = palette[( data & 0x08 ) ? fg : bg]; p++;
			*p = palette[( data & 0x04 ) ? fg : bg]; p++;
			*p = palette[( data & 0x02 ) ? fg : bg]; p++;
			*p = palette[( data & 0x01 ) ? fg : bg]; p++;
		}
	}
}

static MACHINE_CONFIG_START( tv910 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, MASTER_CLOCK/8)
	MCFG_CPU_PROGRAM_MAP(tv910_mem)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(MASTER_CLOCK, 882, 0, 720, 370, 0, 350 ) // not real values
	MCFG_SCREEN_UPDATE_DEVICE( CRTC_TAG, r6545_1_device, screen_update )

	MCFG_MC6845_ADD(CRTC_TAG, R6545_1, "screen", MASTER_CLOCK/8)
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(tv910_state, crtc_update_row)
	MCFG_MC6845_ADDR_CHANGED_CB(tv910_state, crtc_update_addr)
	MCFG_MC6845_OUT_VSYNC_CB(WRITELINE(tv910_state, vbl_w))

	MCFG_DEVICE_ADD(KBDC_TAG, AY3600, 0)
	MCFG_AY3600_MATRIX_X0(IOPORT("X0"))
	MCFG_AY3600_MATRIX_X1(IOPORT("X1"))
	MCFG_AY3600_MATRIX_X2(IOPORT("X2"))
	MCFG_AY3600_MATRIX_X3(IOPORT("X3"))
	MCFG_AY3600_MATRIX_X4(IOPORT("X4"))
	MCFG_AY3600_MATRIX_X5(IOPORT("X5"))
	MCFG_AY3600_MATRIX_X6(IOPORT("X6"))
	MCFG_AY3600_MATRIX_X7(IOPORT("X7"))
	MCFG_AY3600_MATRIX_X8(IOPORT("X8"))
	MCFG_AY3600_SHIFT_CB(READLINE(tv910_state, ay3600_shift_r))
	MCFG_AY3600_CONTROL_CB(READLINE(tv910_state, ay3600_control_r))
	MCFG_AY3600_DATA_READY_CB(WRITELINE(tv910_state, ay3600_data_ready_w))
	MCFG_AY3600_AKO_CB(WRITELINE(tv910_state, ay3600_ako_w))

	MCFG_DEVICE_ADD(ACIA_TAG, MOS6551, 0)
	MCFG_MOS6551_XTAL(XTAL_1_8432MHz)
	MCFG_MOS6551_IRQ_HANDLER(WRITELINE(tv910_state, acia_irq_w))
	MCFG_MOS6551_TXD_HANDLER(DEVWRITELINE(RS232_TAG, rs232_port_device, write_txd))
	MCFG_MOS6551_RTS_HANDLER(DEVWRITELINE(RS232_TAG, rs232_port_device, write_rts))
	MCFG_MOS6551_DTR_HANDLER(DEVWRITELINE(RS232_TAG, rs232_port_device, write_dtr))

	MCFG_RS232_PORT_ADD(RS232_TAG, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE(ACIA_TAG, mos6551_device, write_rxd))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE(ACIA_TAG, mos6551_device, write_dcd))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE(ACIA_TAG, mos6551_device, write_dsr))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE(ACIA_TAG, mos6551_device, write_cts))

MACHINE_CONFIG_END

/* ROM definition */
ROM_START( tv910 )
	ROM_REGION(0x1000, "maincpu", 0)
	ROM_LOAD( "1800000-020e_a38_9182.bin", 0x000000, 0x001000, CRC(ae71dd7f) SHA1(a12da9329e28a4a8e3c902f795059251311d2856) )

	ROM_REGION(0x2000, "graphics", 0)
	ROM_LOAD( "1800000-016a_a17_85ae.bin", 0x000000, 0x001000, CRC(835445b7) SHA1(dde94fb6531dadce48e19bf551f45f61bedf905b) )

	ROM_REGION(0x1000, "keyboard", 0)
	ROM_LOAD( "1800000-019b_bell_a2_43d6.bin", 0x000000, 0x000800, CRC(de954a77) SHA1(c4f7c19799c15d12d89f08dc31064fc6be9befb0) )
ROM_END

/* Driver */
//    YEAR  NAME    PARENT  COMPAT   MACHINE    INPUT  STATE         INIT  COMPANY      FULLNAME  FLAGS
COMP( 1981, tv910,  0,      0,       tv910,     tv910, tv910_state,  0,    "TeleVideo", "TV910",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
