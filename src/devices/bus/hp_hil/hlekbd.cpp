// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
#include "hlekbd.h"

#include "machine/keyboard.ipp"


/***************************************************************************
    DEVICE TYPE GLOBALS
***************************************************************************/

DEFINE_DEVICE_TYPE_NS(HP_IPC_HLE_KEYBOARD, bus::hp_hil, hle_hp_ipc_device, "hp_ipc_hle_kbd", "HP Integral Keyboard (HLE)")


namespace bus { namespace hp_hil {

namespace {

/***************************************************************************
    INPUT PORT DEFINITIONS
***************************************************************************/


// ID codes: A0h..BFh (HP-HIL reference, p. B-4) + (IPC Service Manual, p. 10-2)
INPUT_PORTS_START( id )
	PORT_START("COL0")
	PORT_DIPNAME( 0xff, 0xb7, "Layout" )
	PORT_DIPSETTING( 0xBF, "US" )
	PORT_DIPSETTING( 0xAF, "German" )
	PORT_DIPSETTING( 0xB7, "UK" )
	PORT_DIPSETTING( 0xBB, "French" )
	PORT_DIPSETTING( 0xBD, "Katakana" )
	PORT_DIPSETTING( 0xBE, "Latin Spanish" )
	PORT_DIPSETTING( 0xA7, "Canadian English" )
	PORT_DIPSETTING( 0xAB, "Italian" )
	PORT_DIPSETTING( 0xAD, "Dutch" )
	PORT_DIPSETTING( 0xAE, "Swedish" )
	PORT_DIPSETTING( 0xB3, "European Spanish" )
	PORT_DIPSETTING( 0xB5, "Belgian (Flemish)" )
	PORT_DIPSETTING( 0xB6, "Finnish" )
	PORT_DIPSETTING( 0xB4, "Swiss German" )
	PORT_DIPSETTING( 0xBA, "Norwegian" )
	PORT_DIPSETTING( 0xBC, "Danish" )
	PORT_DIPSETTING( 0xB2, "Swiss French" )
INPUT_PORTS_END

INPUT_PORTS_START( basic )
	// keycodes 90..9f
	PORT_START("COL1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Menu")         PORT_CODE(KEYCODE_F9)         PORT_CHAR(UCHAR_MAMEKEY(F9))
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F4")           PORT_CODE(KEYCODE_F4)         PORT_CHAR(UCHAR_MAMEKEY(F4))
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F3")           PORT_CODE(KEYCODE_F3)         PORT_CHAR(UCHAR_MAMEKEY(F3))
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F2")           PORT_CODE(KEYCODE_F2)         PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F1")           PORT_CODE(KEYCODE_F1)         PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 8")         PORT_CODE(KEYCODE_8_PAD)      PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Stop")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Print/Enter")

	// keycodes a0..af
	PORT_START("COL2")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("User/System")  PORT_CODE(KEYCODE_F10)        PORT_CHAR(UCHAR_MAMEKEY(F10))
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F5")           PORT_CODE(KEYCODE_F5)         PORT_CHAR(UCHAR_MAMEKEY(F5))
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F6")           PORT_CODE(KEYCODE_F6)         PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F7")           PORT_CODE(KEYCODE_F7)         PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F8")           PORT_CODE(KEYCODE_F8)         PORT_CHAR(UCHAR_MAMEKEY(F8))
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 9")         PORT_CODE(KEYCODE_9_PAD)      PORT_CHAR(UCHAR_MAMEKEY(9_PAD))
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Clear Line")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Clear Display")

	// keycodes b0..bf
	PORT_START("COL3")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_8)          PORT_CHAR('8') PORT_CHAR('*')
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_9)          PORT_CHAR('9') PORT_CHAR('(')
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_0)          PORT_CHAR('0') PORT_CHAR(')')
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('=') PORT_CHAR('+')
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_SLASH)      PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Backspace")    PORT_CODE(KEYCODE_BACKSPACE)  PORT_CHAR(8)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes c0..cf
	PORT_START("COL4")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_I)          PORT_CHAR('i') PORT_CHAR('I') PORT_CHAR(0x09) PORT_CHAR(0x09)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_O)          PORT_CHAR('o') PORT_CHAR('O') PORT_CHAR(0x0f) PORT_CHAR(0x0f)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_P)          PORT_CHAR('p') PORT_CHAR('P') PORT_CHAR(0x10) PORT_CHAR(0x10)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("< >")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes d0..df
	PORT_START("COL5")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_J)          PORT_CHAR('j') PORT_CHAR('J') PORT_CHAR(0x0a) PORT_CHAR(0x0a)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_K)          PORT_CHAR('k') PORT_CHAR('K') PORT_CHAR(0x0b) PORT_CHAR(0x0b)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_L)          PORT_CHAR('l') PORT_CHAR('L') PORT_CHAR(0x0c) PORT_CHAR(0x0c)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_QUOTE)      PORT_CHAR('\'') PORT_CHAR('"')
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_BACKSLASH)  PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Return")       PORT_CODE(KEYCODE_ENTER)      PORT_CHAR(13)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Home")         PORT_CODE(KEYCODE_HOME)       PORT_CHAR(UCHAR_MAMEKEY(HOME))
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes e0..ef
	PORT_START("COL6")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_M)          PORT_CHAR('m') PORT_CHAR('M') PORT_CHAR(0x0d) PORT_CHAR(0x0d)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_COMMA)      PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_STOP)       PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-') PORT_CHAR('_')
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Select")       PORT_CODE(KEYCODE_PLUS_PAD)   PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD))
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes f0..ff
	PORT_START("COL7")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_N)          PORT_CHAR('n') PORT_CHAR('N') PORT_CHAR(0x0e) PORT_CHAR(0x0e)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Space")        PORT_CODE(KEYCODE_SPACE)      PORT_CHAR(' ')
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP .")         PORT_CODE(KEYCODE_DEL_PAD)    PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Left")         PORT_CODE(KEYCODE_LEFT)       PORT_CHAR(UCHAR_MAMEKEY(LEFT))
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Down")         PORT_CODE(KEYCODE_DOWN)       PORT_CHAR(UCHAR_MAMEKEY(DOWN))
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Up")           PORT_CODE(KEYCODE_UP)         PORT_CHAR(UCHAR_MAMEKEY(UP))
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Right")        PORT_CODE(KEYCODE_RIGHT)      PORT_CHAR(UCHAR_MAMEKEY(RIGHT))

	// keycodes 00..0f
	PORT_START("COL8")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 5")         PORT_CODE(KEYCODE_5_PAD)      PORT_CHAR(UCHAR_MAMEKEY(5_PAD))
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Extend Char R")PORT_CODE(KEYCODE_RALT)       PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Extend Char L")PORT_CODE(KEYCODE_LALT)       PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("R Shift")      PORT_CODE(KEYCODE_RSHIFT)     PORT_CHAR(UCHAR_MAMEKEY(RSHIFT))
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("L Shift")      PORT_CODE(KEYCODE_LSHIFT)     PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Control")      PORT_CODE(KEYCODE_LCONTROL)   PORT_CHAR(UCHAR_MAMEKEY(LCONTROL))
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Break/Reset")

	// keycodes 10..1f
	PORT_START("COL9")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 4")         PORT_CODE(KEYCODE_4_PAD)      PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 7")         PORT_CODE(KEYCODE_7_PAD)      PORT_CHAR(UCHAR_MAMEKEY(7_PAD))
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes 20..2f
	PORT_START("COL10")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 1")         PORT_CODE(KEYCODE_1_PAD)      PORT_CHAR(UCHAR_MAMEKEY(1_PAD))
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 2")         PORT_CODE(KEYCODE_2_PAD)      PORT_CHAR(UCHAR_MAMEKEY(2_PAD))
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 0")         PORT_CODE(KEYCODE_0_PAD)      PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes 30..3f
	PORT_START("COL11")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_B)          PORT_CHAR('b') PORT_CHAR('B') PORT_CHAR(0x02) PORT_CHAR(0x02)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_V)          PORT_CHAR('v') PORT_CHAR('V') PORT_CHAR(0x16) PORT_CHAR(0x16)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_C)          PORT_CHAR('c') PORT_CHAR('C') PORT_CHAR(0x03) PORT_CHAR(0x03)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_X)          PORT_CHAR('x') PORT_CHAR('X') PORT_CHAR(0x18) PORT_CHAR(0x18)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_Z)          PORT_CHAR('z') PORT_CHAR('Z') PORT_CHAR(0x1a) PORT_CHAR(0x1a)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("DEL/Esc")      PORT_CODE(KEYCODE_ESC)        PORT_CHAR(UCHAR_MAMEKEY(ESC))

	// keycodes 40..4f
	PORT_START("COL12")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 6")         PORT_CODE(KEYCODE_6_PAD)      PORT_CHAR(UCHAR_MAMEKEY(6_PAD))
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("KP 3")         PORT_CODE(KEYCODE_3_PAD)      PORT_CHAR(UCHAR_MAMEKEY(3_PAD))
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED   )

	// keycodes 50..5f
	PORT_START("COL13")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_H)          PORT_CHAR('h') PORT_CHAR('H') PORT_CHAR(0x08) PORT_CHAR(0x08)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_G)          PORT_CHAR('g') PORT_CHAR('G') PORT_CHAR(0x07) PORT_CHAR(0x07)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_F)          PORT_CHAR('f') PORT_CHAR('F') PORT_CHAR(0x06) PORT_CHAR(0x06)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_D)          PORT_CHAR('d') PORT_CHAR('D') PORT_CHAR(0x04) PORT_CHAR(0x04)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_S)          PORT_CHAR('s') PORT_CHAR('S') PORT_CHAR(0x13) PORT_CHAR(0x13)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_A)          PORT_CHAR('a') PORT_CHAR('A') PORT_CHAR(0x01) PORT_CHAR(0x01)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED   )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Caps Lock")    PORT_CODE(KEYCODE_CAPSLOCK)   PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))

	// keycodes 60..6f
	PORT_START("COL14")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_U)          PORT_CHAR('u') PORT_CHAR('U') PORT_CHAR(0x15) PORT_CHAR(0x15)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_Y)          PORT_CHAR('y') PORT_CHAR('Y') PORT_CHAR(0x19) PORT_CHAR(0x19)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_T)          PORT_CHAR('t') PORT_CHAR('T') PORT_CHAR(0x14) PORT_CHAR(0x14)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_R)          PORT_CHAR('r') PORT_CHAR('R') PORT_CHAR(0x12) PORT_CHAR(0x12)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_E)          PORT_CHAR('e') PORT_CHAR('E') PORT_CHAR(0x05) PORT_CHAR(0x05)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_W)          PORT_CHAR('w') PORT_CHAR('W') PORT_CHAR(0x17) PORT_CHAR(0x17)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_Q)          PORT_CHAR('q') PORT_CHAR('Q') PORT_CHAR(0x11) PORT_CHAR(0x11)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Tab")          PORT_CODE(KEYCODE_TAB)        PORT_CHAR(9)

	// keycodes 70..7f
	PORT_START("COL15")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_7)          PORT_CHAR('7') PORT_CHAR('&')
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_6)          PORT_CHAR('6') PORT_CHAR('^')
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_5)          PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_4)          PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_3)          PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_2)          PORT_CHAR('2') PORT_CHAR('@')
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_1)          PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD )                           PORT_CODE(KEYCODE_TILDE)      PORT_CHAR('`') PORT_CHAR('~')
INPUT_PORTS_END


INPUT_PORTS_START( hle_hp_ipc_device )
	PORT_INCLUDE( basic )
	PORT_INCLUDE( id )
INPUT_PORTS_END


} // anonymous namespace


/***************************************************************************
    BASE HLE KEYBOARD DEVICE
***************************************************************************/

/*--------------------------------------------------
    hle_device_base::hle_device_base
    designated device constructor
--------------------------------------------------*/

hle_device_base::hle_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_hp_hil_interface(mconfig, *this)
	, device_matrix_keyboard_interface(mconfig, *this, "COL1", "COL2", "COL3", "COL4", "COL5", "COL6", "COL7", "COL8", "COL9", "COL10", "COL11", "COL12", "COL13", "COL14", "COL15")
{ }


/*--------------------------------------------------
    hle_device_base::~hle_device_base
    destructor
--------------------------------------------------*/

hle_device_base::~hle_device_base()
{ }


/*--------------------------------------------------
    hle_device_base::device_start
    perform expensive initialisations, allocate
    resources, register for save state
--------------------------------------------------*/

void hle_device_base::device_start()
{
	set_hp_hil_mlc_device();

	m_powerup = true;
	m_passthru = false;
}


/*--------------------------------------------------
    hle_device_base::device_reset
    perform startup tasks, also used for host
    requested reset
--------------------------------------------------*/

void hle_device_base::device_reset()
{
	m_fifo.clear();

	// kick the base
	reset_key_state();
	start_processing(attotime::from_hz(1'200));
}


void hle_device_base::hil_write(uint16_t data)
{
	int frames = 0;
//  printf("rx from mlc %04X (%s %02X)\n", data, BIT(data, 11) ? "command" : "data", data & 255);

	if (BIT(data, 11)) switch (data & 255)
	{
	case HPHIL_IFC:
		m_powerup = false;
		break;

	case HPHIL_EPT:
		m_passthru = true;
		break;

	case HPHIL_ELB:
		m_passthru = false;
		break;

	case HPHIL_ACF+1:
		m_device_id = data & 7;
		m_device_id16 = m_device_id << 8;
		m_hp_hil_mlc->hil_write((data & ~7) | ((data + 1) & 7));
		return;
		break;

	case HPHIL_POL:
		if (!m_fifo.empty())
		{
			m_hp_hil_mlc->hil_write(m_device_id16 | 0x40);  // Keycode Set 1, no coordinate data
			frames = 1;
			while (!m_fifo.empty())
			{
				m_hp_hil_mlc->hil_write(m_device_id16 | m_fifo.dequeue());
				frames++;
			}
		}
		m_hp_hil_mlc->hil_write(HPMLC_W1_C | m_device_id16 | HPHIL_POL | ((data + frames) & 7));
		return;
		break;

	case HPHIL_DSR:
		m_device_id = m_device_id16 = 0;
		m_powerup = true;
		break;

	case HPHIL_IDD:
		m_hp_hil_mlc->hil_write(m_device_id16 | ioport("COL0")->read());
		m_hp_hil_mlc->hil_write(m_device_id16 | 0);
		break;

	case HPHIL_DHR:
		device_reset();
		return;
		break;

	default:
		logerror("command %02X unknown\n", data & 255);
		break;
	}

	if (!m_passthru)
		m_hp_hil_mlc->hil_write(data);
//  else
//      m_next->hil_write(data);
}

void hle_device_base::transmit_byte(uint8_t byte)
{
	if (!m_fifo.full()) {
//      printf("queuing %02X\n", byte);
		m_fifo.enqueue(byte);
	}
//  else
//      printf("queuing fail (fifo full)\n");
}

/*--------------------------------------------------
    hle_device_base::key_make
    handle a key being pressed
--------------------------------------------------*/

void hle_device_base::key_make(uint8_t row, uint8_t column)
{
	transmit_byte((((row + 1) ^ 8) << 4) + (column << 1));
}


/*--------------------------------------------------
    hle_device_base::key_break
    handle a key being released
--------------------------------------------------*/

void hle_device_base::key_break(uint8_t row, uint8_t column)
{
	transmit_byte((((row + 1) ^ 8) << 4) + (column << 1) + 1);
}


/***************************************************************************
    HP INTEGRAL HLE KEYBOARD DEVICE
***************************************************************************/

/*--------------------------------------------------
    hle_hp_ipc_device::hle_hp_ipc_device
    abbreviated constructor
--------------------------------------------------*/

hle_hp_ipc_device::hle_hp_ipc_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: hle_device_base(mconfig, HP_IPC_HLE_KEYBOARD, tag, owner, clock)
{ }


/*--------------------------------------------------
    hle_hp_ipc_device::device_input_ports
    get input ports for this device
--------------------------------------------------*/

ioport_constructor hle_hp_ipc_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(hle_hp_ipc_device);
}


} } // namespace bus::hp_hil
