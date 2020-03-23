// license:BSD-3-Clause
// copyright-holders:Dirk Best, Robbbert
/******************************************************************************

 Acorn System 1 (Microcomputer Kit)

 Skeleton driver

 Driver by Dirk Best

2011-06-26 Keyboard & display fixed. [Robbbert]


http://speleotrove.com/acorn/

-   (modify) Memory display and modification    l   (load) Reads a block of bytes from tape
X   (go) Run program starting at an address     r   (return) Resume after a breakpoint
p   (point) Inserts or removes breakpoint       (up) Increment displayed address
s   (store) Writes a block of bytes to tape     (down) Decrement displayed address

Pasting:
        0-F : as is
        (inc) : ^
        (dec) : V
        M (memory) : -
        G (Go) : X

Test Paste:
        -0100^11^22^33^44^55^66^77^88^99^-0100^
        Now press up-arrow to confirm the data has been entered.


Example usage: Turn on. Press -. Mode letter will show 'A'. Type in an address
               (example FE00). Press - (or any command key). Contents will show
               on the right. Use Up & Down keys to cycle through addresses.

To save a tape, press S then enter start address, press S, enter end address+1,
               start recording and press S. The save only takes a few seconds.

To load a tape, the display must just have dots, (reset if necessary), start
               playing tape and immediately press L. The last digit will flicker
               as the bytes load. At the end, the dots will show again. There's
               no error checking, so if it doesn't work, reset and try again.

Note that left-most digit is not wired up, and therefore will always be blank.

******************************************************************************/

#include "emu.h"
#include "cpu/m6502/m6502.h"
#include "machine/ins8154.h"
#include "machine/74145.h"
#include "imagedev/cassette.h"
#include "sound/wave.h"
#include "speaker.h"

#include "acrnsys1.lh"


class acrnsys1_state : public driver_device
{
public:
	acrnsys1_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_ttl74145(*this, "ic8_7445"),
		m_cass(*this, "cassette"),
		m_digit(0)
	{ }

	DECLARE_READ8_MEMBER(ins8154_b1_port_a_r);
	DECLARE_WRITE8_MEMBER(ins8154_b1_port_a_w);
	DECLARE_WRITE8_MEMBER(acrnsys1_led_segment_w);
	TIMER_DEVICE_CALLBACK_MEMBER(acrnsys1_c);
	TIMER_DEVICE_CALLBACK_MEMBER(acrnsys1_p);
private:
	required_device<cpu_device> m_maincpu;
	required_device<ttl74145_device> m_ttl74145;
	required_device<cassette_image_device> m_cass;
	uint8_t m_digit;
	uint8_t m_cass_data[4];
	bool m_cass_state;
	bool m_cassold;
};



/***************************************************************************
    KEYBOARD HANDLING
***************************************************************************/
// bit 7 is cassin
READ8_MEMBER( acrnsys1_state::ins8154_b1_port_a_r )
{
	uint8_t data = 0x7f, i, key_line = m_ttl74145->read();

	for (i = 0; i < 8; i++)
	{
		if (BIT(key_line, i))
		{
			char kbdrow[6];
			sprintf(kbdrow,"X%X",i);
			data = (ioport(kbdrow)->read() & 0x38) | m_digit;
			break;
		}
	}
	data |= m_cass_data[2];
	return data;
}

// bit 6 is cassout
WRITE8_MEMBER( acrnsys1_state::ins8154_b1_port_a_w )
{
	m_digit = data & 0x47;
	m_ttl74145->write(m_digit & 7);
	m_cass_state = BIT(data, 6);
}

TIMER_DEVICE_CALLBACK_MEMBER(acrnsys1_state::acrnsys1_c)
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

TIMER_DEVICE_CALLBACK_MEMBER(acrnsys1_state::acrnsys1_p)
{
	/* cassette - turn 1200/2400Hz to a bit */
	m_cass_data[1]++;
	uint8_t cass_ws = (m_cass->input() > +0.03) ? 1 : 0;

	if (cass_ws != m_cass_data[0])
	{
		m_cass_data[0] = cass_ws;
		m_cass_data[2] = ((m_cass_data[1] < 12) ? 128 : 0);
		m_cass_data[1] = 0;
	}
}

/***************************************************************************
    LED DISPLAY
***************************************************************************/

WRITE8_MEMBER( acrnsys1_state::acrnsys1_led_segment_w )
{
	uint8_t key_line = m_ttl74145->read();

	output().set_digit_value(key_line, data);
}


/***************************************************************************
    ADDRESS MAPS
***************************************************************************/

static ADDRESS_MAP_START( acrnsys1_map, AS_PROGRAM, 8, acrnsys1_state )
	AM_RANGE(0x0000, 0x03ff) AM_RAM
	AM_RANGE(0x0e00, 0x0e7f) AM_MIRROR(0x100) AM_DEVREADWRITE("b1", ins8154_device, ins8154_r, ins8154_w)
	AM_RANGE(0x0e80, 0x0eff) AM_MIRROR(0x100) AM_RAM
	AM_RANGE(0xf800, 0xf9ff) AM_MIRROR(0x600) AM_ROM
ADDRESS_MAP_END


/***************************************************************************
    INPUT PORTS
***************************************************************************/

static INPUT_PORTS_START( acrnsys1 )
	PORT_START("X0")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8") PORT_CODE(KEYCODE_8) PORT_CHAR('8')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("M") PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0") PORT_CODE(KEYCODE_0) PORT_CHAR('0')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X1")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9") PORT_CODE(KEYCODE_9) PORT_CHAR('9')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G") PORT_CODE(KEYCODE_X) PORT_CHAR('X')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1") PORT_CODE(KEYCODE_1) PORT_CHAR('1')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A") PORT_CODE(KEYCODE_A) PORT_CHAR('A')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P") PORT_CODE(KEYCODE_P) PORT_CHAR('P')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2") PORT_CODE(KEYCODE_2) PORT_CHAR('2')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X3")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B") PORT_CODE(KEYCODE_B) PORT_CHAR('B')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("S") PORT_CODE(KEYCODE_S) PORT_CHAR('S')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3") PORT_CODE(KEYCODE_3) PORT_CHAR('3')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X4")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C") PORT_CODE(KEYCODE_C) PORT_CHAR('C')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L") PORT_CODE(KEYCODE_L) PORT_CHAR('L')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4") PORT_CODE(KEYCODE_4) PORT_CHAR('4')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X5")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D") PORT_CODE(KEYCODE_D) PORT_CHAR('D')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R") PORT_CODE(KEYCODE_R) PORT_CHAR('R')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5") PORT_CODE(KEYCODE_5) PORT_CHAR('5')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X6")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E") PORT_CODE(KEYCODE_E) PORT_CHAR('E')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_UP) PORT_CODE(KEYCODE_UP) PORT_CHAR('^')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6") PORT_CODE(KEYCODE_6) PORT_CHAR('6')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X7")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F") PORT_CODE(KEYCODE_F) PORT_CHAR('F')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_DOWN) PORT_CODE(KEYCODE_DOWN) PORT_CHAR('V')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7") PORT_CODE(KEYCODE_7) PORT_CHAR('7')
	PORT_BIT(0xc7, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("reset")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RST") PORT_CODE(KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3))
	PORT_BIT(0xfe, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("switch")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Switch") PORT_CODE(KEYCODE_F1) PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0xfe, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("config")
	PORT_CONFNAME( 0x03, 0x00, "Switch connected to")
	PORT_CONFSETTING( 0x00, "IRQ" )
	PORT_CONFSETTING( 0x01, "NMI" )
	PORT_CONFSETTING( 0x02, "RST" )
INPUT_PORTS_END


/***************************************************************************
    MACHINE DRIVERS
***************************************************************************/

static MACHINE_CONFIG_START( acrnsys1 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, 1008000)  /* 1.008 MHz */
	MCFG_CPU_PROGRAM_MAP(acrnsys1_map)

	MCFG_DEFAULT_LAYOUT(layout_acrnsys1)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_WAVE_ADD(WAVE_TAG, "cassette")
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)

	/* devices */
	MCFG_DEVICE_ADD("b1", INS8154, 0)
	MCFG_INS8154_IN_A_CB(READ8(acrnsys1_state, ins8154_b1_port_a_r))
	MCFG_INS8154_OUT_A_CB(WRITE8(acrnsys1_state, ins8154_b1_port_a_w))
	MCFG_INS8154_OUT_B_CB(WRITE8(acrnsys1_state, acrnsys1_led_segment_w))
	MCFG_DEVICE_ADD("ic8_7445", TTL74145, 0)
	MCFG_CASSETTE_ADD( "cassette" )
	MCFG_TIMER_DRIVER_ADD_PERIODIC("acrnsys1_c", acrnsys1_state, acrnsys1_c, attotime::from_hz(4800))
	MCFG_TIMER_DRIVER_ADD_PERIODIC("acrnsys1_p", acrnsys1_state, acrnsys1_p, attotime::from_hz(40000))
MACHINE_CONFIG_END


/***************************************************************************
    ROM DEFINITIONS
***************************************************************************/

ROM_START( acrnsys1 )
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("acrnsys1.bin", 0xf800, 0x0200, CRC(43dcfc16) SHA1(b987354c55beb5e2ced761970c3339b895a8c09d))
ROM_END


/***************************************************************************
    GAME DRIVERS
***************************************************************************/

/*    YEAR  NAME      PARENT COMPAT MACHINE   INPUT     STATE           INIT  COMPANY  FULLNAME          FLAGS */
COMP( 1978, acrnsys1, 0,     0,     acrnsys1, acrnsys1, acrnsys1_state, 0,    "Acorn", "Acorn System 1", 0 )
