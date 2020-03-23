// license:GPL-2.0+
// copyright-holders:Juergen Buchmueller
/******************************************************************************

    kim1.cpp - KIM-1

    LED: six 7-segment LEDs
        left 4 digits (address)
        right 2 digits (data)
    Keyboard: 23 keys and SST switch
        0-F  16 keys to enter data
        AD   address entry mode
        DA   data entry mode
        +    increment address
        PC   recalls address stored in the Program Counter
        RS   system reset
        GO   execute program
        ST   program stop
        SST  single step slide switch

    How to use cassette:
        00F1      00 to clear decimal mode
        17F5-17F6 start address low and high
        17F7-17F8 end address low and high
        17F9      2 digit program ID
        1800      press GO to save tape
        1873      press GO to load tape
    NOTE: save end address is next address from program end

The cassette interface
======================
The KIM-1 stores data on cassette using 2 frequencies: ~3700Hz (high) and ~2400Hz
(low). A high tone is output for 9 cycles and a low tone for 6 cycles. A logic bit
is encoded using 3 sequences of high and low tones. It always starts with a high
tone and ends with a low tone. The middle tone is high for a logic 0 and low for
0 logic 1.

These high and low tone signals are fed to a circuit containing a LM565 PLL and
a 311 comparator. For a high tone a 1 is passed to DB7 of 6530-U2 for a low tone
a 0 is passed. The KIM-1 software measures the time it takes for the signal to
change from 1 to 0.

Keyboard and Display logic
==========================
PA0-PA6 of 6530-U2 are connected to the columns of the keyboard matrix. These
columns are also connected to segments A-G of the LEDs. PB1-PB3 of 6530-U2 are
connected to a 74145 BCD which connects outputs 0-2 to the rows of the keyboard
matrix. Outputs 4-9 of the 74145 are connected to LEDs U18-U23

When a key is pressed the corresponding input to PA0-PA6 is set low and the KIM-1
software reads this signal. The KIM-1 software sends an output signal to PA0-PA6
and the corresponding segments of an LED are illuminated.

TODO:
- LEDs should be dark at startup (RS key to activate)
- hook up Single Step dip switch
- slots for expansion & application ports
- add TTY support
******************************************************************************/

#include "emu.h"
#include "includes/kim1.h"
#include "kim1.lh"

//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

static ADDRESS_MAP_START(kim1_map, AS_PROGRAM, 8, kim1_state)
	AM_RANGE(0x0000, 0x03ff) AM_MIRROR(0xe000) AM_RAM
	AM_RANGE(0x1700, 0x173f) AM_MIRROR(0xe000) AM_DEVREADWRITE("miot_u3", mos6530_device, read, write )
	AM_RANGE(0x1740, 0x177f) AM_MIRROR(0xe000) AM_DEVREADWRITE("miot_u2", mos6530_device, read, write )
	AM_RANGE(0x1780, 0x17bf) AM_MIRROR(0xe000) AM_RAM
	AM_RANGE(0x17c0, 0x17ff) AM_MIRROR(0xe000) AM_RAM
	AM_RANGE(0x1800, 0x1bff) AM_MIRROR(0xe000) AM_ROM
	AM_RANGE(0x1c00, 0x1fff) AM_MIRROR(0xe000) AM_ROM
ADDRESS_MAP_END

// RS and ST key input
INPUT_CHANGED_MEMBER(kim1_state::trigger_reset)
{
		m_maincpu->set_input_line(INPUT_LINE_RESET, newval ? CLEAR_LINE : ASSERT_LINE);
}

INPUT_CHANGED_MEMBER(kim1_state::trigger_nmi)
{
		m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? CLEAR_LINE : ASSERT_LINE);
}

//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( kim1 )
	PORT_START("ROW0")
	PORT_BIT( 0x80, 0x00, IPT_UNUSED )
	PORT_BIT( 0x40, 0x40, IPT_KEYBOARD ) PORT_NAME("0") PORT_CODE(KEYCODE_0)      PORT_CHAR('0') PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT( 0x20, 0x20, IPT_KEYBOARD ) PORT_NAME("1") PORT_CODE(KEYCODE_1)      PORT_CHAR('1') PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT( 0x10, 0x10, IPT_KEYBOARD ) PORT_NAME("2") PORT_CODE(KEYCODE_2)      PORT_CHAR('2') PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT( 0x08, 0x08, IPT_KEYBOARD ) PORT_NAME("3") PORT_CODE(KEYCODE_3)      PORT_CHAR('3') PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT( 0x04, 0x04, IPT_KEYBOARD ) PORT_NAME("4") PORT_CODE(KEYCODE_4)      PORT_CHAR('4') PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT( 0x02, 0x02, IPT_KEYBOARD ) PORT_NAME("5") PORT_CODE(KEYCODE_5)      PORT_CHAR('5') PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT( 0x01, 0x01, IPT_KEYBOARD ) PORT_NAME("6") PORT_CODE(KEYCODE_6)      PORT_CHAR('6') PORT_CODE(KEYCODE_6_PAD)

	PORT_START("ROW1")
	PORT_BIT( 0x80, 0x00, IPT_UNUSED )
	PORT_BIT( 0x40, 0x40, IPT_KEYBOARD ) PORT_NAME("7") PORT_CODE(KEYCODE_7)      PORT_CHAR('7') PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT( 0x20, 0x20, IPT_KEYBOARD ) PORT_NAME("8") PORT_CODE(KEYCODE_8)      PORT_CHAR('8') PORT_CODE(KEYCODE_8_PAD)
	PORT_BIT( 0x10, 0x10, IPT_KEYBOARD ) PORT_NAME("9") PORT_CODE(KEYCODE_9)      PORT_CHAR('9') PORT_CODE(KEYCODE_9_PAD)
	PORT_BIT( 0x08, 0x08, IPT_KEYBOARD ) PORT_NAME("A") PORT_CODE(KEYCODE_A)      PORT_CHAR('A')
	PORT_BIT( 0x04, 0x04, IPT_KEYBOARD ) PORT_NAME("B") PORT_CODE(KEYCODE_B)      PORT_CHAR('B')
	PORT_BIT( 0x02, 0x02, IPT_KEYBOARD ) PORT_NAME("C") PORT_CODE(KEYCODE_C)      PORT_CHAR('C')
	PORT_BIT( 0x01, 0x01, IPT_KEYBOARD ) PORT_NAME("D") PORT_CODE(KEYCODE_D)      PORT_CHAR('D')

	PORT_START("ROW2")
	PORT_BIT( 0x80, 0x00, IPT_UNUSED )
	PORT_BIT( 0x40, 0x40, IPT_KEYBOARD ) PORT_NAME("E")  PORT_CODE(KEYCODE_E)      PORT_CHAR('E')
	PORT_BIT( 0x20, 0x20, IPT_KEYBOARD ) PORT_NAME("F")  PORT_CODE(KEYCODE_F)      PORT_CHAR('F')
	PORT_BIT( 0x10, 0x10, IPT_KEYBOARD ) PORT_NAME("AD") PORT_CODE(KEYCODE_MINUS)  PORT_CHAR('-') PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT( 0x08, 0x08, IPT_KEYBOARD ) PORT_NAME("DA") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=')
	PORT_BIT( 0x04, 0x04, IPT_KEYBOARD ) PORT_NAME("+")  PORT_CODE(KEYCODE_UP)     PORT_CHAR(' ') PORT_CODE(KEYCODE_PLUS_PAD)
	PORT_BIT( 0x02, 0x02, IPT_KEYBOARD ) PORT_NAME("GO") PORT_CODE(KEYCODE_ENTER)  PORT_CHAR(13)  PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT( 0x01, 0x01, IPT_KEYBOARD ) PORT_NAME("PC") PORT_CODE(KEYCODE_F6)

	PORT_START("SPECIAL")
	PORT_BIT( 0x80, 0x00, IPT_UNUSED )
	PORT_BIT( 0x40, 0x40, IPT_KEYBOARD ) PORT_NAME("sw1: ST") PORT_CODE(KEYCODE_F7) PORT_CHANGED_MEMBER(DEVICE_SELF, kim1_state, trigger_nmi, nullptr)
	PORT_BIT( 0x20, 0x20, IPT_KEYBOARD ) PORT_NAME("sw2: RS") PORT_CODE(KEYCODE_F3) PORT_CHANGED_MEMBER(DEVICE_SELF, kim1_state, trigger_reset, nullptr)
	PORT_DIPNAME(0x10, 0x10, "sw3: SS")                       PORT_CODE(KEYCODE_NUMLOCK) PORT_TOGGLE
	PORT_DIPSETTING( 0x00, "single step")
	PORT_DIPSETTING( 0x10, "run")
	PORT_BIT( 0x08, 0x00, IPT_UNUSED )
	PORT_BIT( 0x04, 0x00, IPT_UNUSED )
	PORT_BIT( 0x02, 0x00, IPT_UNUSED )
	PORT_BIT( 0x01, 0x00, IPT_UNUSED )
INPUT_PORTS_END

// Read from keyboard
READ8_MEMBER( kim1_state::kim1_u2_read_a )
{
	uint8_t data = 0xff;

	switch( ( m_u2_port_b >> 1 ) & 0x0f )
	{
	case 0:
		data = m_row0->read();
		break;
	case 1:
		data = m_row1->read();
		break;
	case 2:
		data = m_row2->read();
		break;
	}
	return data;
}

// Write to 7-Segment LEDs
WRITE8_MEMBER( kim1_state::kim1_u2_write_a )
{
	uint8_t idx = ( m_u2_port_b >> 1 ) & 0x0f;

	if ( idx >= 4 && idx < 10 )
	{
		if ( data & 0x80 )
		{
			output().set_digit_value( idx-4, data & 0x7f );
			m_led_time[idx - 4] = 15;
		}
	}
}

// Load from cassette
READ8_MEMBER( kim1_state::kim1_u2_read_b )
{
	if ( m_riot2->portb_out_get() & 0x20 )
		return 0xFF;

	return 0x7F | ( m_311_output ^ 0x80 );
}

// Save to cassette
WRITE8_MEMBER( kim1_state::kim1_u2_write_b )
{
	m_u2_port_b = data;

	if ( data & 0x20 )
		/* cassette write/speaker update */
		m_cass->output(( data & 0x80 ) ? -1.0 : 1.0 );

	/* Set IRQ when bit 7 is cleared */
}

TIMER_DEVICE_CALLBACK_MEMBER(kim1_state::kim1_cassette_input)
{
	double tap_val = m_cass->input();

	if ( tap_val <= 0 )
	{
		if ( m_cassette_high_count )
		{
			m_311_output = ( m_cassette_high_count < 8 ) ? 0x80 : 0;
			m_cassette_high_count = 0;
		}
	}

	if ( tap_val > 0 )
		m_cassette_high_count++;
}

// Blank LEDs during cassette operations
TIMER_DEVICE_CALLBACK_MEMBER(kim1_state::kim1_update_leds)
{
	uint8_t i;

	for ( i = 0; i < 6; i++ )
	{
		if ( m_led_time[i] )
			m_led_time[i]--;
		else
			output().set_digit_value( i, 0 );
	}
}

// Register for save states
void kim1_state::machine_start()
{
	save_item(NAME(m_u2_port_b));
	save_item(NAME(m_311_output));
	save_item(NAME(m_cassette_high_count));
}

void kim1_state::machine_reset()
{
	uint8_t i;

	for ( i = 0; i < 6; i++ )
		m_led_time[i] = 0;

	m_311_output = 0;
	m_cassette_high_count = 0;
}

//**************************************************************************
//  MACHINE DRIVERS
//**************************************************************************

static MACHINE_CONFIG_START( kim1 )
	// basic machine hardware
	MCFG_CPU_ADD("maincpu", M6502, 1000000)        /* 1 MHz */
	MCFG_CPU_PROGRAM_MAP(kim1_map)
	MCFG_QUANTUM_TIME(attotime::from_hz(60))

	// video hardware
	MCFG_DEFAULT_LAYOUT( layout_kim1 )

	// devices
	MCFG_DEVICE_ADD("miot_u2", MOS6530, 1000000)
	MCFG_MOS6530_IN_PA_CB(READ8(kim1_state, kim1_u2_read_a))
	MCFG_MOS6530_OUT_PA_CB(WRITE8(kim1_state, kim1_u2_write_a))
	MCFG_MOS6530_IN_PB_CB(READ8(kim1_state, kim1_u2_read_b))
	MCFG_MOS6530_OUT_PB_CB(WRITE8(kim1_state, kim1_u2_write_b))

	MCFG_DEVICE_ADD("miot_u3", MOS6530, 1000000)

	MCFG_CASSETTE_ADD( "cassette" )
	MCFG_CASSETTE_FORMATS(kim1_cassette_formats)
	MCFG_CASSETTE_DEFAULT_STATE(CASSETTE_STOPPED)
	MCFG_CASSETTE_INTERFACE ("kim1_cass")

	MCFG_TIMER_DRIVER_ADD_PERIODIC("led_timer", kim1_state, kim1_update_leds, attotime::from_hz(60))
	MCFG_TIMER_DRIVER_ADD_PERIODIC("cassette_timer", kim1_state, kim1_cassette_input, attotime::from_hz(44100))

	// software list
	MCFG_SOFTWARE_LIST_ADD ("cass_list", "kim1_cass")

MACHINE_CONFIG_END

//**************************************************************************
//  ROM DEFINITIONS
//**************************************************************************

ROM_START(kim1)
	ROM_REGION(0x10000,"maincpu",0)
	ROM_LOAD("6530-003.bin",    0x1800, 0x0400, CRC(a2a56502) SHA1(60b6e48f35fe4899e29166641bac3e81e3b9d220))
	ROM_LOAD("6530-002.bin",    0x1c00, 0x0400, CRC(2b08e923) SHA1(054f7f6989af3a59462ffb0372b6f56f307b5362))
ROM_END

//**************************************************************************
//  SYSTEM DRIVERS
//**************************************************************************

//    YEAR  NAME      PARENT    COMPAT  MACHINE   INPUT  CLASS           INIT  COMPANY             FULLNAME  FLAGS
COMP( 1975, kim1,     0,        0,      kim1,     kim1,  kim1_state,     0,    "MOS Technologies", "KIM-1" , MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE)
