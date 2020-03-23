// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic, Robbbert
/*********************************************************************************************************************************

Science of Cambridge MK-14

2009-11-20 Skeleton driver.
2016-08-21 Working

Keys:

UP: MEM increments the currently displayed address, (and goes into data entry mode in V1 bios).
= : TERM changes to "data entry" mode. In this mode, entering hex digits will change the byte at the currently displayed address
- : ABORT changes to "address entry" mode. In this mode, entering hex digits will change the address.
X : GO runs the program from the currently displayed address. On exit, the instruction after the program is displayed

Pasting:
        0-F : as is
        MEM : ^
        TERM: =
        AB :  -
        GO :  X

Example program: ("organ" from p82 of the manual)
-F20=C4^0D^35^C4^00^31^C4^08^C8^F6^C5^01^E4^FF^98^08^8F^00^06^E4^07^07^90^EB^B8^E6^9C^EE^90^E5^-F20X
Pressing keys will produces different tones.


ToDo:
- VDU optional attachment (we are missing the chargen rom)
- The original version of the bios is missing (we have version 2)

*********************************************************************************************************************************/

#include "emu.h"

#include "cpu/scmp/scmp.h"
#include "imagedev/cassette.h"
#include "machine/ins8154.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "sound/wave.h"
#include "speaker.h"

#include "mk14.lh"


class mk14_state : public driver_device
{
public:
	mk14_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_keyboard(*this, "X.%u", 0)
		, m_cass(*this, "cassette")
		, m_dac(*this, "dac")
	{ }

	DECLARE_READ8_MEMBER(keyboard_r);
	DECLARE_WRITE8_MEMBER(display_w);
	DECLARE_WRITE8_MEMBER(port_a_w);
	DECLARE_WRITE_LINE_MEMBER(cass_w);
	DECLARE_READ_LINE_MEMBER(cass_r);
private:
	virtual void machine_reset() override;
	required_device<cpu_device> m_maincpu;
	required_ioport_array<8> m_keyboard;
	required_device<cassette_image_device> m_cass;
	required_device<dac_bit_interface> m_dac;
};

/*
000-1FF  512 byte SCIOS ROM  Decoded by 0xxx
200-3FF  ROM Shadow / Expansion RAM
400-5FF  ROM Shadow / Expansion RAM
600-7FF  ROM Shadow / Expansion RAM
800-87F  I/O Ports  Decoded by 1xx0
880-8FF  128 bytes I/O chip RAM  Decoded by 1xx0
900-9FF  Keyboard & Display  Decoded by 1x01
A00-AFF  I/O Port & RAM Shadow
B00-BFF  256 bytes RAM (Extended) / VDU RAM  Decoded by 1011
C00-CFF  I/O Port & RAM Shadow
D00-DFF  Keyboard & Display Shadow
E00-EFF  I/O Port & RAM Shadow
F00-FFF  256 bytes RAM (Standard) / VDU RAM  Decoded by 1111

*/


READ8_MEMBER( mk14_state::keyboard_r )
{
	if (offset < 8)
		return m_keyboard[offset]->read();
	else
		return 0xff;
}

WRITE8_MEMBER( mk14_state::display_w )
{
	if (offset < 8 )
		output().set_digit_value(offset, data);
	else
	{
		//logerror("write %02x to %02x\n",data,offset);
	}
}

static ADDRESS_MAP_START(mk14_mem, AS_PROGRAM, 8, mk14_state)
	ADDRESS_MAP_UNMAP_HIGH
	ADDRESS_MAP_GLOBAL_MASK(0x0fff)
	AM_RANGE(0x000, 0x1ff) AM_MIRROR(0x600) AM_ROM // ROM
	AM_RANGE(0x800, 0x87f) AM_MIRROR(0x600) AM_DEVREADWRITE("ic8", ins8154_device, ins8154_r, ins8154_w) // I/O
	AM_RANGE(0x880, 0x8ff) AM_MIRROR(0x600) AM_RAM // 128 I/O chip RAM
	AM_RANGE(0x900, 0x9ff) AM_MIRROR(0x400) AM_READWRITE(keyboard_r, display_w)
	AM_RANGE(0xb00, 0xbff) AM_RAM // VDU RAM
	AM_RANGE(0xf00, 0xfff) AM_RAM // Standard RAM
ADDRESS_MAP_END


/* Input ports */
static INPUT_PORTS_START( mk14 )
	PORT_START("X.0")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A")    PORT_CODE(KEYCODE_A)      PORT_CHAR('A')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8")    PORT_CODE(KEYCODE_8)      PORT_CHAR('8')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0")    PORT_CODE(KEYCODE_0)      PORT_CHAR('0')
	PORT_START("X.1")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B")    PORT_CODE(KEYCODE_B)      PORT_CHAR('B')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9")    PORT_CODE(KEYCODE_9)      PORT_CHAR('9')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1")    PORT_CODE(KEYCODE_1)      PORT_CHAR('1')
	PORT_START("X.2")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C")    PORT_CODE(KEYCODE_C)      PORT_CHAR('C')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GO")   PORT_CODE(KEYCODE_X)      PORT_CHAR('X')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2")    PORT_CODE(KEYCODE_2)      PORT_CHAR('2')
	PORT_START("X.3")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D")    PORT_CODE(KEYCODE_D)      PORT_CHAR('D')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("MEM")  PORT_CODE(KEYCODE_UP)     PORT_CHAR('^')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3")    PORT_CODE(KEYCODE_3)      PORT_CHAR('3')
	PORT_START("X.4")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("AB")   PORT_CODE(KEYCODE_MINUS)  PORT_CHAR('-')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4")    PORT_CODE(KEYCODE_4)      PORT_CHAR('4')
	PORT_START("X.5")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5")    PORT_CODE(KEYCODE_5)      PORT_CHAR('5')
	PORT_START("X.6")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E")    PORT_CODE(KEYCODE_E)      PORT_CHAR('E')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6")    PORT_CODE(KEYCODE_6)      PORT_CHAR('6')
	PORT_START("X.7")
		PORT_BIT(0x0F, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F")    PORT_CODE(KEYCODE_F)      PORT_CHAR('F')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TERM") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7")    PORT_CODE(KEYCODE_7)      PORT_CHAR('7')
INPUT_PORTS_END

WRITE8_MEMBER( mk14_state::port_a_w )
{
}

WRITE_LINE_MEMBER( mk14_state::cass_w )
{
	m_cass->output(state ? -1.0 : +1.0);
	m_dac->write(state);
}

READ_LINE_MEMBER( mk14_state::cass_r )
{
	return (m_cass->input() > 0.03) ? 1 : 0;
}

void mk14_state::machine_reset()
{
}

static MACHINE_CONFIG_START( mk14 )
	/* basic machine hardware */
	// IC1 1SP-8A/600 (8060) SC/MP Microprocessor
	MCFG_CPU_ADD("maincpu", INS8060, XTAL_4_433619MHz)
	MCFG_SCMP_CONFIG(WRITELINE(mk14_state, cass_w), NOOP, READLINE(mk14_state, cass_r), NOOP, READLINE(mk14_state, cass_r), NOOP)
	MCFG_CPU_PROGRAM_MAP(mk14_mem)

	/* video hardware */
	MCFG_DEFAULT_LAYOUT(layout_mk14)

	// sound
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_WAVE_ADD(WAVE_TAG, "cassette")
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.05)
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_SOUND_ADD("dac8", ZN425E, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5) // Ferranti ZN425E
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "dac8", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac8", -1.0, DAC_VREF_NEG_INPUT)

	/* devices */
	MCFG_DEVICE_ADD("ic8", INS8154, 0)
	MCFG_INS8154_OUT_A_CB(WRITE8(mk14_state, port_a_w))
	MCFG_INS8154_OUT_B_CB(DEVWRITE8("dac8", dac_byte_interface, write))

	MCFG_CASSETTE_ADD( "cassette" )
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( mk14 )
	ROM_REGION( 0x200, "maincpu", 0 )
	// IC2,3 74S571 512 x 4 bit ROM
	ROM_LOAD( "scios.bin", 0x0000, 0x0200, CRC(8b667daa) SHA1(802dc637ce5391a2a6627f76f919b12a869b56ef)) // V2 bios, V1 is missing
ROM_END

/* Driver */

//    YEAR  NAME   PARENT  COMPAT  MACHINE    INPUT  CLASS       INIT  COMPANY                 FULLNAME  FLAGS
COMP( 1977, mk14,  0,      0,      mk14,      mk14,  mk14_state, 0,    "Science of Cambridge", "MK-14",  0 )
