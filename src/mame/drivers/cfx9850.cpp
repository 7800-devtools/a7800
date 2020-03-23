// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

    Driver for Casio CFX-9850

To operate:
The unit is switched off by default, you have to switch it on by pressing 'Q'.

Currently (year2011) it is on by default, the only key that works is '\'
which turns it off. After that nothing happens.

Normal operation:
- AC/ON - turns the unit on
- SHIFT + OFF - turns the unit off

The unit automatically powers off after no operation for about 6 minutes (intro/quick start, page viii).

Debugging information:
1. g 10b3 (Initialise system)
2. cs=23
3. ip= one of these: ip=d1a4,d1af,d1ba,d1c5,d1d0,d1db,d1ea,d1f9,d208,d217,
   d226,d235,d244
4. g 23108c to get a test pattern on the screen.

***************************************************************************/

#include "emu.h"
#include "cpu/hcd62121/hcd62121.h"
#include "rendlay.h"
#include "screen.h"

class cfx9850_state : public driver_device
{
public:
	cfx9850_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_video_ram(*this, "video_ram")
		, m_display_ram(*this, "display_ram")
		, m_ko_port(*this, "KO%u", 1)
		, m_maincpu(*this, "maincpu")
		, m_ko(0)
		, m_port(0)
		, m_opt(0)
	{ }

	DECLARE_WRITE8_MEMBER(kol_w);
	DECLARE_WRITE8_MEMBER(koh_w);
	DECLARE_WRITE8_MEMBER(port_w);
	DECLARE_WRITE8_MEMBER(opt_w);
	DECLARE_READ8_MEMBER(ki_r);
	DECLARE_READ8_MEMBER(in0_r);
	required_shared_ptr<u8> m_video_ram;
	required_shared_ptr<u8> m_display_ram;
	DECLARE_PALETTE_INIT(cfx9850);
	u32 screen_update_cfx9850(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	required_ioport_array<12> m_ko_port;
	required_device<cpu_device> m_maincpu;

private:
	u16 m_ko;   // KO lines KO1 - KO14
	u8 m_port;  // PORT lines PORT0 - PORT7 (serial I/O)
	u8 m_opt;   // OPT lines OPT0 - OPT7 (contrast)
};


static ADDRESS_MAP_START(cfx9850, AS_PROGRAM, 8, cfx9850_state)
	AM_RANGE( 0x000000, 0x007fff ) AM_ROM
	AM_RANGE( 0x080000, 0x0807ff ) AM_RAM AM_SHARE("video_ram")
//  AM_RANGE( 0x100000, 0x10ffff ) // some memory mapped i/o???
//  AM_RANGE( 0x110000, 0x11ffff ) // some memory mapped i/o???
	AM_RANGE( 0x200000, 0x27ffff ) AM_ROM AM_REGION( "bios", 0 )
	AM_RANGE( 0x400000, 0x40ffff ) AM_RAM
	AM_RANGE( 0x600000, 0x6007ff ) AM_MIRROR(0xf800) AM_RAM AM_SHARE("display_ram")
//  AM_RANGE( 0xe10000, 0xe1ffff ) // some memory mapped i/o???
ADDRESS_MAP_END


WRITE8_MEMBER(cfx9850_state::kol_w)
{
	m_ko = (m_ko & 0xff00) | data;
	logerror("KO is now %04x\n", m_ko);
}


WRITE8_MEMBER(cfx9850_state::koh_w)
{
	m_ko = (m_ko & 0x00ff) | (data << 8);
	logerror("KO is now %04x\n", m_ko);
}


// 7------- PORT7 - RX enable?
// -6------ PORT6 - NC / CP45
// --5----- PORT5 - TX enable?
// ---4---- PORT4 - data out (see code at 2225A0)
// ----3--- PORT3 - NC / CP28
// -----2-- PORT2 - NC / CP29
// ------1- PORT1 - NC / CP30
// -------0 PORT0 - display (enable?) related? + CP31
WRITE8_MEMBER(cfx9850_state::port_w)
{
	m_port = data;
	logerror("PORT is now %02x\n", m_port);
}


// 7------- OPT7 - NC / CP46
// -6------ OPT6 - NC / CP25
// --5----- OPT5 - NC / CP26
// ---4---- OPT4 - NC / CP27
// ----3--- OPT3 - contrast (TC74HC4066AFS pin 12)
// -----2-- OPT2 - contrast (TC74HC4066AFS pin 6)
// ------1- OPT1 - contrast (TC74HC4066AFS pin 5)
// -------0 OPT0 - contrast (TC74HC4066AFS pin 13)
WRITE8_MEMBER(cfx9850_state::opt_w)
{
	m_opt = data;
	logerror("OPT is now %02x\n", m_opt);
}


READ8_MEMBER(cfx9850_state::ki_r)
{
	u8 data = 0;

	for (int i = 0; i < 12; i++)
	{
		if (BIT(m_ko, i))
		{
			data |= m_ko_port[i]->read();
		}
	}

	return data;
}


READ8_MEMBER(cfx9850_state::in0_r)
{
	// battery level?
	// bit4 -> if reset CPU keeps restarting (several unknown instructions before jumping to 0)
	//         perhaps a battery present check?
	// bit 5 -> 0 = low battery

	// --XX ---- VDET
	// ---- -X-- data-in
	return 0x30 & ~ 0x00;
}


static INPUT_PORTS_START(cfx9850)
	PORT_START("KO1")
	/* KI1 */ PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("AC On/Off") PORT_CODE(KEYCODE_BACKSLASH)
			  PORT_BIT(0xfe, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO2")
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("EXE Enter") PORT_CODE(KEYCODE_ENTER)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("(-) Ares") PORT_CODE(KEYCODE_MINUS)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("EXP Pi") PORT_CODE(KEYCODE_EQUALS)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(". SPACE") PORT_CODE(KEYCODE_SPACE)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("0   Z") PORT_CODE(KEYCODE_Z)
	PORT_BIT(0x83, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO3")
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("- ] Y") PORT_CODE(KEYCODE_Y)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("+ [ X") PORT_CODE(KEYCODE_X)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("3   W") PORT_CODE(KEYCODE_W)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("2   V") PORT_CODE(KEYCODE_V)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("1   U") PORT_CODE(KEYCODE_U)
	PORT_BIT(0x83, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO4")
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("/ } T") PORT_CODE(KEYCODE_T)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("* { S") PORT_CODE(KEYCODE_S)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("6   R") PORT_CODE(KEYCODE_R)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("5   Q") PORT_CODE(KEYCODE_Q)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("4   P") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x83, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO5")
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("DEL DG") PORT_CODE(KEYCODE_BACKSPACE)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("9   O") PORT_CODE(KEYCODE_O)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("8   N") PORT_CODE(KEYCODE_N)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("7   M") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x87, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO6")
	/* KI2 */ PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("TAB L") PORT_CODE(KEYCODE_L)
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(",   K") PORT_CODE(KEYCODE_K)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(") x^-1 J") PORT_CODE(KEYCODE_J)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("( .. I") PORT_CODE(KEYCODE_I)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F<=>0 H") PORT_CODE(KEYCODE_H)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("a b/c G") PORT_CODE(KEYCODE_G)
	PORT_BIT(0x81, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO7")
	/* KI2 */ PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("tan tan^-1 F") PORT_CODE(KEYCODE_F)
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("cos cas^-1 E") PORT_CODE(KEYCODE_E)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("sin sin^-1 D") PORT_CODE(KEYCODE_D)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ln e^x C") PORT_CODE(KEYCODE_C)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("log 10^x B") PORT_CODE(KEYCODE_B)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("x,d,t A") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x81, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO8")
	/* KI2 */ PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT)
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("EXIT QUIT") PORT_CODE(KEYCODE_STOP)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("/\\ .. ..") PORT_CODE(KEYCODE_COMMA)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("x^2 sqrt ..") PORT_CODE(KEYCODE_SLASH)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ALPHA ..-LOCK") PORT_CODE(KEYCODE_CAPSLOCK)
	PORT_BIT(0x81, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO9")
	/* KI2 */ PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP)
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("MENU SET UP") PORT_CODE(KEYCODE_OPENBRACE)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("VARS PRGM") PORT_CODE(KEYCODE_CLOSEBRACE)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("OPTN") PORT_CODE(KEYCODE_LCONTROL)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("SHIFT") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x81, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO10")
	/* KI2 */ PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F6 G<=>T") PORT_CODE(KEYCODE_F6)
	/* KI3 */ PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F5 G-Solv") PORT_CODE(KEYCODE_F5)
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F4 Sketch") PORT_CODE(KEYCODE_F4)
	/* KI5 */ PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F3 V-Window") PORT_CODE(KEYCODE_F3)
	/* KI6 */ PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F2 Zoom") PORT_CODE(KEYCODE_F2)
	/* KI7 */ PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F1 Trace") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x81, IP_ACTIVE_HIGH, IPT_UNUSED)

	// KO11 is not connected
	PORT_START("KO11")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KO12")
	/* KI4 */ PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("TEST") PORT_CODE(KEYCODE_TILDE)
	PORT_BIT(0xf7, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END


PALETTE_INIT_MEMBER(cfx9850_state, cfx9850)
{
	palette.set_pen_color(0, 0xff, 0xff, 0xff);
	palette.set_pen_color(1, 0x00, 0x00, 0xff);
	palette.set_pen_color(2, 0x00, 0xff, 0x00);
	palette.set_pen_color(3, 0xff, 0x00, 0x00);
}


u32 cfx9850_state::screen_update_cfx9850(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	u16 offset = 0;

	for (int i = 0; i < 16; i++)
	{
		int x = 120 - i * 8;

		for (int j = 0; j < 64; j++)
		{
			u8 data1 = m_display_ram[offset];
			u8 data2 = m_display_ram[offset + 0x400];

			for (int b = 0; b < 8; b++)
			{
				bitmap.pix16(63-j, x+b) = ( data1 & 0x80 ) ? ( data2 & 0x80 ? 3 : 2 ) : ( data2 & 0x80 ? 1 : 0 );
				data1 <<= 1;
				data2 <<= 1;
			}

			offset++;
		}
	}

	return 0;
}


static MACHINE_CONFIG_START(cfx9850)
	MCFG_CPU_ADD("maincpu", HCD62121, 4300000)    /* X1 - 4.3 MHz */
	MCFG_CPU_PROGRAM_MAP(cfx9850)
	MCFG_HCD62121_KOL_CB(WRITE8(cfx9850_state, kol_w))
	MCFG_HCD62121_KOH_CB(WRITE8(cfx9850_state, koh_w))
	MCFG_HCD62121_PORT_CB(WRITE8(cfx9850_state, port_w))
	MCFG_HCD62121_OPT_CB(WRITE8(cfx9850_state, opt_w))
	MCFG_HCD62121_KI_CB(READ8(cfx9850_state, ki_r))
	MCFG_HCD62121_IN0_CB(READ8(cfx9850_state, in0_r))

	MCFG_SCREEN_ADD("screen", LCD)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_SIZE(128, 64)
	MCFG_SCREEN_VISIBLE_AREA(0, 127, 0, 63)
	MCFG_SCREEN_UPDATE_DRIVER(cfx9850_state, screen_update_cfx9850)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_DEFAULT_LAYOUT(layout_lcd)

	// TODO: Verify amount of colors and palette. Colors can be changed by changing the contrast.
	MCFG_PALETTE_ADD("palette", 4)
	MCFG_PALETTE_INIT_OWNER(cfx9850_state, cfx9850)
MACHINE_CONFIG_END


ROM_START(cfx9850)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hcd62121.bin", 0x0000, 0x8000, CRC(e72075f8) SHA1(f50d176e1c225dab69abfc67702c9dfb296b6a78))

	ROM_REGION(0x80000, "bios", 0)
	// Unknown yet which rom is which version.
	ROM_SYSTEM_BIOS(0, "rom1", "rom1, version unknown")
	ROMX_LOAD("cfx9850.bin", 0x00000, 0x80000, CRC(6c9bd903) SHA1(d5b6677ab4e0d3f84e5769e89e8f3d101f98f848), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "rom2", "rom2, version unknown")
	ROMX_LOAD("cfx9850b.bin", 0x00000, 0x80000, CRC(cd3c497f) SHA1(1d1aa38205eec7aba3ed6bef7389767e38afe075), ROM_BIOS(2))
ROM_END


COMP(1996, cfx9850, 0, 0, cfx9850, cfx9850, cfx9850_state, 0, "Casio", "CFX-9850G", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
