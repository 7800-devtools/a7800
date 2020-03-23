// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/******************************************************************************
    Acorn Electron driver
    By Wilbert Pol

Hardware Overview
-----------------
The Acorn Electron is a budget version of the BBC Micro home computer
Basic specs are:
6502 CPU @ 2MHz or 1MHz
32k RAM and 32k ROM
Text modes: 20x32, 40x25, 40x32, 80x25, 80x32
Graphics modes: 160x256 (4 or 16 colors), 320x256 (2 or 4 colors), 640x256 (2 colors), 320x200 (2 colors), 640x200 (2 colors)
Colors: 8 colors (either solid or flashing)
Sound: 1 channel, 7 octaves; built-in speaker
Internal ports for cassette storage, RGB/CVBS monitors and TV output
Various expansions can be added via the rear expansion port

PCB Layout
----------                           |-------------------|
                            |--------|  EXPANSION PORT   |
|-----|---------------------|                            |---||---------|
|MOD  | SPKR     16MHz                            LS169 18VAC||         []18VAC INPUT
|     |17.7345MHz               ROM                     18VAC|| POWER   |
|-----|       74S04                             |----|       || SUPPLY  |
|          KBD_CONN                    6502     |ULA |       ||         |
|CVBS                                           |----|       ||  +5VDC  |
|RGB                  LS00 LS86                              ||  -5VDC  |
|                                                         PWR||         |
|CASS         LM324   LS00 LS86 S74 LS74 4164 4164 4164 4164 ||         |
|------------------------------------------------------------||---------|
Notes: (all IC's shown. Only 16 ICs are used)
     6502 - 6502 CPU, clock input 2.000MHz [16/8]
      ULA - Custom logic chip 12CO21, also containing most of the BBC Micro circuitry
            Early PCB revisions used a PLCC68 chip in a socket. Later revisions used a
            PGA68 chip soldered directly into the motherboard
     4164 - 4164 64k x4-bit DRAM (4 chips for 32kbytes total)
      ROM - Hitachi HN613256 32k x8-bit MASK ROM containing OS & BASIC
    LM324 - Texas Instruments LM324 Operational Amplifier
      MOD - UHF TV modulator UM1233-E36
     CVBS - Composite color video output socket
      RGB - RGB monitor video output socket
     CASS - Cassette port
      PWR - 3-pin power input from internal power supply
 KBD_CONN - 22-pin keyboard connector
     SPKR - 2-pin internal speaker connector

******************************************************************************
Emulation notes:

I don't have a real system to verify the behaviour of the emulation. The things
that can be done through BASIC programs seem to behave properly (most of the time :).

Incomplete:
    - Sound (sound is too high?)
    - Graphics (seems to be wrong for several games)
    - 1 MHz bus is not emulated
    - Bus claiming by ULA is not implemented
    - Currently the cartridge support always loads the upper rom in page 12
      and the lower rom in page 0. This might need further documentation in
      the software list and loading code.

Missing:
    - Other peripherals
******************************************************************************/

#include "emu.h"
#include "cpu/m6502/m6502.h"
#include "includes/electron.h"
#include "imagedev/cassette.h"
#include "formats/uef_cas.h"
#include "formats/csw_cas.h"
#include "sound/beep.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

static const rgb_t electron_palette[8]=
{
	rgb_t(0x0ff,0x0ff,0x0ff),
	rgb_t(0x000,0x0ff,0x0ff),
	rgb_t(0x0ff,0x000,0x0ff),
	rgb_t(0x000,0x000,0x0ff),
	rgb_t(0x0ff,0x0ff,0x000),
	rgb_t(0x000,0x0ff,0x000),
	rgb_t(0x0ff,0x000,0x000),
	rgb_t(0x000,0x000,0x000)
};

PALETTE_INIT_MEMBER(electron_state, electron)
{
	palette.set_pen_colors(0, electron_palette, ARRAY_LENGTH(electron_palette));
}

static ADDRESS_MAP_START(electron_mem, AS_PROGRAM, 8, electron_state )
	AM_RANGE(0x0000, 0x7fff) AM_READWRITE(electron_mem_r, electron_mem_w)           /* 32KB of RAM */
	AM_RANGE(0x8000, 0xbfff) AM_ROMBANK("bank2")                                    /* Banked ROM pages */
	AM_RANGE(0xc000, 0xfbff) AM_ROM AM_REGION("user1", 0x40000)                     /* OS ROM */
	AM_RANGE(0xfc00, 0xfcff) AM_READWRITE(electron_fred_r, electron_fred_w )        /* FRED */
	AM_RANGE(0xfd00, 0xfdff) AM_READWRITE(electron_jim_r, electron_jim_w )          /* JIM */
	AM_RANGE(0xfe00, 0xfeff) AM_READWRITE(electron_sheila_r, electron_sheila_w )    /* SHEILA */
	AM_RANGE(0xff00, 0xffff) AM_ROM AM_REGION("user1", 0x43f00)                     /* OS ROM continued */
ADDRESS_MAP_END

INPUT_CHANGED_MEMBER(electron_state::trigger_reset)
{
	m_maincpu->set_input_line(INPUT_LINE_RESET, newval ? ASSERT_LINE : CLEAR_LINE);
	if (newval)
	{
		m_exp->reset();
	}
}

static INPUT_PORTS_START( electron )

	PORT_START("LINE.0")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("\xE2\x86\x92 | \\") PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR(UCHAR_MAMEKEY(RIGHT)) PORT_CHAR('|') PORT_CHAR('\\') // on the real keyboard, this would be on the 1st row, the 3rd key after 0
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("COPY") PORT_CODE(KEYCODE_END)  PORT_CHAR(UCHAR_MAMEKEY(F1)) PORT_CHAR('[') PORT_CHAR(']')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)      PORT_CHAR(' ')

	PORT_START("LINE.1")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("\xE2\x86\x90 ^ ~") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR(UCHAR_MAMEKEY(LEFT)) PORT_CHAR('^') PORT_CHAR('~')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("\xE2\x86\x93 _ }") PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(UCHAR_MAMEKEY(DOWN)) PORT_CHAR('_') PORT_CHAR('}')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RETURN") PORT_CODE(KEYCODE_ENTER)     PORT_CHAR(13)
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("DELETE") PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8)

	PORT_START("LINE.2")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-') PORT_CHAR('=')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("\xE2\x86\x91 \xC2\xA3 {") PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR(UCHAR_MAMEKEY(UP)) PORT_CHAR('\xA3') PORT_CHAR('{')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)      PORT_CHAR(':') PORT_CHAR('*')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("LINE.3")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)          PORT_CHAR('0') PORT_CHAR('@')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)          PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)      PORT_CHAR(';') PORT_CHAR('+')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)      PORT_CHAR('/') PORT_CHAR('?')

	PORT_START("LINE.4")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)          PORT_CHAR('o') PORT_CHAR('O')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)          PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)       PORT_CHAR('.') PORT_CHAR('>')

	PORT_START("LINE.5")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)          PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)          PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)      PORT_CHAR(',') PORT_CHAR('<')

	PORT_START("LINE.6")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7)          PORT_CHAR('7') PORT_CHAR('\'')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)          PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)          PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)          PORT_CHAR('m') PORT_CHAR('M')

	PORT_START("LINE.7")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6)          PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)          PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)          PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)          PORT_CHAR('n') PORT_CHAR('N')

	PORT_START("LINE.8")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5)          PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)          PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)          PORT_CHAR('g') PORT_CHAR('G')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)          PORT_CHAR('b') PORT_CHAR('B')

	PORT_START("LINE.9")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4)          PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)          PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)          PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)          PORT_CHAR('v') PORT_CHAR('V')

	PORT_START("LINE.10")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3)          PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)          PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)          PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)          PORT_CHAR('c') PORT_CHAR('C')

	PORT_START("LINE.11")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)          PORT_CHAR('2') PORT_CHAR('"')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)          PORT_CHAR('w') PORT_CHAR('W')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)          PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)          PORT_CHAR('x') PORT_CHAR('X')

	PORT_START("LINE.12")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)          PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)          PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)          PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)          PORT_CHAR('z') PORT_CHAR('Z')

	PORT_START("LINE.13")
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ESCAPE") PORT_CODE(KEYCODE_ESC) PORT_CHAR(UCHAR_MAMEKEY(ESC))
	PORT_BIT(0x02,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("FUNC") PORT_CODE(KEYCODE_TAB) PORT_CHAR(UCHAR_MAMEKEY(LALT))
	PORT_BIT(0x04,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x08,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RSHIFT) PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1)

	PORT_START("BRK")       /* BREAK */
	PORT_BIT(0x01,  IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("BREAK") PORT_CODE(KEYCODE_F12) PORT_CHAR(UCHAR_MAMEKEY(F12)) PORT_CHANGED_MEMBER(DEVICE_SELF, electron_state, trigger_reset, 0)
INPUT_PORTS_END

static MACHINE_CONFIG_START( electron )
	MCFG_CPU_ADD( "maincpu", M6502, XTAL_16MHz/8 )
	MCFG_CPU_PROGRAM_MAP( electron_mem )

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE( 50.08 )
	MCFG_SCREEN_SIZE( 640, 312 )
	MCFG_SCREEN_VISIBLE_AREA( 0, 640-1, 0, 256-1 )
	MCFG_SCREEN_UPDATE_DRIVER(electron_state, screen_update_electron)
	MCFG_SCREEN_VIDEO_ATTRIBUTES(VIDEO_UPDATE_SCANLINE)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD( "palette", 16 )
	MCFG_PALETTE_INIT_OWNER(electron_state, electron)

	MCFG_SPEAKER_STANDARD_MONO( "mono" )
	MCFG_SOUND_ADD( "beeper", BEEP, 300 )
	MCFG_SOUND_ROUTE( ALL_OUTPUTS, "mono", 1.00 )

	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("32K")

	MCFG_CASSETTE_ADD( "cassette" )
	MCFG_CASSETTE_FORMATS(bbc_cassette_formats)
	MCFG_CASSETTE_DEFAULT_STATE(CASSETTE_PLAY)
	MCFG_CASSETTE_INTERFACE("electron_cass")

	MCFG_GENERIC_CARTSLOT_ADD("cartslot", generic_plain_slot, "electron_cart")
	MCFG_GENERIC_LOAD(electron_state, electron_cart)

	/* expansion port */
	MCFG_ELECTRON_EXPANSION_SLOT_ADD("exp", electron_expansion_devices, "plus3", false)
	MCFG_ELECTRON_EXPANSION_SLOT_IRQ_HANDLER(INPUTLINE("maincpu", M6502_IRQ_LINE))
	MCFG_ELECTRON_EXPANSION_SLOT_NMI_HANDLER(INPUTLINE("maincpu", M6502_NMI_LINE))

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("cass_list", "electron_cass")
	MCFG_SOFTWARE_LIST_ADD("cart_list", "electron_cart")
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( btm2105, electron )
	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_COLOR(rgb_t::amber())

	/* expansion port */
	MCFG_DEVICE_MODIFY("exp")
	MCFG_DEVICE_SLOT_INTERFACE(electron_expansion_devices, "m2105", true)

	/* software lists */
	MCFG_SOFTWARE_LIST_REMOVE("cass_list")
MACHINE_CONFIG_END


/* Electron Rom Load */
ROM_START(electron)
	ROM_REGION( 0x44000, "user1", 0 ) /* OS Rom */
	ROM_LOAD( "os.rom", 0x40000, 0x4000, CRC(bf63fb1f) SHA1(a48b8fa0cfb09140e808ac8a187316c605a0b32e) ) /* OS rom */
	/* 00000  0 Second external socket on the expansion module (SK2) */
	/* 04000  1 Second external socket on the expansion module (SK2) */
	/* 08000  2 First external socket on the expansion module (SK1)  */
	/* 0c000  3 First external socket on the expansion module (SK1)  */
	/* 10000  4 Disc                                                 */
	/* 14000  5 USER applications                                    */
	/* 18000  6 USER applications                                    */
	/* 1c000  7 Modem interface ROM                                  */
	/* 20000  8 Keyboard                                             */
	/* 24000  9 Keyboard mirror                                      */
	/* 28000 10 BASIC rom                                            */
	/* 2c000 11 BASIC rom mirror                                     */
	/* 30000 12 Expansion module operating system                    */
	/* 34000 13 High priority slot in expansion module               */
	/* 38000 14 ECONET                                               */
	/* 3c000 15 Reserved                                             */
		ROM_LOAD("basic.rom", 0x28000, 0x4000, CRC(79434781) SHA1(4a7393f3a45ea309f744441c16723e2ef447a281))
		ROM_COPY("user1", 0x28000, 0x2c000, 0x4000)
ROM_END


#define rom_btm2105 rom_electron


/*     YEAR  NAME       PARENT    COMPAT  MACHINE   INPUT     CLASS           INIT  COMPANY                             FULLNAME           FLAGS */
COMP ( 1983, electron,  0,        0,      electron, electron, electron_state, 0,    "Acorn",                            "Acorn Electron",  MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )
COMP ( 1985, btm2105,   electron, 0,      btm2105,  electron, electron_state, 0,    "British Telecom Business Systems", "BT Merlin M2105", MACHINE_NOT_WORKING )
