// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom
/***************************************************************************
 *
 *  KRON K-180 VGA terminal server
 *
 *  27/10/2015
 *
 * I baught this hardware on Ebay to have something with a Z80 CPU to play with.
 * The hardware is a serial terminal controller with VGA output and a PC keyboard
 * and was manufactured mid 90:ies by a company from Vinnitsa,Ukraine called KRON.
 * There is a character generator with support for both western and cyrilic characters.
 * The PCB is also filled with chips with cyrrilic characters on but thanks to this
 * page I managed to translate most of them into western TTL logic names:
 *
 * - http://ganswijk.home.xs4all.nl/chipdir/soviet/
 * - https://frakaday.blogspot.se/p/kron180.html - schematics and more info
 *
 * +-----||||||||||||||||||||-|||||||||||----|||||||||||||||||||-|||||||||-||||||||+
 * |     |     RS232C       | |Serial PN|    | CENTRONICS PN   | | VGA   | |KEYBRD||
 * |     +----------------XP1 +-------XP2    +---------------XS3 +-----XS2 +----XS1|
 * |                      +---DD27 +-----DD20 +-------DD34   +----DD3 +-------DD33 |
 * |                      | 1488|  |CD75189|  | 74299   |    | 7407 | | 74244.1 |  |
 * |                      +-----+  +-------+  +---------+    +------+ +---------+  |
 * |                                           +------DD17   +----DD9              |
 * |                               +-----DD35  | 74670   |   | 7403 |              |
 * |     +--------------------DD10 | 7474  |  ++------DD12  ++----DD14  +-----DD13 |
 * |     |   Z8018006PSC         | +-------+  | 74374    |  | 74151 |   | 74174 |  |
 * |     |   Z180 MPU            |            +----------+  +-------+   +-------+  |
 * |     +-----------------------+ +-----DD36                           +------DD6 |
 * |                               |7432.1 |  +-------DD2    +----DD7   | 7474  |  |
 * |                               +-------+  | 74374   |    |7474.1|   +-------+  |
 * |                                          +---------+    +------+              |
 * |                              +------DD15   +-----DD1    +----DD19             |
 * |                              | 74138  |    | 74166 |    |7474.2|              |
 * |+-------BQ1 +-------------DD4 +--------+    +-------+    +------+              |
 * ||XTAL     | |               |  +-----DD16   +-----DD18      +-DD28  ..-^-..    |
 * ||12.280MHz| | NM27C512      |  | 7432  |    | 74670 |   93C46CB1| /         \  |
 * |+---------+ +---------------+  +-----DD37   +-------+   EEPROM--+/  Beeper   \ |
 * |+------DD26                    | 7400  |     +----DD32  +----DD23|     O     | |
 * || 74299   |                    +-------+     | 7474  |  | 74259 |\   BQ2     / |
 * |+---------+  +------------DD5                +-------+  +-------+ \         /  |
 * |+---------+  | HY6264A      |  +-----DD8     +----DD31   +---DD25  ''--_--''   |
 * || 74374   |  | 8Kb SRAM     |  | 7408  |     | 7474  |   | 7414 |           +XS4
 * |+------DD24  +--------------+  +-------+     +-------+   +------+           |P |
 * |                DD30-------+   +-----DD29 +---------BQ3  +---DD11           |W |
 * |                 | 74244   |   | 74393 |  |XTL 29.3MHz|  | 7404 |           |R |
 * +-----------------+---------+---+-------+--+-----------+--+------+--------------+
 *
 * 74299 - 8 bit bidirectional universal shift/storage register
 * 7407  - 6 open collector drivers
 * 74244 - Octal buffers/line drivers/line receivers
 * 74670 - 4 x 4 register file with 3-state outputs
 * 74151 - 1 of 8 demultiplexor
 * 74374 - octal D type flip flops
 * 74174 - 6 D type  flip flops with clear
 * 75189 - quadruple line receivers
 * 74395 - 4 bit universal shift registers
 * 74393 - dual 4 bit counters
 * 74259 - 8 bit adressable latches
 * 74166 - 8 bit shift register
 *
 *
 * Keyboard interface
 * ------------------
 * XT: 1 start bit + 8 data bits @ ~2kbps
 * AT: 1 start bit + 8 data bits + 1 odd parity bit @ ~10kbps
 *
 * Pin 1 CLK/CTS -
 * Pin 2 RxD   (AT:+ TxD/RTS) - via R to GND + Pin 4 (out) 7407 + pin 11 (DS0) 74299 8 bit shift register
 *       u                                 Pin 3 (in)  7407 + pin 11 (INT2) CPU + pin 1 74299
 *  1         3                            + pin 4 (*PRE) 7474.2 + pin 6 (*Q) 7474.2
 *    4  2  5                              Pin 3 (CLK) 7474.2 + pin 8 (4Y) 7414
 *                                         Pin 2 (D) 7474.2 + Pin 17 (Q7 serial out) 74299
 * Pin 3 Reset (AT:+ NC)
 * Pin 4 GND                  - GND
 * Pin 5 +5v                  - VCC
 *
 * Identified chips
 * -----------------
 * Z180 MPU (Z8018006PCS)
 * NM27C512Q 64Kb EPROM, but A15 tied to +5v and schematics supports only 27256
 * HY6264A 8Kb SRAM
 * 93C46B1 128 bytes EEPROM
 *
 * Misc findings
 * --------------
 * - $17B9 might be keyboard input routine
 * - indentified used OUT ports: $00, $02, $04, $07, $08, $0A, $0C, $0E, $0F, $40, $60
 * - identified used IN ports: $10 (keyboard?), $30
 * - screen memory at 0x8600
 * - each position has 2 bytes <character> + <mode>
 * - mode 0x08 is double height
 * - characters seems to follow IBM PC Code page 437 for opening screen
 * - terminal defaults to cyrillic characterset possibly due to setting in EEPROM
 * - http://www.phantom.sannata.ru/forum/index.php?t=5200 - Kron-2 for sale
 * - http://f-picture.net/fp/3b2a0496b981437a9c3f90ed236363c9 - Picture of Kron-2
 * - http://www.kron.com.ua/
 */

#include "emu.h"
#include "cpu/z180/z180.h"
#include "machine/pckeybrd.h"
#include "screen.h"

#define VERBOSE 2

#define LOGPRINT(x) do { if (VERBOSE) logerror x; } while (0)
#define LOG(x) {}
#define LOGIO(x) {}
#define LOGSCAN(x) {}
#define LOGSCREEN(x) {}
#define LOGKBD(x) LOGPRINT(x)
#define RLOG(x) {}
#define LOGCS(x) {}

#if VERBOSE >= 2
#define logerror printf
#endif

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

class kron180_state : public driver_device
{
public:
kron180_state(const machine_config &mconfig, device_type type, const char *tag) :
	driver_device (mconfig, type, tag)
	,m_maincpu (*this, "maincpu")
	,m_videoram(*this, "videoram")
	,m_keyboard(*this, "pc_keyboard")
	{ }
	uint8_t *m_char_ptr;
	uint8_t *m_vram;
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(keyb_interrupt);
	DECLARE_WRITE8_MEMBER( sn74259_w ){     LOGIO(("%s %02x = %02x\n", FUNCNAME, offset & 0x07, offset & 0x08 ? 1 : 0)); }
	DECLARE_WRITE8_MEMBER( ap5_w ){         LOGIO(("%s %02x = %02x\n", FUNCNAME, offset, data)); }
	DECLARE_READ8_MEMBER( ap5_r ){          LOGIO(("%s() %02x = %02x\n",  FUNCNAME, offset, 1)); return 1; }
	DECLARE_WRITE8_MEMBER( wkb_w ){         LOGIO(("%s %02x = %02x\n", FUNCNAME, offset, data)); }
	DECLARE_WRITE8_MEMBER( sn74299_w ){     LOGIO(("%s %02x = %02x\n", FUNCNAME, offset, data)); }
	DECLARE_READ8_MEMBER( sn74299_r ){      LOGIO(("%s() %02x = %02x\n", FUNCNAME, offset, 1)); return 1; }
	DECLARE_WRITE8_MEMBER( txen_w ){        LOGIO(("%s %02x = %02x\n", FUNCNAME, offset, data)); }
	DECLARE_WRITE8_MEMBER( kbd_reset_w ){   LOGIO(("%s %02x = %02x\n", FUNCNAME, offset, data)); }
	DECLARE_WRITE8_MEMBER( dreq_w ){        LOGIO(("%s %02x = %02x\n", FUNCNAME, offset, data)); }
protected:
	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_videoram;
	required_device<pc_keyboard_device> m_keyboard;
	uint8_t m_kbd_data;
private:
	virtual void machine_start () override;
};

static ADDRESS_MAP_START (kron180_mem, AS_PROGRAM, 8, kron180_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE (0x0000, 0x7fff) AM_ROM AM_REGION("roms", 0x8000)
	AM_RANGE (0x8000, 0x9fff) AM_RAM AM_MIRROR(0x6000)
	AM_RANGE (0x8600, 0x95ff) AM_RAM AM_SHARE("videoram")
ADDRESS_MAP_END

/*   IO decoding
 *
 *   A7 A6 A5 A4 A3 A2 A1 A0
 *    0  x  x  x  x  x  x  x - 74138 selected
 *    0  0  0  0  x  x  x  x - 74259 selected
 *    0  0  0  0  D  0  0  0 - EEPROM Data In
 *    0  0  0  0  D  0  0  1 - COMDTR
 *    0  0  0  0  D  0  1  0 - EEPROM CS
 *    0  0  0  0  D  0  1  1 - Vertical Sync
 *    0  0  0  0  D  1  0  0 - TBRQ and BLANK set
 *    0  0  0  0  D  1  0  1 - BLINK
 *    0  0  0  0  D  1  1  0 - CSTR
 *    0  0  0  0  D  1  1  1 - EEPROM CLK
 *    0  0  0  1  x  x  x  x - AP5
 *    0  0  1  0  x  x  x  x - WKB
 *    0  0  1  1  x  x  x  x - 74299 G1+G2
 *    0  1  0  0  x  x  x  x - Reset of 74299
 *    0  1  0  1  x  x  x  x - Enable TX?
 *    0  1  1  0  x  x  x  x - Reset KBD
 *    0  1  1  1  x  x  x  x - DKA/DREQ0 Z180 = D0
 *
 * Now, in paralell there is alot of stuff going on in the upper I/O address lines
 * they are driving the character generator and some other signals
 *  A19 - not available on the DIP64 package
 *  A18 - multiplexed pin used as Tout pulsing the VT1 signal
 *  A17 - WA on DD17 (74670) and DD18 (74670)
 *  A16 - WB on DD 17 (74670) and DD18 (74670)
 *
 * At the moment we emulate the screen at a high level so I just disregard the special functions on A16 - A18 by mirroring the mapping below
 */
static ADDRESS_MAP_START( kron180_iomap, AS_IO, 8, kron180_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE( 0x0000, 0x000f ) AM_WRITE(sn74259_w)
	AM_RANGE( 0x0010, 0x001f ) AM_READWRITE(ap5_r, ap5_w)
	AM_RANGE( 0x0020, 0x002f ) AM_WRITE(wkb_w)
	AM_RANGE( 0x0030, 0x003f ) AM_READ(sn74299_r)
	AM_RANGE( 0x0040, 0x004f ) AM_WRITE(sn74299_w)
	AM_RANGE( 0x0050, 0x005f ) AM_WRITE(txen_w)
	AM_RANGE( 0x0060, 0x006f ) AM_WRITE(kbd_reset_w)
	AM_RANGE( 0x0070, 0x007f ) AM_WRITE(dreq_w)
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START (kron180)
	PORT_INCLUDE(pc_keyboard)
INPUT_PORTS_END

/* Video TODO: find and understand the char table within main rom */
uint32_t kron180_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int x, y;
	int vramad;
	uint8_t *chardata;
	uint8_t charcode;

	LOGSCREEN(("%s()\n", FUNCNAME));
	vramad = 0;
	for (int row = 0; row < 25 * 8; row += 8)
	{
		for (int col = 0; col < 80 * 8; col += 8)
		{
			/* look up the character data */
			charcode = m_vram[vramad];
			if (VERBOSE && charcode != 0x20 && charcode != 0) LOGSCREEN(("\n %c at X=%d Y=%d: ", charcode, col, row));
			chardata = &m_char_ptr[(charcode * 8) + 8];
			/* plot the character */
			for (y = 0; y < 8; y++)
			{
				chardata--;
				if (VERBOSE && charcode != 0x20 && charcode != 0) LOGSCREEN(("\n  %02x: ", *chardata));
				for (x = 0; x < 8; x++)
				{
					if (VERBOSE && charcode != 0x20 && charcode != 0) LOGSCREEN((" %02x: ", *chardata));
					bitmap.pix16(row + (8 - y), col + (8 - x)) = (*chardata & (1 << x)) ? 1 : 0;
				}
			}
			vramad += 2;
		}
		if (VERBOSE && charcode != 0x20 && charcode != 0) LOGSCREEN(("\n"));
		vramad += 96; // Each row is aligned at a 128 byte boundary
	}

	return 0;
}

/* Start it up */
void kron180_state::machine_start ()
{
	LOG(("%s()\n", FUNCNAME));
	m_char_ptr  = memregion("chargen")->base();
	m_vram      = (uint8_t *)m_videoram.target();

	/* register for state saving */
	save_pointer (NAME (m_char_ptr), sizeof(m_char_ptr));
	save_pointer (NAME (m_vram), sizeof(m_vram));
}

/* Interrupt Handling */
#if 0
WRITE8_MEMBER(kron180_state::irq0_ack_w)
{
	m_irq0_ack = data;
	if ((data & 1) == 1)
		m_maincpu->set_input_line(0, CLEAR_LINE);
}

INTERRUPT_GEN_MEMBER(kron180_state::interrupt)
{
	if ((m_irq0_ack & 1) == 1)
	{
		device.execute().set_input_line(0, ASSERT_LINE);
	}
}
#endif

WRITE_LINE_MEMBER(kron180_state::keyb_interrupt)
{
	if(state && (m_kbd_data = m_keyboard->read(machine().dummy_space(), 0)))
	{
		LOGKBD(("%s(%02x)\n", FUNCNAME, m_kbd_data));
		m_maincpu->set_input_line(2, ASSERT_LINE);
		/* TODO: store and present this to K180 in a good way. */
	}
}

/*
 * Machine configuration
 */
static MACHINE_CONFIG_START (kron180)
	/* basic machine hardware */
	MCFG_CPU_ADD ("maincpu", Z180, XTAL_12_288MHz)
	MCFG_CPU_PROGRAM_MAP (kron180_mem)
	MCFG_CPU_IO_MAP(kron180_iomap)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_COLOR(rgb_t::green())
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_UPDATE_DRIVER(kron180_state, screen_update)
	MCFG_SCREEN_SIZE(80 * 10, 24 * 10)
	MCFG_SCREEN_VISIBLE_AREA(0, 639, 0, 199) // TODO: This need to be fixed once the real chartable is used...
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD_MONOCHROME("palette")

	/* keyboard TODO: fix it, doesn't work yet */
	MCFG_PC_KEYB_ADD("pc_keyboard", WRITELINE(kron180_state, keyb_interrupt))
MACHINE_CONFIG_END

/* ROM definitions */
ROM_START (kron180)
	ROM_REGION(0x10000, "roms", 0)
	ROM_LOAD ("k180DD4-2.8M.bin", 0x000000, 0x10000, CRC (ae0642ad) SHA1 (2c53a714de6af4b64e46fcd34bca6d4438511765))

	ROM_REGION(0x1000, "chargen",0) /* TODO: This character rom is taken from ibmjr rom set and will be replaced */
	ROM_LOAD( "cga.chr", 0x0000, 0x1000, CRC(42009069) SHA1(ed08559ce2d7f97f68b9f540bddad5b6295294dd) )
ROM_END

/* Driver */
//     YEAR  NAME          PARENT  COMPAT   MACHINE         INPUT     CLASS         INIT  COMPANY        FULLNAME      FLAGS
COMP ( 1995, kron180,      0,      0,       kron180,        kron180, kron180_state, 0,    "Kron Ltd",    "Kron K-180", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
