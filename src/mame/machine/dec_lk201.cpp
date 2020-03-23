// license:BSD-3-Clause
// copyright-holders:R. Belmont, M. Burke
/*
    DEC LK-201 keyboard
    Emulation by R. Belmont & M. Burke
with contributions by Cracyc and Karl-Ludwig Deisenhofer (2016)

This is the later "cost-reduced" 6805 version with green LEDs; there's also an 8048 version.
    The LK-201 mechanical elements are described in US Patent 4,467,150
*/

/* LK201-AA keyboard matrix (8048 version with updates)
   Source: VCB02 Technical Reference.

   KBD controller scan matrix (PORT 1): 8 x BCD IN => 18 DECIMAL OUT

   Keyboard itself:
   18 x IN (KEYBOARD DRIVE) KBD 17... KBD 0 =>
   8 OUT (keyboard data @ D7..D0)

   to => PORT 0 @ KBD controller.

________|D7  |D6  |D5  |D4 |D3 |D2 |D1 |D0
..KBD17:|[R] |F19 |[R] |F20|PF4|N- | N,| Enter (numpad)
........|    |    |    |   |   |   NOTE1)
........|    |G22 |    |G23|E23|D23|C23| A23
--------|----|----|----|---|---|---|---|---
..KBD16:|F18 |PF3 |[R] |N9 |C:D|N6 |N3 |N.
........|G21 |E22 |    |D22|B17|C22|B22|A22
--------|----|----|----|---|---|---|---|---
..KBD15:|F17 |PF2 |[R] |N8 |N5 |C:R| N2|N0 (right)
........|    |    |    |   |   |   |   |see NOTE 2)
........|G20 |E21 |    |D21|C21|B18|B21|A21
--------|----|----|----|---|---|---|---|---
  KBD14:|PF1 |Next|Rem-|C:U|N7 |N4 |N1 |N0 (left)
........|    |Scrn|move|...|   |   |   |see NOTE 2)
........|E20 |D18 |E18 |C17|D20|C20|B20|A20
--------|----|----|----|---|---|---|---|---
..KBD13:|Ins.|'_' |'Do'|Prev { |"  |[R]|[R]
........|Here|'-' |    |Scrn [ |'  |   |
........|E17 |E11 |G16 |D17|D11|C11|B13|A17
--------|----|----|----|---|---|---|---|---
..KBD12:|Find|+   |Help|Se-| } |Re-|C:L| |
........|    |=   |    |lect ] |turn...| \
........|E16 |E12 |G15 |D16 D12|C13|B16|C12
--------|----|----|----|---|---|---|---|---
..KBD11:Addtnl <X||[R] |)  |P  NOTE|:  | ?
.......Options Del|    |0  |   | 3)|;  | /
........|G14 | E13|....|E10|D10|...|C10|B10
--------|----|----|----|---|---|---|---|---
..KBD10:|[R] |F12 |[R] |F13| ( |O  |L  | .
........|....|(BS)|    |(LF) 9 |   |   | .
........|....|G12 |....|G13|E09|D09|C09|B09
--------|----|----|----|---|---|---|---|---
..KBD_9:|[R] |F11 |[R] |[R]|*  |I  |K  | ,
........|....|ESC |    |   |8  |   |   | ,
........|....|G11 |....|...|E08|D08|C08|B08
--------|----|----|----|---|---|---|---|---
..KBD_8:|[R] |Main|[R] Exit|&  |U  |J  |M
........|    |Scrn|    |   |7  |   |   |
........|    |G08 |    |G09|E07|D07|C07|B07
--------|----|----|----|---|---|---|---|---
..KBD_7:|[R] Cancel[R] Resu ^  |Y  |H  |N
........|....|....|.....me |6  |   |   |
........|....|G07 |....|G06|E06|D06|C06|B06
--------|----|----|----|---|---|---|---|---
..KBD_6:|[R] |[R] |[R] Inter % |T  |G  |B
........|....|....|....rupt| 5 |   |   |
........|....|....|....|G05|E05|D05|C05|B05
--------|----|----|----|---|---|---|---|---
..KBD_5: F4  |Break [R]|$  |R  |F  |V  |Space
........|....|....|....|4  |   |   |   |
........ G02 |G03 |....|E04 D04 C04 B04 A01-A09
--------|----|----|----|---|---|---|---|---
..KBD_4: [R] |Prt.|[R] |Set|#  |E  |D  |C
........|....|Scrn|....|-Up|3  |   |   |
........|....|G00 |....|G01 E03 D03 C03 B03
--------|----|----|----|---|---|---|---|---
..KBD_3: Hold| @  |[R] |Tab|W  |S  |X  |>
........|Scrn| 2  |....|   |   |   |   |<
........|G99 |E02 |....|D00|D02|C02|B02|B00
--------|----|----|----|---|---|---|---|---
..KBD_2: [R] |[R] |[R] |~  |!  |Q  |A  |Z
........|..............|...|1
........|..............|E00 E01 D01 C01 B01
--------|----|----|----|---|---|---|---|---
..KBD_1: Ctrl|Lock|Comp|[R]
........|C99 |C00 |A99 |
--------|----|----|----|---|---|---|---|---
..KBD_0: Shift
........|B99,B11

---
  [R] = Reserved
  NOTE 1) N0-N9, N-, N. and N, refer to numeric keypad
  NOTE 2) N0 can be divided into 2 keys.
   Normally only the N0 keyswitch is implemented as a double-sized key.
   A21 = N0 (right); A20 = N0 (left)
  NOTE 3) Return key occupies 2 positions that are
   decoded as the Return (C13) key.  C13 = RETURN (bot.); D13 = RETURN (top)

  C:D - Cursor down (B17)
  C:U - Cursor up (C17)
  C:R - Cursor right (B18)
 */

#include "emu.h"
#include "dec_lk201.h"
#include "cpu/m6805/m6805.h"
#include "speaker.h"

//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define LK201_CPU_TAG   "lk201_cpu"
#define LK201_SPK_TAG   "beeper"

//-------------------------------------------------
//  SERIAL COMMUNICATIONS INTERFACE
//-------------------------------------------------

#define SCI_BAUD        0                               // Baud rate register
#define BAUD_SCR        0x07                            // SCI baud rate select
#define BAUD_SCP        0x30                            // SCI prescaler select

#define SCI_SCCR1       1                               // Control register 1
#define SCCR1_WAKE      0x08                            // Wakeup method
#define SCCR1_M         0x10                            // Character length
#define SCCR1_T8        0x40                            // Transmit bit 8
#define SCCR1_R8        0x80                            // Receive bit 8

#define SCI_SCCR2       2                               // Control register 2
#define SCCR2_SBK       0x01                            // Send break
#define SCCR2_RWU       0x02                            // Receiver wakeup enable
#define SCCR2_RE        0x04                            // Receiver enable
#define SCCR2_TE        0x08                            // Transmitter enable
#define SCCR2_ILIE      0x10                            // Idle line interrupt enable
#define SCCR2_RIE       0x20                            // Receiver interrupt enable
#define SCCR2_TCIE      0x40                            // Transmit complete interrupt enable
#define SCCR2_TIE       0x80                            // Transmit interrupt enable

#define SCI_SCSR        3                               // Status register
#define SCSR_FE         0x02                            // Receiver framing error
#define SCSR_NF         0x04                            // Receiver noise
#define SCSR_OR         0x08                            // Receiver overrun
#define SCSR_IDLE       0x10                            // Receiver idle
#define SCSR_RDRF       0x20                            // Receive data register full
#define SCSR_TC         0x40                            // Transmit complete
#define SCSR_TDRE       0x80                            // Transmit data register empty
#define SCSR_INT        (SCSR_IDLE | SCSR_RDRF| \
							SCSR_TC | SCSR_TDRE)        // Interrupt sources

#define SCI_SCDR        4                               // Data register

//-------------------------------------------------
//  SERIAL PERIPHERAL INTERFACE
//-------------------------------------------------

#define SPI_SPCR        0                               // Control register
#define SPCR_SPR        0x03                            // SPI clock rate select
#define SPCR_CPHA       0x04                            // Clock phase
#define SPCR_CPOL       0x08                            // Clock polarity
#define SPCR_MSTR       0x10                            // Master mode select
#define SPCR_DWOM       0x20                            // Port D wire-or mode option
#define SPCR_SPE        0x40                            // Serial peripheral system enable
#define SPCR_SPIE       0x80                            // Serial peripheral interrupt enable

#define SPI_SPSR        1                               // Status register
#define SPSR_MODF       0x10                            // Mode fault flag
#define SPSR_WCOL       0x40                            // Write collision
#define SPSR_SPIF       0x80                            // SPI transfer complete

#define SPI_SPDR        2                               // Data I/O Register

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(LK201, lk201_device, "lk201", "DEC LK201 Keyboard")

ROM_START( lk201 )
	ROM_REGION(0x2000, LK201_CPU_TAG, 0)

//  23-001s9-00.bin is for the newer LK201 version (green LEDs, Motorola 6805)
	ROM_LOAD( "23-001s9-00.bin", 0x0000, 0x2000, CRC(be293c51) SHA1(a11ae004d2d6055d7279da3560c3e56610a19fdb) )

//  23-004M1 or 23-004M2 are in the older LK201 keyboard with red LEDs (8051 or 8051H respectively, same code on both?)
	ROM_REGION( 0x1000, "i8051cpu", 0 )
	ROM_LOAD( "23-004m2.8051h.e1", 0x0000, 0x1000, CRC(35dd04c6) SHA1(9744c561d76fe85afaccc7a2485ce2865236873a) )

ROM_END

//-------------------------------------------------
//  ADDRESS_MAP
//-------------------------------------------------

static ADDRESS_MAP_START( lk201_map, AS_PROGRAM, 8, lk201_device )
	AM_RANGE(0x0000, 0x0002) AM_READWRITE(ports_r, ports_w)
	AM_RANGE(0x0004, 0x0006) AM_READWRITE(ddr_r, ddr_w)
	AM_RANGE(0x000a, 0x000c) AM_READWRITE(spi_r, spi_w)
	AM_RANGE(0x000d, 0x0011) AM_READWRITE(sci_r, sci_w)
	AM_RANGE(0x0012, 0x001b) AM_READWRITE(timer_r, timer_w)
	AM_RANGE(0x0050, 0x00ff) AM_RAM
	AM_RANGE(0x0100, 0x1fff) AM_ROM AM_REGION(LK201_CPU_TAG, 0x100)
ADDRESS_MAP_END


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( lk201_device::device_add_mconfig )
	MCFG_CPU_ADD(LK201_CPU_TAG, M68HC05EG, XTAL_4MHz) // actually 68HC05C4, clock verified by Lord_Nightmare
	MCFG_CPU_PROGRAM_MAP(lk201_map)

	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD(LK201_SPK_TAG, BEEP, 2000) // clocked by a 555 timer at E8, the volume of the beep is controllable by: (8051 model) P2.0 thru P2.3, or (6805 model) the upper 4 bits of the LED data latch
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

const tiny_rom_entry *lk201_device::device_rom_region() const
{
	return ROM_NAME( lk201 );
}

//-------------------------------------------------
//  INPUT_PORTS( lk201 )
//-------------------------------------------------

/* [Foreign language caps] are referenced in software titles. Please leave them in.

   DEC omitted terms like 'Interrupt', 'Break' and 'Data / Talk' on some keyboards,
   so Fn numbers are definitely important for end users.

   === CURRENT SPECIAL KEYS ===
   [PC-AT] ......=> [DEC]
   LEFT CONTROL..=> Control
   LEFT ALT .....=> Compose

   RIGHT ALT ....=> Help
   RIGHT CONTROL => Do
   ==============================================================================================
   === (PC - AT ) keys above cursor block ===
   * KEYCODE_INSERT * KEYCODE_HOME * KEYCODE_PGUP
   * KEYCODE_DEL... * KEYCODE_END  * KEYCODE_PGDN

   === (DEC LK 201 layout above cursor) ===
   * Find   ........| Insert Here  | Remove
   * Select.........| Prev   ..... | Next
   ==============================================================================================
   === CURRENT NUM PAD ASSIGNMENTS ===
   [PF1] to [PF4] are mapped to NUM LOCK, SLASH etc. (=> 4 keys on top on num pad).
   Num pad '+' gives         ',' on the DEC.
           ',' translates to '.' (=> more or less the layout of model 'LK-201-AG')

   Switch between 'full' and 'partial keyboard emulation' with Scroll Lock.
*/

INPUT_PORTS_START( lk201 )

/*
Actual membrane part number (from later 6805-based LK201 keyboard): 54-15172

Matrix Rows to bit translation, from schematic page 20 ( http://bitsavers.trailing-edge.com/pdf/dec/terminal/lk201/MP01395_LK201_Schematic_Oct83.pdf )
Bit D0 - Column P3-3
    D1 - Column P3-7
    D2 - Column P3-8
    D3 - Column P3-12
    D4 - Column P2-8
    D5 - Column P1-2
    D6 - Column P3-11
    D7 - Column P1-3

Keyboard Matrix
---------------
Row select
|         Columns SwitchID by bit                    Columns Key by bit
|         D5   D7   D4   D0   D1   D2   D3   D6      D5   D7   D4   D0   D1   D2   D3   D6
V         V    V    V    V    V    V    V    V       V    V    V    V    V    V    V    V
P1-6      ---  E99  E00  B01  C01  D01  E01  D99     ---  N/C  `    Z    A    Q    1    N/C
P1-7      ---  G99  D00  B00  B02  C02  D02  E02     ---  F1   TAB  <>   X    S    W    2
P1-9      ---  ---  G01  B03  C03  D03  E03  G00     ---  ---  F3   C    D    E    3    F2
P1-10     A99  C99  A10  ---  ---  ---  ---  C00     COMP CTRL N/C  ---  ---  ---  ---  CAPS
P1-11     ---  G02  E04 A0406 B04  C04  D04  G03     ---  F4   4   SPACE V    F    R    F5
P2-2      ---  ---  G05  B05  C05  D05  E05  G04     ---  ---  F6   B    G    T    5    N/C
P2-3      ---  ---  G06  B06  C06  D06  E06  G07     ---  ---  F7   N    H    Y    6    F8
P2-4      ---  ---  G09  B07  C07  D07  E07  G08     ---  ---  F10  M    J    U    7    F9
P2-5      --- B9911 ---  ---  ---  ---  ---  ---     ---  SHFT ---  ---  ---  ---  ---  ---
P2-6      ---  ---  G10  B08  C08  D08  E08  G11     ---  ---  N/C  ,    K    I    8    F11
P2-9      ---  ---  G13  B09  C09  D09  E09  G12     ---  ---  F13  .    L    O    9    F12
P2-10     ---  G14  E10  B10  C10  D13  D10  E13     ---  F14  0    /    ;    N/C  P    BKSP(DEL)
P2-11     G15  E16  D16  C12  B16  C13  D12  E12     HELP FIND SLCT \    LEFT RETN ]    =
P3-2      G16  E17  D17  A17  B13  C11  D11  E11     DO   INST PREV N/C  N/C  '    [    -
P3-4      E18  E20  C17  A20  B20  C20  D20  D18     RMVE PF1  UP   k0   k1   k4   k7   NEXT
P3-5      ---  G20  D21  A21  B21  B18  C21  E21     ---  F17  k8   N/C  k2   RGHT k5   PF2
P3-6      ---  G21  D22  A22  B22  C22  B17  E22     ---  F18  k9   k.   k3   k6   DOWN PF3
P3-9      ---  ---  G23  A23  C23  D23  E23  G22     ---  ---  F20  kRTN k,   k-   PF4  F19

--- = No matrix switch at all
N/C = switch present, but officially unused?
    */

	PORT_START("KBD0") // Row P2-5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_CHAR(UCHAR_SHIFT_1) // B99 and B11

	PORT_START("KBD1") // Row P1-10
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED ) // A10, exists but no key above this position (right compose?)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Compose") PORT_CODE(KEYCODE_LALT) PORT_CHAR(UCHAR_MAMEKEY(LALT)) // A99
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Caps Lock") PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK)) // C00, does not toggle/physically lock
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Ctrl") PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_MAMEKEY(LCONTROL)) // C99

	PORT_START("KBD2") // Row P1-6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Z") PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z') // B01
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("A") PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A') // C01
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Q") PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q') // D01
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("1") PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!') // E01
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Tilde") PORT_CODE(KEYCODE_TILDE) PORT_CHAR('`') PORT_CHAR('~') // E00
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED ) // D99, exists but no key above this position
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED ) // E99, exists but no key above this position

	PORT_START("KBD3") // Row P1-7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("<") PORT_CODE(KEYCODE_BACKSLASH2) // B00
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("X") PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X') // B02
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("S") PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S') // C02
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("W") PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W') // D02
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Tab") PORT_CODE(KEYCODE_TAB) PORT_CHAR(UCHAR_MAMEKEY(TAB)) // D00
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("2") PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@') // E02
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Hold Screen (F1)") PORT_CODE(KEYCODE_F1) PORT_CHAR(UCHAR_MAMEKEY(F1)) // G99

	PORT_START("KBD4") // Row P1-9
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("C") PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C') // B03
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("D") PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D') // C03
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("E") PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E') // D03
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("3") PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') // E03
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Setup (F3)") PORT_CODE(KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3)) // G01
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Print Screen (F2)") PORT_CODE(KEYCODE_F2) PORT_CHAR(UCHAR_MAMEKEY(F2)) // G00
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KBD5") // Row P1-11
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Space") PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ') // A04 and A06
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("V") PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V') // B04
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F") PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F') // C04
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("R") PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R') // D04
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("4") PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') // E04
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Break (F5)") PORT_CODE(KEYCODE_F5) PORT_CHAR(UCHAR_MAMEKEY(F5)) // G03
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Data / Talk (F4)") PORT_CODE(KEYCODE_F4) PORT_CHAR(UCHAR_MAMEKEY(F4)) // G02

	PORT_START("KBD6") // Row P2-2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("B") PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B') // B05
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("G") PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G') // C05
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("T") PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T') // D05
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("5") PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') // E05
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Interrupt (F6) [X]") PORT_CODE(KEYCODE_F6) PORT_CHAR(UCHAR_MAMEKEY(F6)) // G05
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNUSED ) // G04, exists but no key above this position (between F5 and F6)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KBD7") // Row P2-3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("N") PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N') // B06
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("H") PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H') // C06
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Y") PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y') // D06
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("6") PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^') // E06
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Resume (F7) [Fortsetzen]") PORT_CODE(KEYCODE_F7) PORT_CHAR(UCHAR_MAMEKEY(F7)) // G06
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Cancel (F8) [Zuruecknehmen]") PORT_CODE(KEYCODE_F8) PORT_CHAR(UCHAR_MAMEKEY(F8)) // G07
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KBD8") // Row P2-4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("M") PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M') // B07
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("J") PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J') // C07
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("U") PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U') // D07
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("7") PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&') // E07
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Exit (F10) [Fertig]") PORT_CODE(KEYCODE_F10) PORT_CHAR(UCHAR_MAMEKEY(F10)) // G09
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Main Screen (F9) [Hauptbild]") PORT_CODE(KEYCODE_F9) PORT_CHAR(UCHAR_MAMEKEY(F9)) // G08
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KBD9") // Row P2-6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME(",") PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<') // B08
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("K") PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K') // C08
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("I") PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I') // D08
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("8") PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('*') // E08
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED ) // G10, exists but no key above this position (between F10 and F11)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("ESC (F11)") PORT_CODE(KEYCODE_F11) PORT_CHAR(UCHAR_MAMEKEY(F11)) // G11
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KBD10") // Row P2-9
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME(".") PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>') // B09
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("L") PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L') // C09
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("O") PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O') // D09
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("9") PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR('(') // E09
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("LF (F13)") PORT_CODE(KEYCODE_F13) // G13
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("BS (F12)") PORT_CODE(KEYCODE_F12) PORT_CHAR(UCHAR_MAMEKEY(F12)) // G12
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KBD11") // Row P2-10
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("/") PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?') // B10
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME(";") PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':') // C10
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED ) // D13, exists but no key(?) above this position (under the upper half of the 'Return' key), DOES register as return; NOT LISTED on older schematic but definitely appears on the physical membrane of the later 6805 keyboard!
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("P") PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P') // D10
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("0") PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR(')') // E10
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Delete <X") PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8) // E13
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Additional Options (F14) [Zusaetze]") PORT_CODE(KEYCODE_PRTSCR) PORT_CHAR(UCHAR_MAMEKEY(PRTSCR)) // G14

	PORT_START("KBD12") // Row P2-11
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("\\") PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|') // C12
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT)) // B16
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Return") PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13) // C13
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("]") PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}') // D12
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Select") PORT_CODE(KEYCODE_DEL) PORT_CHAR(UCHAR_MAMEKEY(DEL)) // D16
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Help (F15)") PORT_CODE(KEYCODE_RALT) PORT_CHAR(UCHAR_MAMEKEY(RALT)) // G15
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("=") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+') // E12
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Find") PORT_CODE(KEYCODE_INSERT) PORT_CHAR(UCHAR_MAMEKEY(INSERT)) // E16

	PORT_START("KBD13") // Row P3-2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED ) // A17, exists but no key above this position (below the 'Down Arrow')
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED ) // B13, exists but no key above this position (right of right shift, 'whoami' or 'linefeed' key on decwriter?)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("'") PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'') PORT_CHAR('"') // C11
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("[") PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{') // D11
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Previous [^]") PORT_CODE(KEYCODE_END) PORT_CHAR(UCHAR_MAMEKEY(END)) // D17
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Do (F16) [Ausfuehren]") PORT_CODE(KEYCODE_RCONTROL) PORT_CHAR(UCHAR_MAMEKEY(RCONTROL)) // G16
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("-") PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('_') // E11
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Insert Here") PORT_CODE(KEYCODE_HOME) PORT_CHAR(UCHAR_MAMEKEY(HOME)) // E17

	PORT_START("KBD14") // Row P3-4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 0") PORT_CODE(KEYCODE_0_PAD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD)) // A20
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 1") PORT_CODE(KEYCODE_1_PAD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD)) // B20
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 4") PORT_CODE(KEYCODE_4_PAD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD)) // C20
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 7") PORT_CODE(KEYCODE_7_PAD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD)) // D20
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Up") PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP)) // C17
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Remove") PORT_CODE(KEYCODE_PGUP) PORT_CHAR(UCHAR_MAMEKEY(PGUP)) // E18
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Next [v]") PORT_CODE(KEYCODE_PGDN) PORT_CHAR(UCHAR_MAMEKEY(PGDN)) // D18
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF1") PORT_CODE(KEYCODE_NUMLOCK) PORT_CHAR(UCHAR_MAMEKEY(NUMLOCK)) // E20

	PORT_START("KBD15") // Row P3-5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) // A21, exists AND WORKS (as '0') but no key above this position (under right side of 'Keypad 0' key, maybe intended as a '00' key,)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 2") PORT_CODE(KEYCODE_2_PAD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD)) // B21
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT)) // B18
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 5") PORT_CODE(KEYCODE_5_PAD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD)) // C21
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 8") PORT_CODE(KEYCODE_8_PAD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD)) // D21
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF2") PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD)) // E21
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F17") // G20

	PORT_START("KBD16") // Row P3-6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num .") PORT_CODE(KEYCODE_DEL_PAD) PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD)) // A22
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 3") PORT_CODE(KEYCODE_3_PAD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD)) // B22
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 6") PORT_CODE(KEYCODE_6_PAD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD)) // C22
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN)) // B17
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 9") PORT_CODE(KEYCODE_9_PAD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD)) // D22
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF3") PORT_CODE(KEYCODE_ASTERISK) // E22
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F18") // G21

	PORT_START("KBD17") // Row P3-9
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD)) // A23
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num ,") PORT_CODE(KEYCODE_PLUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD)) // C23
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num -") // D23
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF4") PORT_CODE(KEYCODE_MINUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD)) // E23
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F20") // G23
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F19") // G22
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNUSED )

INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor lk201_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( lk201 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  lk201_device - constructor
//-------------------------------------------------

lk201_device::lk201_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, LK201, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	m_maincpu(*this, LK201_CPU_TAG),
	m_speaker(*this, LK201_SPK_TAG),
	m_kbd0(*this, "KBD0"),
	m_kbd1(*this, "KBD1"),
	m_kbd2(*this, "KBD2"),
	m_kbd3(*this, "KBD3"),
	m_kbd4(*this, "KBD4"),
	m_kbd5(*this, "KBD5"),
	m_kbd6(*this, "KBD6"),
	m_kbd7(*this, "KBD7"),
	m_kbd8(*this, "KBD8"),
	m_kbd9(*this, "KBD9"),
	m_kbd10(*this, "KBD10"),
	m_kbd11(*this, "KBD11"),
	m_kbd12(*this, "KBD12"),
	m_kbd13(*this, "KBD13"),
	m_kbd14(*this, "KBD14"),
	m_kbd15(*this, "KBD15"),
	m_kbd16(*this, "KBD16"),
	m_kbd17(*this, "KBD17"),
	m_tx_handler(*this)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void lk201_device::device_start()
{
	m_count = timer_alloc(1);
	m_tx_handler.resolve_safe();

	m_beeper = timer_alloc(2);
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void lk201_device::device_reset()
{
	m_beeper->adjust(attotime::never);

	m_speaker->set_state(0);
	m_speaker->set_output_gain(0, 0);

	ddrs[0] = ddrs[1] = ddrs[2] = 0;
	ports[0] = ports[1] = ports[2] = 0;

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rate(4800);
	m_count->adjust(attotime::from_hz(1200), 0, attotime::from_hz(1200));
	memset(m_timer.regs, 0, sizeof(m_timer.regs));

	sci_status = (SCSR_TC | SCSR_TDRE);

	spi_status = 0;
	spi_data = 0;

	kbd_data = 0;
	led_data = 0;

	transmit_register_reset();
	receive_register_reset();
}

void lk201_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
	case 1:
		m_timer.tsr |= TSR_OCFL;

		if ((m_timer.tcr & TCR_OCIE) && (m_timer.tsr & TSR_OCFL))
			m_maincpu->set_input_line(M68HC05EG_INT_TIMER, ASSERT_LINE);
		break;

	case 2:
		m_speaker->set_output_gain(0, 0);
		m_speaker->set_state(0);
		break;

	default:
		break;
	}
}


void lk201_device::rcv_complete()
{
	sci_status |= SCSR_RDRF;
	update_interrupts();
	receive_register_extract();

	int data = get_received_char();
	m_kbd_state = data;
//  printf("\nlk201 got %02x\n", m_kbd_state);
}

void lk201_device::tra_complete()
{
	sci_status |= (SCSR_TC | SCSR_TDRE);
	update_interrupts();
}

void lk201_device::tra_callback()
{
	m_tx_handler(transmit_register_get_data_bit());
}

void lk201_device::update_interrupts()
{
	if (sci_ctl2 & sci_status & SCSR_INT)
	{
		m_maincpu->set_input_line(M68HC05EG_INT_CPI, 1);
	}
	else
	{
		m_maincpu->set_input_line(M68HC05EG_INT_CPI, 0);
	}
}

READ8_MEMBER( lk201_device::timer_r )
{
	static uint16_t count;

	uint8_t ret = m_timer.regs[offset];

	switch (offset)
	{
	case 8:  // ACRH (high value is stored and reused when reading low)
		count = (m_maincpu->total_cycles() / 4) & 0x0000ffff;
		ret = count >> 8;
		break;
	case 9:  // ACRL
		ret = count & 0x00ff;
		break;
	}

	if(m_timer.tsr)
	{
		m_timer.tsr = 0;
		m_maincpu->set_input_line(M68HC05EG_INT_TIMER, CLEAR_LINE);
	}
	return ret;
}

WRITE8_MEMBER( lk201_device::timer_w )
{
	static uint16_t count;
	static int save_tsr;

	m_timer.regs[offset] = data;

	switch (offset)
	{
	case 4: // OCRH
		save_tsr = m_timer.tsr; //  prevent crashes in between OCRH / OCRL loads
		m_timer.tsr &= (255-TSR_OCFL); // OCFL flag tested for zero in TIMER routine

		count = m_timer.ocrh << 8; // store for later (when LOW is written).
		break;
	case 5 : // OCRL
		count |= m_timer.ocrl;
		m_timer.ocrh = count >> 8;

		m_timer.tsr = save_tsr; // restore flags
		break;
	}
}

READ8_MEMBER( lk201_device::ddr_r )
{
	return ddrs[offset];
}

WRITE8_MEMBER( lk201_device::ddr_w )
{
//  printf("%02x to PORT %c DDR (PC=%x)\n", data, 'A' + offset, m_maincpu->pc());

	uint8_t olddata = ddrs[offset];
	ddrs[offset] = data;
	send_port(space, offset, ports[offset] & olddata);
}

READ8_MEMBER( lk201_device::ports_r )
{
	uint8_t incoming = 0;

	// apply data direction registers
	incoming &= (ddrs[offset] ^ 0xff);
	// add in ddr-masked version of port writes
	incoming |= (ports[offset] & ddrs[offset]);

//  printf("PORT %c read = %02x (DDR = %02x latch = %02x) (PC=%x)\n", 'A' + offset, ports[offset], ddrs[offset], ports[offset], m_maincpu->pc());

	return incoming;
}

WRITE8_MEMBER( lk201_device::ports_w )
{
	uint8_t olddata = ports[offset];
	ports[offset] = data;            //   "port writes are independent of DDRC"
	send_port(space, offset, olddata & ddrs[offset]);
//  printf("\nPORT %c write %02x (OLD = %02x) (DDR = %02x) (PC=%x)\n", 'A' + offset, data, olddata, ddrs[offset], m_maincpu->pc());
}

void lk201_device::send_port(address_space &space, uint8_t offset, uint8_t olddata)
{
	uint8_t porta = ports[0] & ddrs[0];
	uint8_t portb = ports[1] & ddrs[1];
	uint8_t portc = ports[2] & ddrs[2];

	switch (offset)
	{
		case 0: // port A
			break;

		case 1: // port B
			break;

		case 2: // port C
			// Check for keyboard read strobe
			if (((portc & 0x40) == 0) && (olddata & 0x40))
			{
				if (porta & 0x1) kbd_data = m_kbd0->read();
				if (porta & 0x2) kbd_data = m_kbd1->read();
				if (porta & 0x4) kbd_data = m_kbd2->read();
				if (porta & 0x8) kbd_data = m_kbd3->read();
				if (porta & 0x10) kbd_data = m_kbd4->read();
				if (porta & 0x20) kbd_data = m_kbd5->read();
				if (porta & 0x40) kbd_data = m_kbd6->read();
				if (porta & 0x80) kbd_data = m_kbd7->read();
				if (portb & 0x1) kbd_data = m_kbd8->read();
				if (portb & 0x2) kbd_data = m_kbd9->read();
				if (portb & 0x4) kbd_data = m_kbd10->read();
				if (portb & 0x8) kbd_data = m_kbd11->read();
				if (portb & 0x10) kbd_data = m_kbd12->read();
				if (portb & 0x20) kbd_data = m_kbd13->read();
				if (portb & 0x40) kbd_data = m_kbd14->read();
				if (portb & 0x80) kbd_data = m_kbd15->read();
				if (portc & 0x1) kbd_data = m_kbd16->read();
				if (portc & 0x2) kbd_data = m_kbd17->read();
			}

			// Check for LED update strobe
			if (((portc & 0x80) == 0) && (olddata & 0x80))
			{
			if(ddrs[2] != 0x00)
			{   // Lower nibble contains the LED values (1 = on, 0 = off)
				machine().output().set_value("led_wait"   , (led_data & 0x1) == 1);
				machine().output().set_value("led_compose", (led_data & 0x2) == 2);
				machine().output().set_value("led_lock"   , (led_data & 0x4) == 4);
				machine().output().set_value("led_hold"   , (led_data & 0x8) == 8);
			}
			if (led_data & 0xf0)
			{
				m_speaker->set_state(1);
				// Beeps < 20 ms are clipped. A key click on a LK201 lasts 2 ms...
				if(m_kbd_state == LK_CMD_BELL)
					m_beeper->adjust(attotime::from_msec(125));
				else
					m_beeper->adjust(attotime::from_msec(25)); // see note
	}
			// Upper 4 bits of LED_DATA contain encoded volume info
			switch (led_data & 0xf0)
			{
				case 0xf0: // 8  - (see TABLE 4 in 68HC05xx ROM)
					m_speaker->set_output_gain(0, 100.0f);
					break;
				case 0xd0: // 7
					m_speaker->set_output_gain(0, (100 - (12 * 1)) / 100.0f);
					break;
				case 0xc0: // 6
					m_speaker->set_output_gain(0, (100 - (12 * 2)) / 100.0f);
					break;
				case 0x60: // 5
					m_speaker->set_output_gain(0, (100 - (12 * 3)) / 100.0f);
					break;
				case 0xb0: // 4
					m_speaker->set_output_gain(0, (100 - (12 * 4)) / 100.0f);
					break;
				case 0xa0: // 3
					m_speaker->set_output_gain(0, (100 - (12 * 5)) / 100.0f);
					break;
				case 0x30: // 2
					m_speaker->set_output_gain(0, (100 - (12 * 6)) / 100.0f);
					break;
				case 0x90: // 1
					m_speaker->set_output_gain(0, (100 - (12 * 7)) / 100.0f);
					break;
				default:
					;
			} // switch (volume)

		} // if (update_strobe)

		} // outer switch
}

READ8_MEMBER( lk201_device::sci_r )
{
	uint8_t incoming = 0;

	switch (offset)
	{
		case SCI_BAUD:  // Baud rate
			break;

		case SCI_SCCR1: // Control 1
			break;

		case SCI_SCCR2: // Control 2
			incoming = sci_ctl2;
			break;

		case SCI_SCSR:  // Status
			incoming = sci_status;
			break;

		case SCI_SCDR:  // Data
			incoming = get_received_char();
			sci_status &= ~SCSR_RDRF;
			m_maincpu->set_input_line(M68HC05EG_INT_CPI, 0);
			update_interrupts();
			break;
	}

//  printf("SCI read @ %x = %02x (PC=%x)\n", offset, incoming, m_maincpu->pc());

	return incoming;
}

WRITE8_MEMBER( lk201_device::sci_w )
{
	switch (offset)
	{
		case SCI_BAUD:  // Baud rate
			break;

		case SCI_SCCR1: // Control 1
			break;

		case SCI_SCCR2: // Control 2
			sci_ctl2 = data;
			update_interrupts();
			break;

		case SCI_SCSR:  // Status
			break;

		case SCI_SCDR:  // Data
//          printf("LK201: sending %02x\n", data);
			transmit_register_setup(data);
			sci_status &= ~(SCSR_TC | SCSR_TDRE);
			m_maincpu->set_input_line(M68HC05EG_INT_CPI, 0);
			update_interrupts();
			break;
	}

//  printf("SCI %02x to %x (PC=%x)\n", data, offset, m_maincpu->pc());
}

READ8_MEMBER( lk201_device::spi_r )
{
	uint8_t incoming = 0;

	switch (offset)
	{
		case SPI_SPCR:  // Control
			break;

		case SPI_SPSR:  // Status
			incoming = spi_status;
			spi_status &= ~SPSR_SPIF;
			break;

		case SPI_SPDR:  // Data I/O
			incoming = spi_data;
			break;
	}

//  printf("SPI read @ %x = %02x (PC=%x)\n", offset, incoming, m_maincpu->pc());

	return incoming;
}

WRITE8_MEMBER( lk201_device::spi_w )
{
	switch (offset)
	{
		case SPI_SPCR:  // Control
			break;

		case SPI_SPSR:  // Status (read only)
			break;

		case SPI_SPDR:  // Data I/O
			spi_data = data;

			// Transfer only allowed if transfer complete flag has been acknowleged
			if ((spi_status & SPSR_SPIF) == 0)
			{
				// Data out
				led_data = data;

				// Data in
				spi_data = kbd_data;

				// Indicate transfer complete
				spi_status |= SPSR_SPIF;
			}
			break;
	}

//  printf("SPI %02x to %x (PC=%x)\n", data, offset, m_maincpu->pc());
}
