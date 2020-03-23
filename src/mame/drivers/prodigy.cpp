// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom
/******************************************************************************

    ACI Prodigy chess computer driver

    TODO: Everything

    +-------------------------------------------------------------------------------------+
    |LEDS--------------------------------------------+        +-----------------+         |
    | | |              o o o o o o o o COLCN         |        |                 |         |
    | V |                                            |        | 4 char BCD LED  |         |
    | O | 8                                          |        +-----------------+         |
    |   |                                      ROWCN |        |||||||||||||||||||  ____   |
    | O | 7                                         o|+----------------------+    /    \  |
    |   |                                           o||  VIA                 |   ( beep ) |
    | O | 6                                         o|| R6522                |    \____/  |
    |   |                                           o|+----------------------+            |
    | O | 5                                         o|+----------------------+            |
    |   |                                           o||  CPU                 |            |
    | O | 4         8 x 8 button matrix             o|| R6502-13             |   +--+  __ |
    |   |                                           o|+----------------------+   |74| PWR||
    | O | 3                                         o|   o    o  +-------------+ |LS| SW >|
    |   |                                            | +=======+ |  ROM        | |14|  __||
    | O | 2                                          | | 2MHz  | | R2912       | +--+     |
    |   |                                            | |  XTAL | +-------------+     +--+ |
    | O | 1                                          |++-------+ +-------------+     |74| |
    |   |                                            || 74145N | |  RAM        |     |LS| |
    |   |   A     B    C    D    E    F     G    H   |+--------+ | M58725P     |     |00| |
    |   +--------------------------------------------+           +-------------+     +--+ |
    |LEDS-> O     O    O    O    O    O     O    O                 OOOOOOOOOOO KPDCN      |
    +-------------------------------------------------------------------------------------+

 Tracing the image shows that VIA Port A is used on the ROWCN and Port B on COLCN

 The VIA pins CB1 and CB2 together with PB0 and PB1 via the 74145 drives the BCD display.
 The BCD display is of common anode type and each anode a0-a3 selects which digit that
 will be lit selected by PB0 and PB1. The segments to be lit is selected by the 8 bits
 shifted into the 74164 by clock from VIA CB1 and data from CB2.

  Behind the BCD display we find the following supporting circuit

                                                               4x7 segment BCD display
                                      +---+       +-----+          +---+---+---+---+
+-----------------+            CB1    |74 |==/4/=>|2x   |==/8/====>| 0   1   2   3 |
|                 |       VIA  CB2    |164|==/4/=>|75491| segments |               |
| 4 char BCD LED  |  ===> 6522        +---+       +-----+          +---+---+---+---+
+-----------------+            PB1--->|74 |                          |   |   |   |
|||||||||||||||||||            PB2--->|145|=/4/=/R/=>b(4x    )c=/4/==============>
                                      +---+           (PN2907)e=+     anodes
                                                                |+5v
*******************************************************************************************/

#include "emu.h"
#include "cpu/m6502/m6502.h"
#include "machine/74145.h"
#include "machine/netlist.h"
#include "machine/nl_prodigy.h"
#include "machine/6522via.h"
// Generated artwork includes
#include "prodigy.lh"

#define LOG_SETUP   (1U <<  1)
#define LOG_READ    (1U <<  2)
#define LOG_BCD     (1U <<  3)
#define LOG_NETLIST (1U <<  4)
#define LOG_CLK     (1U <<  5)

//#define VERBOSE (LOG_BCD|LOG_NETLIST|LOG_SETUP)
//#define LOG_OUTPUT_FUNC printf

#include "logmacro.h"

#define LOGSETUP(...) LOGMASKED(LOG_SETUP,   __VA_ARGS__)
#define LOGR(...)     LOGMASKED(LOG_READ,    __VA_ARGS__)
#define LOGBCD(...)   LOGMASKED(LOG_BCD,     __VA_ARGS__)
#define LOGNL(...)    LOGMASKED(LOG_NETLIST, __VA_ARGS__)
#define LOGCLK(...)   LOGMASKED(LOG_CLK,     __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

#define NETLIST_TAG "bcd"
#define TTL74164DEV 0

class prodigy_state : public driver_device
{
public:
	prodigy_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_74145(*this, "io_74145")
		, m_segments(0)
		, m_via(*this, "via")
		, m_bcd(*this, NETLIST_TAG)
		, m_cb1(*this, "bcd:cb1")
		, m_cb2(*this, "bcd:cb2")
		, m_digit(0.0)
	{ }

	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit0_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit1_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit2_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit3_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit4_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit5_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit6_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(bcd_bit7_cb);

	DECLARE_WRITE8_MEMBER( via_pb_w );
	DECLARE_WRITE_LINE_MEMBER(via_cb1_w);
	DECLARE_WRITE_LINE_MEMBER(via_cb2_w);
	DECLARE_WRITE_LINE_MEMBER(irq_handler);

private:
	required_device<cpu_device> m_maincpu;
	required_device<ttl74145_device> m_74145;
	uint8_t m_segments;
	required_device<via6522_device> m_via;
#if TTL74164DEV
	required_device<ttl74164_device> m_shift;
#else
	required_device<netlist_mame_device> m_bcd;
	required_device<netlist_mame_logic_input_device> m_cb1;
	required_device<netlist_mame_logic_input_device> m_cb2;
#endif
	uint8_t m_digit;
	void update_bcd();

	virtual void device_reset() override;
};

NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit0_cb) { if (data != 0) m_digit |= 0x01; else m_digit &= ~(0x01); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit1_cb) { if (data != 0) m_digit |= 0x02; else m_digit &= ~(0x02); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit2_cb) { if (data != 0) m_digit |= 0x04; else m_digit &= ~(0x04); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit3_cb) { if (data != 0) m_digit |= 0x08; else m_digit &= ~(0x08); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit4_cb) { if (data != 0) m_digit |= 0x10; else m_digit &= ~(0x10); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit5_cb) { if (data != 0) m_digit |= 0x20; else m_digit &= ~(0x20); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit6_cb) { if (data != 0) m_digit |= 0x40; else m_digit &= ~(0x40); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }
NETDEV_LOGIC_CALLBACK_MEMBER(prodigy_state::bcd_bit7_cb) { if (data != 0) m_digit |= 0x80; else m_digit &= ~(0x80); LOGBCD("%s: %d m_digit: %02x\n", FUNCNAME, data, m_digit); }

void prodigy_state::device_reset()
{
#if TTL74164DEV
	m_shift->b_w(1);
	m_shift->clear_w(1);
#endif
}

WRITE_LINE_MEMBER(prodigy_state::via_cb1_w)
{
	LOGCLK("%s: %d\n", FUNCNAME, state);
	m_cb1->write(state);
}

WRITE_LINE_MEMBER(prodigy_state::via_cb2_w)
{
	LOGCLK("%s: %d\n", FUNCNAME, state);
	m_cb2->write(state);
}

WRITE_LINE_MEMBER(prodigy_state::irq_handler)
{
	LOGBCD("%s: %d\n", FUNCNAME, state);
	m_maincpu->set_input_line(M6502_IRQ_LINE, state);
	update_bcd();
}

/* Pulling the base of the PN2907 PNP transistor low by the output of the 74145 controlled by PB0 and PB1 will feed
   the coresponding anode enabling the correct digit and lit the segments currently held by the outputs of the 74164
   serial to 8 bit parallel converter fed by serial data using CB1 clock and CB2 data

   PB2 and PB3 is also connected to the 74145, usage to be traced....
*/
WRITE8_MEMBER( prodigy_state::via_pb_w ) // Needs to trace which port decides what digit
{
	LOGBCD("%s: %02x ANODE %02x\n", FUNCNAME, data, data & 0x03);
	m_74145->write( data & 0x0f ); // Write PB0-PB3 to the 74145
}

void prodigy_state::update_bcd()
{
	LOGBCD("%s\n", FUNCNAME);
	uint8_t ttl74145_data;
	uint8_t digit_nbr = 4;

	ttl74145_data = m_74145->read();
	LOGBCD(" - 74145: %02x\n", ttl74145_data);

	if ((ttl74145_data & 0x0f) != 0x00)
	{
		switch (ttl74145_data & 0x0f)
		{
		case 0x01: digit_nbr = 0; break;
		case 0x02: digit_nbr = 1; break;
		case 0x04: digit_nbr = 2; break;
		case 0x08: digit_nbr = 3; break;
		default: logerror("Wrong BCD digit, shouldn't happen, call the maintainer!\n");
		}

		LOGBCD(" - digit number: %d\n", digit_nbr);
		LOGBCD(" - segments: %02x -> ", m_digit);
		m_segments = m_digit;
		LOGBCD("%02x\n", m_segments);
		output().set_digit_value( digit_nbr, m_segments);
	}
}

static ADDRESS_MAP_START( maincpu_map, AS_PROGRAM, 8, prodigy_state )
	AM_RANGE(0x0000, 0x07ff) AM_RAM
	AM_RANGE(0x2000, 0x200f) AM_DEVREADWRITE("via", via6522_device, read, write)
	AM_RANGE(0x6000, 0x7fff) AM_ROM AM_REGION("roms", 0x0000) AM_MIRROR(0x8000)
ADDRESS_MAP_END

static INPUT_PORTS_START( prodigy )
INPUT_PORTS_END

static MACHINE_CONFIG_START( prodigy )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, XTAL_2MHz)
	MCFG_CPU_PROGRAM_MAP(maincpu_map)
	MCFG_DEFAULT_LAYOUT(layout_prodigy)

	MCFG_DEVICE_ADD("io_74145", TTL74145, 0)

	MCFG_DEVICE_ADD("via", VIA6522, XTAL_2MHz)
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(prodigy_state, irq_handler));
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(prodigy_state, via_pb_w))
	MCFG_VIA6522_CB1_HANDLER(WRITELINE(prodigy_state, via_cb1_w))
	MCFG_VIA6522_CB2_HANDLER(WRITELINE(prodigy_state, via_cb2_w))

	MCFG_DEVICE_ADD(NETLIST_TAG, NETLIST_CPU, XTAL_2MHz * 30)
	MCFG_NETLIST_SETUP(prodigy)

	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cb1", "cb1.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cb2", "cb2.IN", 0)
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit0",  "bcd_bit0",  prodigy_state, bcd_bit0_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit1",  "bcd_bit1",  prodigy_state, bcd_bit1_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit2",  "bcd_bit2",  prodigy_state, bcd_bit2_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit3",  "bcd_bit3",  prodigy_state, bcd_bit3_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit4",  "bcd_bit4",  prodigy_state, bcd_bit4_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit5",  "bcd_bit5",  prodigy_state, bcd_bit5_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit6",  "bcd_bit6",  prodigy_state, bcd_bit6_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT(NETLIST_TAG, "bcd_bit7",  "bcd_bit7",  prodigy_state, bcd_bit7_cb, "")
MACHINE_CONFIG_END

/*
 * 6522 VIA init sequence
 * :via Reg 03 <- 00 - DDRA - Data Direction A, pin A0-A7 programmed as inputs
 * :via Reg 02 <- 8f - DDRB - Data Direction B, pin B4-B6 programmed as inputs, other pins as outputs
 * :via Reg 0c <- e0 - PCR - Peripheral Control Register, CB2 out pos out, CB1 ints on neg edg, CA2 inp neg act edg, CA1 ints on neg edg
 * :via Reg 0b <- 58 - ACR - Auxilary Control Register, con ints, B7 dis, timed ints, shift out on 0/2, PA and PB latches disabled
 * :via Reg 09 <- 58 - T2C-H - Timer 2 High Order Counter value
 * :via Reg 04 <- ff - T1C-L - Timer 1 Low Order Latches
 * :via Reg 0e <- a0 - IER - Interrupt Enable Register, Timer 2 Interrupts enabled
 *
 * ISR VIA accesses
 * Reg 08 -> e1 - T2C-L - T2 low order counter
 * Reg 08 <- 6e - T2C-L - T2 low order latches
 * Reg 09 <- 20 - T2C-H - T2 High Order Counter
 * Reg 0a <- 00 - SR - Shift Register
 * Reg 00 <- 09
 * Reg 01 -> ff
 * Reg 00 -> 09
 * Reg 00 <- 08
 * Reg 01 -> ff
 * Reg 00 -> 08
 * Reg 00 <- 07
 * Reg 01 -> ff
 * Reg 00 <- 06
 * Reg 01 -> ff
 * Reg 00 <- 05
 * Reg 01 -> ff
 * Reg 00 <- 04
 * Reg 01 -> ff
 * Reg 00 <- 03
 * Reg 01 -> ff
 * Reg 00 <- 02
 * Reg 01 -> ff
 * Reg 00 <- 01
 * Reg 01 -> ff
 * Reg 00 <- 00
 * Reg 01 -> ff
 * Reg 0a <- 00
 * Reg 00 <- 05
 * Reg 08 -> e2
*/
ROM_START(prodigy)
	ROM_REGION(0x2000, "roms", 0)
	ROM_LOAD("0x2000.bin",  0x0000, 0x02000, CRC(8d60345a) SHA1(fff18ff12e1b1be91f8eac1178605a682564eff2))
ROM_END

//    YEAR  NAME        PARENT    COMPAT  MACHINE    INPUT      STATE          INIT  COMPANY,                FULLNAME,              FLAGS
CONS( 1981, prodigy,    0,        0,      prodigy,   prodigy,   prodigy_state, 0,    "Applied Concepts Inc", "ACI Destiny Prodigy", MACHINE_IS_SKELETON )
