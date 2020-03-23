// license:BSD-3-Clause
// copyright-holders:Kevin Horton,Jonathan Gevaryahu,Sandro Ronco,hap
// thanks-to:Berger,yoyo_chessboard
/******************************************************************************

    Fidelity Electronics 6502 based board driver

    NOTE: MAME doesn't include a generalized implementation for boardpieces yet,
    greatly affecting user playability of emulated electronic board games.
    As workaround for the chess games, use an external chess GUI on the side,
    such as Arena(in editmode).

    TODO:
    - verify cpu speed and rom labels where unknown
    - EAG missing bankswitch? where is the 2nd half of the 32KB ROM used, if at all?
    - granits gives error beeps at start, need to press clear to play
    - finish fphantom emulation

******************************************************************************

Champion Sensory Chess Challenger (CSC)
---------------------------------------

Memory map:
-----------
0000-07FF: 2K of RAM
0800-0FFF: 1K of RAM (note: mirrored twice)
1000-17FF: PIA 0 (display, TSI speech chip)
1800-1FFF: PIA 1 (keypad, LEDs)
2000-3FFF: 101-64019 ROM (also used on the regular sensory chess challenger)
4000-7FFF: mirror of 0000-3FFF
8000-9FFF: not used
A000-BFFF: 101-1025A03 ROM
C000-DFFF: 101-1025A02 ROM
E000-FDFF: 101-1025A01 ROM
FE00-FFFF: 512 byte 74S474 PROM

CPU is a 6502 running at 1.95MHz (3.9MHz resonator, divided by 2)

NMI is not used.
IRQ is connected to a 600Hz oscillator (38.4KHz divided by 64).
Reset is connected to a power-on reset circuit.

PIA 0:
------
PA0 - 7seg segments E, TSI A0
PA1 - 7seg segments D, TSI A1
PA2 - 7seg segments C, TSI A2
PA3 - 7seg segments H, TSI A3
PA4 - 7seg segments G, TSI A4
PA5 - 7seg segments F, TSI A5
PA6 - 7seg segments B
PA7 - 7seg segments A

PB0 - A12 on speech ROM (if used... not used on this model, ROM is 4K)
PB1 - START line on TSI
PB2 - white wire
PB3 - BUSY line from TSI
PB4 - hi/lo TSI speaker volume
PB5 - button row 9
PB6 - selection jumper (resistor to 5V)
PB7 - selection jumper (resistor to ground)

CA1 - NC
CA2 - violet wire

CB1 - NC
CB2 - NC (connects to pin 14 of soldered connector)

PIA 1:
------
PA0 - button row 1
PA1 - button row 2
PA2 - button row 3
PA3 - button row 4
PA4 - button row 5
PA5 - button row 6
PA6 - 7442 selector bit 0
PA7 - 7442 selector bit 1

PB0 - LED row 1
PB1 - LED row 2
PB2 - LED row 3
PB3 - LED row 4
PB4 - LED row 5
PB5 - LED row 6
PB6 - LED row 7
PB7 - LED row 8

CA1 - button row 7
CA2 - selector bit 3

CB1 - button row 8
CB2 - selector bit 2

Selector: (attached to PIA 1, outputs 1 of 10 pins low. 7442)
---------
output # (selected turns this column on, and all others off)
0 - LED column A, button column A, 7seg digit 1
1 - LED column B, button column B, 7seg digit 2
2 - LED column C, button column C, 7seg digit 3
3 - LED column D, button column D, 7seg digit 4
4 - LED column E, button column E
5 - LED column F, button column F
6 - LED column G, button column G
7 - LED column H, button column H
8 - button column I
9 - Tone line (toggle to make a tone in the buzzer)

The rows/columns are indicated on the game board:

 ABCDEFGH   I
--------------
|            | 8
|            | 7
|            | 6
|            | 5
|            | 4
|            | 3
|            | 2
|            | 1
--------------

The "lone LED" is above the control column.
column I is the "control column" on the right for starting a new game, etc.

The upper 6 buttons are connected as such:

column A - speak
column B - RV
column C - TM
column D - LV
column E - DM
column F - ST

these 6 buttons use row 9 (connects to PIA 0)

LED display:
------------
43 21 (digit number)
-----
88:88

The LED display is four 7 segment digits.  normal ABCDEFG lettering is used for segments.

The upper dot is connected to digit 3 common
The lower dot is connected to digit 4 common
The lone LED is connected to digit 1 common

All three of the above are called "segment H".


******************************************************************************

Super 9 Sensory Chess Challenger (SU9)
This is basically the Fidelity Elite A/S program on CSC hardware.
---------------------------------

R6502AP CPU, assume 2MHz
1 RAM chip, assume 4KB
2*8KB ROM + 1*2KB ROM (+another 8KB ROM?)
built-in CB9 module

See CSC description above for more information.


******************************************************************************

Reversi Sensory Challenger (RSC)
The 1st version came out in 1980, a program revision was released in 1981.
Another distinction is the board color and layout, the 1981 version is green.
---------------------------------

8*(8+1) buttons, 8*8+1 LEDs
1KB RAM(2*2114), 4KB ROM
MOS MPS 6502B CPU, frequency unknown
MOS MPS 6520 PIA, I/O is nearly same as CSC's PIA 1
PCB label 510-1035A01


******************************************************************************

Elite A/S Challenger (EAS)
This came out in 1982. 2 program updates were released in 1983 and 1984,
named Budapest and Glasgow, places where Fidelity won chess computer matches.
A/S stands for auto sensory, it's the 1st Fidelity board with magnet sensors.
---------------------------------

8*8 magnet sensors, 11 buttons, 8*(8+1) LEDs + 4*7seg LEDs
R65C02P4 or R6502BP CPU, default frequency 3MHz*
4KB RAM (2*HM6116), 24KB ROM
TSI S14001A + speech ROM
I/O with 8255 PPI and bunch of TTL
module slot and printer port
PCB label 510-1071A01

*It was advertised as 3.2, 3.6, or 4MHz, with unofficial modifications up to 8MHz.
PCB photos show only a 3MHz XTAL.

A condensator keeps RAM contents alive for a few hours when powered off.

Elite Avant Garde (models 6081,6088,6089) is on the same hardware.


******************************************************************************

Sensory Chess Challenger "9" (SC9)
3 versions were available, the newest "B" version was 2MHz and included the Budapest program.
The Playmatic S was only released in Germany, it's basically a 'deluxe' version of SC9
with magnet sensors and came with CB9 and CB16.
---------------------------------

8*(8+1) buttons, 8*8+1 LEDs
36-pin edge connector, assume same as SC12
4KB RAM(TMM2016P), 2*8KB ROM(HN48364P)
R6502-13, 1.4MHz from resonator, another pcb with the same resonator was measured 1.49MHz*
PCB label 510-1046C01 2-1-82

*: 2 other boards were measured 1.60MHz and 1.88MHz(newest serial). Online references
suggest 3 versions of SC9(C01) total: 1.5MHz, 1.6MHz, and 1.9MHz.

I/O is via TTL, not further documented here


******************************************************************************

Sensory 12 Chess Challenger (SC12-B, 6086)
4 versions are known to exist: A,B,C, and X, with increasing CPU speed.
---------------------------------
RE information from netlist by Berger

8*(8+1) buttons, 8+8+2 red LEDs
DIN 41524C printer port
36-pin edge connector
CPU is a R65C02P4, running at 4MHz

NE556 dual-timer IC:
- timer#1, one-shot at power-on, to CPU _RESET
- timer#2: R1=82K+50K pot at 26K, R2=1K, C=22nF, to CPU _IRQ: ~596Hz, active low=15.25us

Memory map:
-----------
0000-0FFF: 4K RAM (2016 * 2)
2000-5FFF: cartridge
6000-7FFF: control(W)
8000-9FFF: 8K ROM SSS SCM23C65E4
A000-BFFF: keypad(R)
C000-DFFF: 4K ROM TI TMS2732AJL-45
E000-FFFF: 8K ROM Toshiba TMM2764D-2

control: (74LS377)
--------
Q0-Q3: 7442 A0-A3
Q4: enable printer port pin 1 input
Q5: printer port pin 5 output
Q6,Q7: LEDs common anode

7442 0-8: input mux and LEDs cathode
7442 9: buzzer

The keypad is read through a 74HC251, where S0,1,2 is from CPU A0,1,2, Y is connected to CPU D7.
If control Q4 is set, printer data can be read from I0.


******************************************************************************

Voice Excellence (model 6092)
----------------
PCB 1: 510.1117A02, appears to be identical to other "Excellence" boards
CPU: GTE G65SC102P-3, 32 KB PRG ROM: AMI 101-1080A01(IC5), 8192x8 SRAM SRM2264C10(IC6)
2 rows of LEDs on the side: 1*8 green, 1*8 red

PCB 2: 510.1117A01
Speech: TSI S14001A, 32 KB ROM: AMI 101-1081A01(IC2)
Dip Switches set ROM A13 and ROM A14, on the side of the board

ROM A12 is tied to S14001A's A11 (yuck)
ROM A11 is however tied to the CPU's XYZ

0000_07FF - Spanish 1/4
0800_0FFF - Spanish 3/4
1000_17FF - Spanish 2/4
1800_1FFF - Spanish 4/4

2000_27FF - French 1/4
2800_2FFF - French 3/4
3000_3FFF - French 2/4
3800_3FFF - French 4/4

4000_47FF - German 1/4
4800_4FFF - German 3/4
5000_57FF - German 2/4
5800_5FFF - German 4/4

6000_67FF - English 1/2
6800_6FFF - Bridge Challenger 1/2
7000_77FF - English 2/2
7800_7FFF - Bridge Challenger 2/2

------------------
RE info by hap, based on PCB photos

Memory map:
-----------
0000-3FFF: 8K RAM (SRM2264)
4000-7FFF: control (R/W)
8000-FFFF: 32K ROM (M27256 compatible)

control (W):
------------
CPU A0-A2 to 3*74259, CPU Dx to D (_C unused)

CPU D0:
- Q4,Q5: led commons
- Q6,Q7,Q2,Q1: 7seg panel digit select
- Q0-Q3: 7442 A0-A3
  + 0-7: led data
  + 0-8: keypad mux
  + 9: buzzer out

CPU D1: (model 6093)
- Q0-Q7: 7seg data

CPU D2: (model 6092)
- Q0-Q5: TSI C0-C5
- Q6: TSI START pin
- Q7: TSI ROM A11

A11 from TSI is tied to TSI ROM A12(!)
TSI ROM A13,A14 are hardwired to the 2 language switches.
Sound comes from the Audio out pin, digital out pins are N/C.

control (R):
------------
CPU A0-A2 to 2*74251, CPU Dx to output

CPU D7 to Y:
- D0-D7: keypad row data

CPU D6 to W: (model 6092, tied to VCC otherwise)
- D0,D1: language switches
- D2-D6: VCC
- D7: TSI BUSY


******************************************************************************

Designer series:

Designer 2000 (model 6102)
----------------
8KB RAM(KM6264AL-10), 32KB ROM(AMI 101.1077A01)
Ricoh RP65C02G CPU, 3MHz XTAL
PCB label 510.1129A01
basically same as (Par) Excellence hardware, reskinned board

Designer 2100 (model 6103): exactly same, but running at 5MHz

Designer 2100 Display (model 6106)
----------------
8KB RAM(MS6264L-10), 2*32KB ROM(27C256)
WDC W65C02P-6 CPU, 6MHz XTAL
4-digit LCD panel
PCB label 510.1130A01

Designer 2000 Display (model 6105): same hardware, no bookrom, 3MHz

Designer 1500 is on 80C50 hardware


******************************************************************************

Phantom (model 6100)
----------------
R65C02P4, XTAL marked 4.91?200
2*32KB ROM 27C256-15, 8KB RAM MS6264L-10
LCD driver, display panel for digits
magnetized x/y motor under chessboard, chesspieces have magnet underneath
piezo speaker, LEDs, 8*8 chessboard buttons
PCB label 510.1128A01

Fidelity licensed the design of the Milton/Phantom motorized chessboard and released
their own version. It has a small LCD panel added, the rest looks nearly the same from
the outside. After Fidelity was taken over by H&G, it was rereleased in 1990 as the
Mephisto Phantom. This is assumed to be identical.


******************************************************************************

Chesster (model 6120)
There is also a German version titled Kishon Chesster (model 6120G, or 6127)
----------------

8*(8+1) buttons, 8+8+1 LEDs
8KB RAM(UM6264-12), 32KB ROM(M27C256B)
Ricoh RP65C02G CPU, 5MHz XTAL
8-bit DAC speech timed via IRQ, 128KB ROM(AMI custom label)
PCB label 510.1141C01

I/O is via TTL, very similar to Designer Display

******************************************************************************/

#include "emu.h"
#include "includes/fidelbase.h"

#include "cpu/m6502/m6502.h"
#include "cpu/m6502/r65c02.h"
#include "cpu/m6502/m65sc02.h"
#include "machine/6821pia.h"
#include "machine/i8255.h"
#include "machine/nvram.h"
#include "sound/volt_reg.h"
#include "speaker.h"

// internal artwork
#include "fidel_chesster.lh" // clickable
#include "fidel_csc.lh" // clickable, with preliminary boardpieces simulation
#include "fidel_des.lh" // clickable
#include "fidel_desdis.lh" // clickable
#include "fidel_eag.lh" // clickable
#include "fidel_eas.lh" // clickable
#include "fidel_ex.lh" // clickable
#include "fidel_exd.lh" // clickable
#include "fidel_playmatic.lh" // clickable
#include "fidel_rsc_v2.lh" // clickable
#include "fidel_sc9.lh" // clickable
#include "fidel_sc12.lh" // clickable
#include "fidel_su9.lh" // clickable


class fidel6502_state : public fidelbase_state
{
public:
	fidel6502_state(const machine_config &mconfig, device_type type, const char *tag)
		: fidelbase_state(mconfig, type, tag),
		m_ppi8255(*this, "ppi8255"),
		m_cart(*this, "cartslot")
	{ }

	// devices/pointers
	optional_device<i8255_device> m_ppi8255;
	optional_device<generic_slot_device> m_cart;

	TIMER_DEVICE_CALLBACK_MEMBER(irq_on) { m_maincpu->set_input_line(M6502_IRQ_LINE, ASSERT_LINE); }
	TIMER_DEVICE_CALLBACK_MEMBER(irq_off) { m_maincpu->set_input_line(M6502_IRQ_LINE, CLEAR_LINE); }

	// CSC, SU9, RSC
	void csc_prepare_display();
	DECLARE_READ8_MEMBER(csc_speech_r);
	DECLARE_WRITE8_MEMBER(csc_pia0_pa_w);
	DECLARE_WRITE8_MEMBER(csc_pia0_pb_w);
	DECLARE_READ8_MEMBER(csc_pia0_pb_r);
	DECLARE_WRITE_LINE_MEMBER(csc_pia0_ca2_w);
	DECLARE_WRITE8_MEMBER(csc_pia1_pa_w);
	DECLARE_WRITE8_MEMBER(csc_pia1_pb_w);
	DECLARE_READ8_MEMBER(csc_pia1_pa_r);
	DECLARE_WRITE_LINE_MEMBER(csc_pia1_ca2_w);
	DECLARE_WRITE_LINE_MEMBER(csc_pia1_cb2_w);
	DECLARE_READ_LINE_MEMBER(csc_pia1_ca1_r);
	DECLARE_READ_LINE_MEMBER(csc_pia1_cb1_r);

	// EAS, EAG
	void eas_prepare_display();
	DECLARE_WRITE8_MEMBER(eas_segment_w);
	DECLARE_WRITE8_MEMBER(eas_led_w);
	DECLARE_READ8_MEMBER(eas_input_r);
	DECLARE_WRITE8_MEMBER(eas_ppi_porta_w);
	DECLARE_READ8_MEMBER(eas_ppi_portb_r);
	DECLARE_WRITE8_MEMBER(eas_ppi_portc_w);

	// SC9
	void sc9_prepare_display();
	DECLARE_WRITE8_MEMBER(sc9_control_w);
	DECLARE_WRITE8_MEMBER(sc9_led_w);
	DECLARE_READ8_MEMBER(sc9_input_r);
	DECLARE_MACHINE_RESET(sc9c);
	DECLARE_INPUT_CHANGED_MEMBER(sc9c_cpu_freq);
	void sc9c_set_cpu_freq();

	// SC12
	DECLARE_WRITE8_MEMBER(sc12_control_w);
	DECLARE_READ8_MEMBER(sc12_input_r);
	DECLARE_READ8_MEMBER(sc12_cart_r);

	// Excellence
	DECLARE_INPUT_CHANGED_MEMBER(fexcelv_bankswitch);
	DECLARE_READ8_MEMBER(fexcelv_speech_r);
	DECLARE_WRITE8_MEMBER(fexcel_ttl_w);
	DECLARE_READ8_MEMBER(fexcelb_ttl_r);
	DECLARE_READ8_MEMBER(fexcel_ttl_r);

	// Designer Display
	DECLARE_WRITE8_MEMBER(fdesdis_control_w);
	DECLARE_WRITE8_MEMBER(fdesdis_lcd_w);
	DECLARE_READ8_MEMBER(fdesdis_input_r);
	DECLARE_DRIVER_INIT(fdesdis);

	// Phantom
	DECLARE_MACHINE_RESET(fphantom);
	DECLARE_DRIVER_INIT(fphantom);

	// Chesster
	DECLARE_WRITE8_MEMBER(chesster_control_w);
	DECLARE_WRITE8_MEMBER(kishon_control_w);
	DECLARE_DRIVER_INIT(chesster);
};



// Devices, I/O

/******************************************************************************
    CSC, SU9, RSC
******************************************************************************/

// misc handlers

void fidel6502_state::csc_prepare_display()
{
	// 7442 0-8: led select, input mux
	m_inp_mux = 1 << m_led_select & 0x3ff;

	// 7442 9: speaker out
	m_dac->write(BIT(m_inp_mux, 9));

	// 7seg leds+H (not on all models), 8*8(+1) chessboard leds
	set_display_segmask(0xf, 0x7f);
	display_matrix(16, 9, m_led_data << 8 | m_7seg_data, m_inp_mux);
}

READ8_MEMBER(fidel6502_state::csc_speech_r)
{
	return m_speech_rom[m_speech_bank << 12 | offset];
}


// 6821 PIA 0

WRITE8_MEMBER(fidel6502_state::csc_pia0_pa_w)
{
	// d0-d5: TSI C0-C5
	m_speech->data_w(space, 0, data & 0x3f);

	// d0-d7: data for the 4 7seg leds, bits are ABFGHCDE (H is extra led)
	m_7seg_data = BITSWAP8(data,0,1,5,6,7,2,3,4);
	csc_prepare_display();
}

WRITE8_MEMBER(fidel6502_state::csc_pia0_pb_w)
{
	// d0: speech ROM A12
	m_speech->force_update(); // update stream to now
	m_speech_bank = data & 1;

	// d1: TSI START line
	m_speech->start_w(data >> 1 & 1);

	// d4: lower TSI volume
	m_speech->set_output_gain(0, (data & 0x10) ? 0.5 : 1.0);
}

READ8_MEMBER(fidel6502_state::csc_pia0_pb_r)
{
	// d2: printer?
	u8 data = 0x04;

	// d3: TSI BUSY line
	if (m_speech->busy_r())
		data |= 0x08;

	// d5: button row 8 (active low)
	// d6,d7: language switches
	data |= (~read_inputs(9) >> 3 & 0x20) | (~m_inp_matrix[9]->read() << 6 & 0xc0);

	return data;
}

WRITE_LINE_MEMBER(fidel6502_state::csc_pia0_ca2_w)
{
	// printer?
}


// 6821 PIA 1

READ8_MEMBER(fidel6502_state::csc_pia1_pa_r)
{
	// d0-d5: button row 0-5 (active low)
	return (read_inputs(9) & 0x3f) ^ 0xff;
}

WRITE8_MEMBER(fidel6502_state::csc_pia1_pa_w)
{
	// d6,d7: 7442 A0,A1
	m_led_select = (m_led_select & ~3) | (data >> 6 & 3);
	csc_prepare_display();
}

WRITE8_MEMBER(fidel6502_state::csc_pia1_pb_w)
{
	// d0-d7: led row data
	m_led_data = data;
	csc_prepare_display();
}

READ_LINE_MEMBER(fidel6502_state::csc_pia1_ca1_r)
{
	// button row 6 (active low)
	return ~read_inputs(9) >> 6 & 1;
}

READ_LINE_MEMBER(fidel6502_state::csc_pia1_cb1_r)
{
	// button row 7 (active low)
	return ~read_inputs(9) >> 7 & 1;
}

WRITE_LINE_MEMBER(fidel6502_state::csc_pia1_cb2_w)
{
	// 7442 A2
	m_led_select = (m_led_select & ~4) | (state ? 4 : 0);
	csc_prepare_display();
}

WRITE_LINE_MEMBER(fidel6502_state::csc_pia1_ca2_w)
{
	// 7442 A3
	m_led_select = (m_led_select & ~8) | (state ? 8 : 0);
	csc_prepare_display();
}



/******************************************************************************
    EAS, EAG
******************************************************************************/

// TTL/generic

void fidel6502_state::eas_prepare_display()
{
	// 4/8 7seg leds+H, 8*8(+1) chessboard leds
	set_display_segmask(0x1ef, 0x7f);
	display_matrix(16, 9, m_led_data << 8 | m_7seg_data, m_led_select);
}

WRITE8_MEMBER(fidel6502_state::eas_segment_w)
{
	// a0-a2,d7: digit segment
	m_7seg_data = (data & 0x80) >> offset;
	m_7seg_data = BITSWAP8(m_7seg_data,7,6,4,5,0,2,1,3);
	eas_prepare_display();
}

WRITE8_MEMBER(fidel6502_state::eas_led_w)
{
	// a0-a2,d0: led data
	m_led_data = (data & 1) << offset;
	eas_prepare_display();
}

READ8_MEMBER(fidel6502_state::eas_input_r)
{
	// multiplexed inputs (active low)
	return read_inputs(9) ^ 0xff;
}


// 8255 PPI

WRITE8_MEMBER(fidel6502_state::eas_ppi_porta_w)
{
	// d0-d5: TSI C0-C5
	// d6: TSI START line
	m_speech->data_w(space, 0, data & 0x3f);
	m_speech->start_w(data >> 6 & 1);

	// d7: printer? (black wire to LED pcb)
}

WRITE8_MEMBER(fidel6502_state::eas_ppi_portc_w)
{
	// d0-d3: 7442 a0-a3
	// 7442 0-8: led select, input mux
	m_led_select = 1 << (data & 0xf) & 0x3ff;
	m_inp_mux = m_led_select & 0x1ff;
	eas_prepare_display();

	// 7442 9: speaker out
	m_dac->write(BIT(m_led_select, 9));

	// d4: speech ROM A12
	m_speech->force_update(); // update stream to now
	m_speech_bank = data >> 4 & 1;

	// d5: lower TSI volume
	m_speech->set_output_gain(0, (data & 0x20) ? 0.5 : 1.0);

	// d6,d7: N/C?
}

READ8_MEMBER(fidel6502_state::eas_ppi_portb_r)
{
	// d0: printer? white wire from LED pcb
	u8 data = 1;

	// d1: TSI BUSY line
	data |= (m_speech->busy_r()) ? 2 : 0;

	// d2,d3: language switches
	data |= ~m_inp_matrix[9]->read() << 2 & 0x0c;

	// d5: multiplexed inputs highest bit
	data |= (read_inputs(9) & 0x100) ? 0 : 0x20;

	// other: ?
	return data | 0xd0;
}



/******************************************************************************
    SC9
******************************************************************************/

// TTL/generic

void fidel6502_state::sc9_prepare_display()
{
	// 8*8 chessboard leds + 1 corner led
	display_matrix(8, 9, m_led_data, m_inp_mux);
}

WRITE8_MEMBER(fidel6502_state::sc9_control_w)
{
	// d0-d3: 74245 P0-P3
	// 74245 Q0-Q8: input mux, led select
	u16 sel = 1 << (data & 0xf) & 0x3ff;
	m_inp_mux = sel & 0x1ff;
	sc9_prepare_display();

	// 74245 Q9: speaker out
	m_dac->write(BIT(sel, 9));

	// d4,d5: ?
	// d6,d7: N/C
}

WRITE8_MEMBER(fidel6502_state::sc9_led_w)
{
	// a0-a2,d0: led data via NE591N
	m_led_data = (data & 1) << offset;
	sc9_prepare_display();
}

READ8_MEMBER(fidel6502_state::sc9_input_r)
{
	// multiplexed inputs (active low)
	return read_inputs(9) ^ 0xff;
}

void fidel6502_state::sc9c_set_cpu_freq()
{
	// SC9(C01) was released with 1.5MHz, 1.6MHz, or 1.9MHz CPU
	u8 inp = ioport("FAKE")->read();
	m_maincpu->set_unscaled_clock((inp & 2) ? 1900000 : ((inp & 1) ? 1600000 : 1500000));
}

MACHINE_RESET_MEMBER(fidel6502_state, sc9c)
{
	fidelbase_state::machine_reset();
	sc9c_set_cpu_freq();
}



/******************************************************************************
    SC12
******************************************************************************/

// TTL/generic

WRITE8_MEMBER(fidel6502_state::sc12_control_w)
{
	// d0-d3: 7442 a0-a3
	// 7442 0-8: led data, input mux
	u16 sel = 1 << (data & 0xf) & 0x3ff;
	m_inp_mux = sel & 0x1ff;

	// 7442 9: speaker out
	m_dac->write(BIT(sel, 9));

	// d6,d7: led select (active low)
	display_matrix(9, 2, sel & 0x1ff, ~data >> 6 & 3);

	// d4,d5: printer
	//..
}

READ8_MEMBER(fidel6502_state::sc12_input_r)
{
	// a0-a2,d7: multiplexed inputs (active low)
	return (read_inputs(9) >> offset & 1) ? 0 : 0x80;
}

READ8_MEMBER(fidel6502_state::sc12_cart_r)
{
	if (m_cart->exists())
		return m_cart->read_rom(space, offset);
	else
		return 0;
}



/******************************************************************************
    Excellence
******************************************************************************/

// misc handlers

INPUT_CHANGED_MEMBER(fidel6502_state::fexcelv_bankswitch)
{
	// tied to speech ROM highest bits
	m_speech->force_update();
	m_speech_bank = (m_speech_bank & 1) | newval << 1;
}

READ8_MEMBER(fidel6502_state::fexcelv_speech_r)
{
	// TSI A11 is A12, program controls A11, user controls A13,A14(language switches)
	offset = (offset & 0x7ff) | (offset << 1 & 0x1000);
	return m_speech_rom[offset | (m_speech_bank << 11 & 0x800) | (~m_speech_bank << 12 & 0x6000)];
}


// TTL

WRITE8_MEMBER(fidel6502_state::fexcel_ttl_w)
{
	// a0-a2,d0: 74259(1)
	u8 mask = 1 << offset;
	m_led_select = (m_led_select & ~mask) | ((data & 1) ? mask : 0);

	// 74259 Q0-Q3: 7442 a0-a3
	// 7442 0-8: led data, input mux
	u16 sel = 1 << (m_led_select & 0xf) & 0x3ff;
	u8 led_data = sel & 0xff;
	m_inp_mux = sel & 0x1ff;

	// 7442 9: speaker out
	m_dac->write(BIT(sel, 9));

	// 74259 Q4-Q7,Q2,Q1: digit/led select (active low)
	u8 led_sel = ~BITSWAP8(m_led_select,0,3,1,2,7,6,5,4) & 0x3f;

	// a0-a2,d1: digit segment data (model 6093)
	m_7seg_data = (m_7seg_data & ~mask) | ((data & 2) ? mask : 0);
	u8 seg_data = BITSWAP8(m_7seg_data,0,1,3,2,7,5,6,4);

	// update display: 4 7seg leds, 2*8 chessboard leds
	for (int i = 0; i < 6; i++)
		m_display_state[i] = (led_sel >> i & 1) ? ((i < 2) ? led_data : seg_data) : 0;

	set_display_size(8, 2+4);
	set_display_segmask(0x3c, 0x7f);
	display_update();

	// speech (model 6092)
	if (m_speech != nullptr)
	{
		// a0-a2,d2: 74259(2) to speech board
		m_speech_data = (m_speech_data & ~mask) | ((data & 4) ? mask : 0);

		// 74259 Q6: TSI ROM A11
		m_speech->force_update(); // update stream to now
		m_speech_bank = (m_speech_bank & ~1) | (m_speech_data >> 6 & 1);

		// Q0-Q5: TSI C0-C5
		// Q7: TSI START line
		m_speech->data_w(space, 0, m_speech_data & 0x3f);
		m_speech->start_w(m_speech_data >> 7 & 1);
	}
}

READ8_MEMBER(fidel6502_state::fexcelb_ttl_r)
{
	// a0-a2,d6: from speech board: language switches and TSI BUSY line, otherwise tied to VCC
	u8 d6 = (m_inp_matrix[9].read_safe(0xff) >> offset & 1) ? 0x40 : 0;

	// a0-a2,d7: multiplexed inputs (active low)
	return d6 | ((read_inputs(9) >> offset & 1) ? 0 : 0x80);
}

READ8_MEMBER(fidel6502_state::fexcel_ttl_r)
{
	u8 d7 = 0x80;

	// 74259(1) Q7 + 74251 I0: battery status
	if (m_inp_mux == 1 && ~m_led_select & 0x80)
		d7 = m_inp_matrix[9]->read() & 0x80;

	// a0-a2,d7: multiplexed inputs (active low)
	return d7 & ((read_inputs(9) >> offset & 1) ? 0 : 0x80);
}



/******************************************************************************
    Designer Display
******************************************************************************/

// TTL/generic

WRITE8_MEMBER(fidel6502_state::fdesdis_control_w)
{
	u8 q3_old = m_led_select & 8;

	// a0-a2,d7: 74259
	u8 mask = 1 << offset;
	m_led_select = (m_led_select & ~mask) | ((data & 0x80) ? mask : 0);

	// 74259 Q4-Q7: 7442 a0-a3
	// 7442 0-8: led data, input mux
	u16 sel = 1 << (m_led_select >> 4 & 0xf) & 0x3ff;
	m_inp_mux = sel & 0x1ff;

	// 7442 9: speaker out
	m_dac->write(BIT(sel, 9));

	// 74259 Q0,Q1: led select (active low)
	display_matrix(9, 2, m_inp_mux, ~m_led_select & 3, false);

	// 74259 Q2: book rom A14
	membank("bank1")->set_entry(~m_led_select >> 2 & 1);

	// 74259 Q3: lcd common, update on rising edge
	if (~q3_old & m_led_select & 8)
	{
		for (int i = 0; i < 4; i++)
			m_display_state[i+2] = m_7seg_data >> (8*i) & 0xff;
	}

	m_display_maxy += 4;
	set_display_segmask(0x3c, 0x7f);
	display_update();
}

WRITE8_MEMBER(fidel6502_state::fdesdis_lcd_w)
{
	// a0-a2,d0-d3: 4*74259 to lcd digit segments
	u32 mask = BITSWAP8(1 << offset,3,7,6,0,1,2,4,5);
	for (int i = 0; i < 4; i++)
	{
		m_7seg_data = (m_7seg_data & ~mask) | ((data >> i & 1) ? 0 : mask);
		mask <<= 8;
	}
}

READ8_MEMBER(fidel6502_state::fdesdis_input_r)
{
	// a0-a2,d7: multiplexed inputs (active low)
	return (read_inputs(9) >> offset & 1) ? 0 : 0x80;
}

DRIVER_INIT_MEMBER(fidel6502_state, fdesdis)
{
	membank("bank1")->configure_entries(0, 2, memregion("user1")->base(), 0x4000);
}



/******************************************************************************
    Phantom
******************************************************************************/

// TTL/generic

MACHINE_RESET_MEMBER(fidel6502_state, fphantom)
{
	fidelbase_state::machine_reset();
	membank("bank1")->set_entry(0);
}

DRIVER_INIT_MEMBER(fidel6502_state, fphantom)
{
	membank("bank1")->configure_entries(0, 2, memregion("user1")->base(), 0x4000);
}



/******************************************************************************
    Chesster
******************************************************************************/

// TTL/generic

WRITE8_MEMBER(fidel6502_state::chesster_control_w)
{
	// a0-a2,d7: 74259(1)
	u8 mask = 1 << offset;
	m_led_select = (m_led_select & ~mask) | ((data & 0x80) ? mask : 0);

	// 74259 Q4-Q7: 7442 a0-a3
	// 7442 0-8: led data, input mux
	u16 sel = 1 << (m_led_select >> 4 & 0xf) & 0x3ff;
	m_inp_mux = sel & 0x1ff;

	// 74259 Q0,Q1: led select (active low)
	display_matrix(9, 2, m_inp_mux, ~m_led_select & 3);

	// 74259 Q2,Q3: speechrom A14,A15
	// a0-a2,d0: 74259(2) where Q3 is speechrom A16, other outputs unconnected
	m_speech_bank = (m_speech_bank & ~mask) | ((data & 1) ? mask : 0);
	membank("bank1")->set_entry((m_led_select >> 2 & 3) | (m_speech_bank >> 1 & 4));
}

WRITE8_MEMBER(fidel6502_state::kishon_control_w)
{
	chesster_control_w(space, offset, data);

	// 2 more bankswitch bits: 74259(2) Q2 to A17, Q0 to A18
	membank("bank1")->set_entry((m_led_select >> 2 & 3) | (m_speech_bank >> 1 & 4) | (m_speech_bank << 1 & 8) | (m_speech_bank << 4 & 0x10));
}

DRIVER_INIT_MEMBER(fidel6502_state, chesster)
{
	membank("bank1")->configure_entries(0, memregion("user1")->bytes() / 0x4000, memregion("user1")->base(), 0x4000);
}



/******************************************************************************
    Address Maps
******************************************************************************/

// CSC, SU9, RSC

static ADDRESS_MAP_START( csc_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x07ff) AM_MIRROR(0x4000) AM_RAM
	AM_RANGE(0x0800, 0x0bff) AM_MIRROR(0x4400) AM_RAM
	AM_RANGE(0x1000, 0x1003) AM_MIRROR(0x47fc) AM_DEVREADWRITE("pia0", pia6821_device, read, write)
	AM_RANGE(0x1800, 0x1803) AM_MIRROR(0x47fc) AM_DEVREADWRITE("pia1", pia6821_device, read, write)
	AM_RANGE(0x2000, 0x3fff) AM_MIRROR(0x4000) AM_ROM
	AM_RANGE(0xa000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( su9_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x0fff) AM_RAM
	AM_RANGE(0x1000, 0x1003) AM_DEVREADWRITE("pia0", pia6821_device, read, write)
	AM_RANGE(0x1800, 0x1803) AM_DEVREADWRITE("pia1", pia6821_device, read, write)
	AM_RANGE(0x2000, 0x3fff) AM_ROM
	AM_RANGE(0xa000, 0xa7ff) AM_ROM
	AM_RANGE(0xc000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( rsc_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x03ff) AM_RAM
	AM_RANGE(0x2000, 0x2003) AM_DEVREADWRITE("pia", pia6821_device, read, write)
	AM_RANGE(0xf000, 0xffff) AM_ROM
ADDRESS_MAP_END


// EAS, EAG

static ADDRESS_MAP_START( eas_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x0fff) AM_RAM AM_SHARE("nvram")
	AM_RANGE(0x2000, 0x5fff) AM_READ(sc12_cart_r)
	AM_RANGE(0x7000, 0x7003) AM_DEVREADWRITE("ppi8255", i8255_device, read, write)
	AM_RANGE(0x7020, 0x7027) AM_WRITE(eas_segment_w) AM_READNOP
	AM_RANGE(0x7030, 0x7037) AM_WRITE(eas_led_w) AM_READNOP
	AM_RANGE(0x7050, 0x7050) AM_READ(eas_input_r)
	AM_RANGE(0x8000, 0x9fff) AM_ROM
	AM_RANGE(0xc000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( eag_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x1fff) AM_RAM AM_SHARE("nvram")
	AM_RANGE(0x2000, 0x5fff) AM_READ(sc12_cart_r)
	AM_RANGE(0x7000, 0x7003) AM_DEVREADWRITE("ppi8255", i8255_device, read, write)
	AM_RANGE(0x7020, 0x7027) AM_WRITE(eas_segment_w) AM_READNOP
	AM_RANGE(0x7030, 0x7037) AM_WRITE(eas_led_w) AM_READNOP
	AM_RANGE(0x7050, 0x7050) AM_READ(eas_input_r)
	AM_RANGE(0x8000, 0xffff) AM_ROM //AM_WRITENOP
ADDRESS_MAP_END


// SC9

static ADDRESS_MAP_START( sc9_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x07ff) AM_MIRROR(0x1800) AM_RAM
	AM_RANGE(0x2000, 0x5fff) AM_READ(sc12_cart_r)
	AM_RANGE(0x6000, 0x6000) AM_MIRROR(0x1fff) AM_WRITE(sc9_control_w)
	AM_RANGE(0x8000, 0x8007) AM_MIRROR(0x1ff8) AM_WRITE(sc9_led_w) AM_READNOP
	AM_RANGE(0xa000, 0xa000) AM_MIRROR(0x1fff) AM_READ(sc9_input_r)
	AM_RANGE(0xc000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( sc9d_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0xa000, 0xa007) AM_MIRROR(0x1ff8) AM_READ(sc12_input_r)
	AM_IMPORT_FROM( sc9_map )
ADDRESS_MAP_END


// SC12

static ADDRESS_MAP_START( sc12_map, AS_PROGRAM, 8, fidel6502_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x0fff) AM_RAM
	AM_RANGE(0x2000, 0x5fff) AM_READ(sc12_cart_r)
	AM_RANGE(0x6000, 0x6000) AM_MIRROR(0x1fff) AM_WRITE(sc12_control_w)
	AM_RANGE(0x8000, 0x9fff) AM_ROM
	AM_RANGE(0xa000, 0xa007) AM_MIRROR(0x1ff8) AM_READ(sc12_input_r)
	AM_RANGE(0xc000, 0xcfff) AM_MIRROR(0x1000) AM_ROM
	AM_RANGE(0xe000, 0xffff) AM_ROM
ADDRESS_MAP_END


// Excellence

static ADDRESS_MAP_START( fexcel_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x0000, 0x07ff) AM_MIRROR(0x3800) AM_RAM
	AM_RANGE(0x4000, 0x4007) AM_MIRROR(0x3ff8) AM_READWRITE(fexcel_ttl_r, fexcel_ttl_w)
	//AM_RANGE(0x8000, 0x8000) AM_READNOP // checks for opening book module, but hw doesn't have a module slot
	AM_RANGE(0xc000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( fexcelp_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x0000, 0x1fff) AM_MIRROR(0x2000) AM_RAM
	AM_RANGE(0x4000, 0x4007) AM_MIRROR(0x3ff8) AM_READWRITE(fexcel_ttl_r, fexcel_ttl_w)
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( fexcelb_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x0000, 0x1fff) AM_MIRROR(0x2000) AM_RAM
	AM_RANGE(0x4000, 0x4007) AM_MIRROR(0x3ff8) AM_READWRITE(fexcelb_ttl_r, fexcel_ttl_w)
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END


// Designer Display, Phantom, Chesster

static ADDRESS_MAP_START( fdesdis_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x0000, 0x1fff) AM_RAM
	AM_RANGE(0x2000, 0x2007) AM_MIRROR(0x1ff8) AM_READWRITE(fdesdis_input_r, fdesdis_control_w)
	AM_RANGE(0x4000, 0x7fff) AM_ROMBANK("bank1")
	AM_RANGE(0x6000, 0x6007) AM_MIRROR(0x1ff8) AM_WRITE(fdesdis_lcd_w)
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( fphantom_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x0000, 0x1fff) AM_RAM
	AM_RANGE(0x4000, 0x7fff) AM_ROMBANK("bank1")
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( chesster_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x0000, 0x1fff) AM_RAM
	AM_RANGE(0x2000, 0x2007) AM_MIRROR(0x1ff8) AM_READWRITE(fdesdis_input_r, chesster_control_w)
	AM_RANGE(0x4000, 0x7fff) AM_ROMBANK("bank1")
	AM_RANGE(0x6000, 0x6000) AM_MIRROR(0x1fff) AM_DEVWRITE("dac8", dac_byte_interface, write)
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( kishon_map, AS_PROGRAM, 8, fidel6502_state )
	AM_RANGE(0x2000, 0x2007) AM_MIRROR(0x1ff8) AM_READWRITE(fdesdis_input_r, kishon_control_w)
	AM_IMPORT_FROM( chesster_map )
ADDRESS_MAP_END



/******************************************************************************
    Input Ports
******************************************************************************/

static INPUT_PORTS_START( rsc )
	PORT_INCLUDE( fidel_cb_buttons )

	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("ST")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("RV")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("DM")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("CL")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("LV")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("PV")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_SPACE) PORT_NAME("Speaker")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_R) PORT_NAME("RE")
INPUT_PORTS_END


static INPUT_PORTS_START( csc )
	PORT_INCLUDE( fidel_cb_buttons )

	PORT_MODIFY("IN.0")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_SPACE) PORT_NAME("Speaker")

	PORT_MODIFY("IN.1")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_V) PORT_NAME("RV")

	PORT_MODIFY("IN.2")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_T) PORT_NAME("TM")

	PORT_MODIFY("IN.3")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_L) PORT_NAME("LV")

	PORT_MODIFY("IN.4")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_M) PORT_NAME("DM")

	PORT_MODIFY("IN.5")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_S) PORT_NAME("ST")

	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("Pawn")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("Rook")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("Knight")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("Bishop")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("Queen")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("King")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DEL) PORT_NAME("CL")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_R) PORT_NAME("RE")

	PORT_START("IN.9") // hardwired
	PORT_CONFNAME( 0x01, 0x00, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
	PORT_CONFNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_CONFSETTING(    0x00, DEF_STR( Off ) )
	PORT_CONFSETTING(    0x02, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( cscg )
	PORT_INCLUDE( csc )

	PORT_MODIFY("IN.9")
	PORT_CONFNAME( 0x01, 0x01, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
INPUT_PORTS_END


static INPUT_PORTS_START( su9 )
	PORT_INCLUDE( csc )

	PORT_MODIFY("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("RV / Pawn")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("DM / Knight")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("TB / Bishop")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("LV / Rook")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("PV / Queen")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("PB / King")
INPUT_PORTS_END

static INPUT_PORTS_START( su9g )
	PORT_INCLUDE( su9 )

	PORT_MODIFY("IN.9")
	PORT_CONFNAME( 0x01, 0x01, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
INPUT_PORTS_END


static INPUT_PORTS_START( eas )
	PORT_INCLUDE( fidel_cb_magnets )

	PORT_MODIFY("IN.0")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_M) PORT_NAME("DM")

	PORT_MODIFY("IN.1")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DEL) PORT_NAME("CL")

	PORT_MODIFY("IN.2")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_V) PORT_NAME("RV")

	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_G) PORT_NAME("Game Control")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_SPACE) PORT_NAME("Speaker")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("PB / King")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("PV / Queen")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("TM / Rook")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("ST / Bishop")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("TB / Knight")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("LV / Pawn")

	PORT_START("IN.9") // hardwired
	PORT_CONFNAME( 0x01, 0x00, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
	PORT_CONFNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_CONFSETTING(    0x00, DEF_STR( Off ) )
	PORT_CONFSETTING(    0x02, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( easg )
	PORT_INCLUDE( eas )

	PORT_MODIFY("IN.9")
	PORT_CONFNAME( 0x01, 0x01, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
INPUT_PORTS_END


static INPUT_PORTS_START( eag )
	PORT_INCLUDE( fidel_cb_magnets )

	PORT_MODIFY("IN.0")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DEL) PORT_NAME("CL")

	PORT_MODIFY("IN.1")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_M) PORT_NAME("DM")

	PORT_MODIFY("IN.2")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_R) PORT_CODE(KEYCODE_N) PORT_NAME("New Game")

	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_V) PORT_NAME("RV")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_O) PORT_NAME("Option")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("LV / Pawn")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("TB / Knight")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("ST / Bishop")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("TM / Rook")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("PV / Queen")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("PB / King")

	PORT_START("IN.9") // hardwired
	PORT_CONFNAME( 0x01, 0x00, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
	PORT_CONFNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_CONFSETTING(    0x00, DEF_STR( Off ) )
	PORT_CONFSETTING(    0x02, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( eagg )
	PORT_INCLUDE( eag )

	PORT_MODIFY("IN.9")
	PORT_CONFNAME( 0x01, 0x01, DEF_STR( Language ) )
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, "Other" )
INPUT_PORTS_END


static INPUT_PORTS_START( sc12_base )
	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("RV / Pawn")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("DM / Knight")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("TB / Bishop")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("LV / Rook")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("PV / Queen")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("PB / King")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DEL) PORT_NAME("CL")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_R) PORT_NAME("RE")
INPUT_PORTS_END

static INPUT_PORTS_START( sc12 )
	PORT_INCLUDE( fidel_cb_buttons )
	PORT_INCLUDE( sc12_base )
INPUT_PORTS_END

static INPUT_PORTS_START( playmatic )
	PORT_INCLUDE( fidel_cb_magnets )
	PORT_INCLUDE( sc12_base )
INPUT_PORTS_END

static INPUT_PORTS_START( sc9c )
	PORT_INCLUDE( sc12 )

	PORT_START("FAKE")
	PORT_CONFNAME( 0x03, 0x00, "CPU Frequency" ) PORT_CHANGED_MEMBER(DEVICE_SELF, fidel6502_state, sc9c_cpu_freq, nullptr) // factory set
	PORT_CONFSETTING(    0x00, "1.5MHz" )
	PORT_CONFSETTING(    0x01, "1.6MHz" )
	PORT_CONFSETTING(    0x02, "1.9MHz" )
INPUT_PORTS_END

INPUT_CHANGED_MEMBER(fidel6502_state::sc9c_cpu_freq)
{
	sc9c_set_cpu_freq();
}


static INPUT_PORTS_START( fexcelb )
	PORT_INCLUDE( fidel_cb_buttons )

	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DEL) PORT_NAME("Clear")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("Move / Pawn")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("Hint / Knight")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("Take Back / Bishop")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("Level / Rook")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("Options / Queen")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("Verify / King")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_R) PORT_CODE(KEYCODE_N) PORT_NAME("New Game")
INPUT_PORTS_END

static INPUT_PORTS_START( fexcelv )
	PORT_INCLUDE( fexcelb )

	PORT_START("IN.9")
	PORT_CONFNAME( 0x03, 0x00, DEF_STR( Language ) ) PORT_CHANGED_MEMBER(DEVICE_SELF, fidel6502_state, fexcelv_bankswitch, 0)
	PORT_CONFSETTING(    0x00, DEF_STR( English ) )
	PORT_CONFSETTING(    0x01, DEF_STR( German ) )
	PORT_CONFSETTING(    0x02, DEF_STR( French ) )
	PORT_CONFSETTING(    0x03, DEF_STR( Spanish ) )
	PORT_BIT(0x7c, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_SPECIAL) PORT_READ_LINE_DEVICE_MEMBER("speech", s14001a_device, busy_r)
INPUT_PORTS_END

static INPUT_PORTS_START( fexcel )
	PORT_INCLUDE( fexcelb )

	PORT_START("IN.9")
	PORT_CONFNAME( 0x80, 0x00, "Battery Status" )
	PORT_CONFSETTING(    0x80, "Low" )
	PORT_CONFSETTING(    0x00, DEF_STR( Normal ) )
INPUT_PORTS_END

static INPUT_PORTS_START( fdes )
	PORT_INCLUDE( fexcel )

	PORT_MODIFY("IN.9")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED) // no low-voltage detection circuit (still works in software though)
INPUT_PORTS_END


static INPUT_PORTS_START( fdesdis )
	PORT_INCLUDE( fidel_cb_buttons )

	PORT_START("IN.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DEL) PORT_NAME("Clear")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_H) PORT_NAME("Move / Alternate")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_G) PORT_NAME("Hint / Info")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_F) PORT_NAME("Take Back / Replay")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_C) PORT_NAME("Level / New")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_B) PORT_NAME("Option / Time")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_A) PORT_NAME("Verify / Problem")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("Shift")
INPUT_PORTS_END

static INPUT_PORTS_START( chesster )
	PORT_INCLUDE( fdesdis )

	PORT_MODIFY("IN.8")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_H) PORT_NAME("Move / No")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_G) PORT_NAME("Hint / Yes")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_F) PORT_NAME("Take Back / Repeat")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_B) PORT_NAME("Option / Replay")
INPUT_PORTS_END


static INPUT_PORTS_START( fphantom )
	PORT_INCLUDE( fidel_cb_buttons )
INPUT_PORTS_END



/******************************************************************************
    Machine Drivers
******************************************************************************/

static MACHINE_CONFIG_START( rsc )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, 1800000) // measured approx 1.81MHz
	MCFG_CPU_PROGRAM_MAP(rsc_map)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(546)) // from 555 timer, measured
	MCFG_TIMER_START_DELAY(attotime::from_hz(546) - attotime::from_usec(38)) // active for 38us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(546))

	MCFG_DEVICE_ADD("pia", PIA6821, 0) // MOS 6520
	MCFG_PIA_READPA_HANDLER(READ8(fidel6502_state, csc_pia1_pa_r))
	MCFG_PIA_READCA1_HANDLER(READLINE(fidel6502_state, csc_pia1_ca1_r))
	MCFG_PIA_READCB1_HANDLER(READLINE(fidel6502_state, csc_pia1_cb1_r))
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(fidel6502_state, csc_pia1_pa_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(fidel6502_state, csc_pia1_pb_w))
	MCFG_PIA_CA2_HANDLER(WRITELINE(fidel6502_state, csc_pia1_ca2_w))
	MCFG_PIA_CB2_HANDLER(WRITELINE(fidel6502_state, csc_pia1_cb2_w))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_rsc_v2)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( csc )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, XTAL_3_9MHz/2) // from 3.9MHz resonator
	MCFG_CPU_PROGRAM_MAP(csc_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(fidel6502_state, irq0_line_hold, 600) // 38400kHz/64

	MCFG_DEVICE_ADD("pia0", PIA6821, 0)
	MCFG_PIA_READPB_HANDLER(READ8(fidel6502_state, csc_pia0_pb_r))
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(fidel6502_state, csc_pia0_pa_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(fidel6502_state, csc_pia0_pb_w))
	MCFG_PIA_CA2_HANDLER(WRITELINE(fidel6502_state, csc_pia0_ca2_w))

	MCFG_DEVICE_ADD("pia1", PIA6821, 0)
	MCFG_PIA_READPA_HANDLER(READ8(fidel6502_state, csc_pia1_pa_r))
	MCFG_PIA_READCA1_HANDLER(READLINE(fidel6502_state, csc_pia1_ca1_r))
	MCFG_PIA_READCB1_HANDLER(READLINE(fidel6502_state, csc_pia1_cb1_r))
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(fidel6502_state, csc_pia1_pa_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(fidel6502_state, csc_pia1_pb_w))
	MCFG_PIA_CA2_HANDLER(WRITELINE(fidel6502_state, csc_pia1_ca2_w))
	MCFG_PIA_CB2_HANDLER(WRITELINE(fidel6502_state, csc_pia1_cb2_w))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_csc)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("speech", S14001A, 25000) // R/C circuit, around 25khz
	MCFG_S14001A_EXT_READ_HANDLER(READ8(fidel6502_state, csc_speech_r))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.75)

	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( su9, csc )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(su9_map)

	MCFG_DEFAULT_LAYOUT(layout_fidel_su9)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( eas )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", R65C02, XTAL_3MHz)
	MCFG_CPU_PROGRAM_MAP(eas_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(fidel6502_state, irq0_line_hold, 600) // guessed

	MCFG_DEVICE_ADD("ppi8255", I8255, 0) // port B: input, port A & C: output
	MCFG_I8255_OUT_PORTA_CB(WRITE8(fidel6502_state, eas_ppi_porta_w))
	MCFG_I8255_TRISTATE_PORTA_CB(CONSTANT(0))
	MCFG_I8255_IN_PORTB_CB(READ8(fidel6502_state, eas_ppi_portb_r))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(fidel6502_state, eas_ppi_portc_w))

	MCFG_NVRAM_ADD_0FILL("nvram")

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_eas)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("speech", S14001A, 25000) // R/C circuit, around 25khz
	MCFG_S14001A_EXT_READ_HANDLER(READ8(fidel6502_state, csc_speech_r))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.75)

	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)

	/* cartridge */
	MCFG_GENERIC_CARTSLOT_ADD("cartslot", generic_plain_slot, "fidel_scc")
	MCFG_GENERIC_EXTENSIONS("bin,dat")
	MCFG_GENERIC_LOAD(fidelbase_state, scc_cartridge)
	MCFG_SOFTWARE_LIST_ADD("cart_list", "fidel_scc")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( eag, eas )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", R65C02, XTAL_5MHz) // R65C02P4
	MCFG_CPU_PROGRAM_MAP(eag_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(fidel6502_state, irq0_line_hold, 600) // guessed

	MCFG_DEFAULT_LAYOUT(layout_fidel_eag)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( sc9d )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, XTAL_3_9MHz/2) // R6502AP, 3.9MHz resonator
	MCFG_CPU_PROGRAM_MAP(sc9d_map)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(610)) // from 555 timer (22nF, 102K, 2.7K)
	MCFG_TIMER_START_DELAY(attotime::from_hz(610) - attotime::from_usec(41)) // active for 41us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(610))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_sc9)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)

	/* cartridge */
	MCFG_GENERIC_CARTSLOT_ADD("cartslot", generic_plain_slot, "fidel_scc")
	MCFG_GENERIC_EXTENSIONS("bin,dat")
	MCFG_GENERIC_LOAD(fidelbase_state, scc_cartridge)
	MCFG_SOFTWARE_LIST_ADD("cart_list", "fidel_scc")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sc9b, sc9d )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", M6502, 1500000) // from ceramic resonator "681 JSA", measured
	MCFG_CPU_PROGRAM_MAP(sc9_map)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sc9c, sc9b )

	/* basic machine hardware */
	MCFG_MACHINE_RESET_OVERRIDE(fidel6502_state, sc9c)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( playmatic, sc9b )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_DEVICE_CLOCK(3100000) // approximation

	MCFG_DEFAULT_LAYOUT(layout_fidel_playmatic)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( sc12 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", R65C02, XTAL_4MHz)
	MCFG_CPU_PROGRAM_MAP(sc12_map)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(596)) // from 556 timer (22nF, 82K+26K, 1K)
	MCFG_TIMER_START_DELAY(attotime::from_hz(596) - attotime::from_nsec(15250)) // active for 15.25us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(596))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_sc12)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)

	/* cartridge */
	MCFG_GENERIC_CARTSLOT_ADD("cartslot", generic_plain_slot, "fidel_scc")
	MCFG_GENERIC_EXTENSIONS("bin,dat")
	MCFG_GENERIC_LOAD(fidelbase_state, scc_cartridge)
	MCFG_SOFTWARE_LIST_ADD("cart_list", "fidel_scc")
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( fexcel )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M65SC02, XTAL_12MHz/4) // G65SC102P-3, 12.0M ceramic resonator
	MCFG_CPU_PROGRAM_MAP(fexcel_map)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(630)) // from 556 timer (22nF, 102K, 1K)
	MCFG_TIMER_START_DELAY(attotime::from_hz(630) - attotime::from_nsec(15250)) // active for 15.25us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(630))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_ex)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fexcel4, fexcel )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", R65C02, XTAL_4MHz) // R65C02P4
	MCFG_CPU_PROGRAM_MAP(fexcel_map)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fexcelb, fexcel )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(fexcelb_map)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fexcelp, fexcel )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", R65C02, XTAL_5MHz) // R65C02P4
	MCFG_CPU_PROGRAM_MAP(fexcelp_map)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( granits, fexcelp )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_DEVICE_CLOCK(XTAL_8MHz) // overclocked
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fdes2000, fexcel )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", R65C02, XTAL_3MHz) // RP65C02G
	MCFG_CPU_PROGRAM_MAP(fexcelp_map)

	// change irq timer frequency
	MCFG_DEVICE_REMOVE("irq_on")
	MCFG_DEVICE_REMOVE("irq_off")
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(585)) // from 556 timer (22nF, 110K, 1K)
	MCFG_TIMER_START_DELAY(attotime::from_hz(585) - attotime::from_nsec(15250)) // active for 15.25us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(585))

	MCFG_DEFAULT_LAYOUT(layout_fidel_des)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fdes2100, fdes2000 )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_DEVICE_CLOCK(XTAL_5MHz)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fexcelv, fexcelb )

	/* sound hardware */
	MCFG_SOUND_ADD("speech", S14001A, 25000) // R/C circuit, around 25khz
	MCFG_S14001A_EXT_READ_HANDLER(READ8(fidel6502_state, fexcelv_speech_r))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.75)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fexceld, fexcelb )

	/* basic machine hardware */
	MCFG_DEFAULT_LAYOUT(layout_fidel_exd)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( fdes2100d )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M65C02, XTAL_6MHz) // W65C02P-6
	MCFG_CPU_PROGRAM_MAP(fdesdis_map)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(630)) // from 556 timer (22nF, 102K, 1K)
	MCFG_TIMER_START_DELAY(attotime::from_hz(630) - attotime::from_nsec(15250)) // active for 15.25us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(630))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_desdis)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( fdes2000d, fdes2100d )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", R65C02, XTAL_3MHz) // R65C02P3
	MCFG_CPU_PROGRAM_MAP(fdesdis_map)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( fphantom )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", R65C02, XTAL_4_9152MHz) // R65C02P4
	MCFG_CPU_PERIODIC_INT_DRIVER(fidel6502_state, irq0_line_hold, 600) // guessed
	MCFG_CPU_PROGRAM_MAP(fphantom_map)

	MCFG_MACHINE_RESET_OVERRIDE(fidel6502_state, fphantom)

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	//MCFG_DEFAULT_LAYOUT(layout_fidel_phantom)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_1BIT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( chesster )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", R65C02, XTAL_5MHz) // RP65C02G
	MCFG_CPU_PROGRAM_MAP(chesster_map)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_on", fidel6502_state, irq_on, attotime::from_hz(9615)) // R/C circuit, measured
	MCFG_TIMER_START_DELAY(attotime::from_hz(9615) - attotime::from_nsec(2600)) // active for 2.6us
	MCFG_TIMER_DRIVER_ADD_PERIODIC("irq_off", fidel6502_state, irq_off, attotime::from_hz(9615))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", fidelbase_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_fidel_chesster)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac8", DAC_8BIT_R2R, 0) // m74hc374b1.ic1 + 8l513_02.z2
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac8", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac8", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( kishon, chesster )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_DEVICE_CLOCK(XTAL_3_579545MHz)
	MCFG_CPU_PROGRAM_MAP(kishon_map)
MACHINE_CONFIG_END



/******************************************************************************
    ROM Definitions
******************************************************************************/

ROM_START( reversic )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1000a01", 0xf000, 0x1000, CRC(ca7723a7) SHA1(bd92330f2d9494fa408f5a2ca300d7a755bdf489) )
ROM_END


ROM_START( csc )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-64109.bin", 0x2000, 0x2000, CRC(08a3577c) SHA1(69fe379d21a9d4b57c84c3832d7b3e7431eec341) )
	ROM_LOAD("1025a03.bin",   0xa000, 0x2000, CRC(63982c07) SHA1(5ed4356323d5c80df216da55994abe94ba4aa94c) )
	ROM_LOAD("1025a02.bin",   0xc000, 0x2000, CRC(9e6e7c69) SHA1(4f1ed9141b6596f4d2b1217d7a4ba48229f3f1b0) )
	ROM_LOAD("1025a01.bin",   0xe000, 0x2000, CRC(57f068c3) SHA1(7d2ac4b9a2fba19556782863bdd89e2d2d94e97b) )
	ROM_LOAD("74s474.bin",    0xfe00, 0x0200, CRC(4511ba31) SHA1(e275b1739f8c3aa445cccb6a2b597475f507e456) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("101-32107.bin", 0x0000, 0x1000, CRC(f35784f9) SHA1(348e54a7fa1e8091f89ac656b4da22f28ca2e44d) )
	ROM_RELOAD(               0x1000, 0x1000)
ROM_END

ROM_START( cscsp )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-64109.bin", 0x2000, 0x2000, CRC(08a3577c) SHA1(69fe379d21a9d4b57c84c3832d7b3e7431eec341) )
	ROM_LOAD("1025a03.bin",   0xa000, 0x2000, CRC(63982c07) SHA1(5ed4356323d5c80df216da55994abe94ba4aa94c) )
	ROM_LOAD("1025a02.bin",   0xc000, 0x2000, CRC(9e6e7c69) SHA1(4f1ed9141b6596f4d2b1217d7a4ba48229f3f1b0) )
	ROM_LOAD("1025a01.bin",   0xe000, 0x2000, CRC(57f068c3) SHA1(7d2ac4b9a2fba19556782863bdd89e2d2d94e97b) )
	ROM_LOAD("74s474.bin",    0xfe00, 0x0200, CRC(4511ba31) SHA1(e275b1739f8c3aa445cccb6a2b597475f507e456) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-spanish.bin", 0x0000, 0x2000, BAD_DUMP CRC(8766e128) SHA1(78c7413bf240159720b131ab70bfbdf4e86eb1e9) ) // taken from vcc/fexcelv, assume correct
ROM_END

ROM_START( cscg )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-64109.bin", 0x2000, 0x2000, CRC(08a3577c) SHA1(69fe379d21a9d4b57c84c3832d7b3e7431eec341) )
	ROM_LOAD("1025a03.bin",   0xa000, 0x2000, CRC(63982c07) SHA1(5ed4356323d5c80df216da55994abe94ba4aa94c) )
	ROM_LOAD("1025a02.bin",   0xc000, 0x2000, CRC(9e6e7c69) SHA1(4f1ed9141b6596f4d2b1217d7a4ba48229f3f1b0) )
	ROM_LOAD("1025a01.bin",   0xe000, 0x2000, CRC(57f068c3) SHA1(7d2ac4b9a2fba19556782863bdd89e2d2d94e97b) )
	ROM_LOAD("74s474.bin",    0xfe00, 0x0200, CRC(4511ba31) SHA1(e275b1739f8c3aa445cccb6a2b597475f507e456) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-german.bin", 0x0000, 0x2000, BAD_DUMP CRC(6c85e310) SHA1(20d1d6543c1e6a1f04184a2df2a468f33faec3ff) ) // taken from fexcelv, assume correct
ROM_END

ROM_START( cscfr )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-64109.bin", 0x2000, 0x2000, CRC(08a3577c) SHA1(69fe379d21a9d4b57c84c3832d7b3e7431eec341) )
	ROM_LOAD("1025a03.bin",   0xa000, 0x2000, CRC(63982c07) SHA1(5ed4356323d5c80df216da55994abe94ba4aa94c) )
	ROM_LOAD("1025a02.bin",   0xc000, 0x2000, CRC(9e6e7c69) SHA1(4f1ed9141b6596f4d2b1217d7a4ba48229f3f1b0) )
	ROM_LOAD("1025a01.bin",   0xe000, 0x2000, CRC(57f068c3) SHA1(7d2ac4b9a2fba19556782863bdd89e2d2d94e97b) )
	ROM_LOAD("74s474.bin",    0xfe00, 0x0200, CRC(4511ba31) SHA1(e275b1739f8c3aa445cccb6a2b597475f507e456) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-french.bin", 0x0000, 0x2000, BAD_DUMP CRC(fe8c5c18) SHA1(2b64279ab3747ee81c86963c13e78321c6cfa3a3) ) // taken from fexcelv, assume correct
ROM_END


ROM_START( super9cc )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("cb9.bin",     0x2000, 0x2000, CRC(421147e8) SHA1(ccf62f6f218e8992baf30973fe41b35e14a1cc1a) )
	ROM_LOAD("101-1024b03", 0xa000, 0x0800, CRC(e8c97455) SHA1(ed2958fc5474253ee8c2eaf27fc64226e12f80ea) )
	ROM_LOAD("101-1024b02", 0xc000, 0x2000, CRC(95004699) SHA1(ea79f43da73267344545df8ad61730f613876c2e) )
	ROM_LOAD("101-1024c01", 0xe000, 0x2000, CRC(03904e86) SHA1(bfa0dd9d8541e3ec359a247a3eba543501f727bc) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-english.bin", 0x0000, 0x1000, BAD_DUMP CRC(f35784f9) SHA1(348e54a7fa1e8091f89ac656b4da22f28ca2e44d) ) // taken from csc, assume correct
	ROM_RELOAD(                 0x1000, 0x1000)
ROM_END

ROM_START( super9ccsp )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("cb9.bin",     0x2000, 0x2000, CRC(421147e8) SHA1(ccf62f6f218e8992baf30973fe41b35e14a1cc1a) )
	ROM_LOAD("101-1024b03", 0xa000, 0x0800, CRC(e8c97455) SHA1(ed2958fc5474253ee8c2eaf27fc64226e12f80ea) )
	ROM_LOAD("101-1024b02", 0xc000, 0x2000, CRC(95004699) SHA1(ea79f43da73267344545df8ad61730f613876c2e) )
	ROM_LOAD("101-1024c01", 0xe000, 0x2000, CRC(03904e86) SHA1(bfa0dd9d8541e3ec359a247a3eba543501f727bc) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-spanish.bin", 0x0000, 0x2000, BAD_DUMP CRC(8766e128) SHA1(78c7413bf240159720b131ab70bfbdf4e86eb1e9) ) // taken from vcc/fexcelv, assume correct
ROM_END

ROM_START( super9ccg )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("cb9.bin",     0x2000, 0x2000, CRC(421147e8) SHA1(ccf62f6f218e8992baf30973fe41b35e14a1cc1a) )
	ROM_LOAD("101-1024b03", 0xa000, 0x0800, CRC(e8c97455) SHA1(ed2958fc5474253ee8c2eaf27fc64226e12f80ea) )
	ROM_LOAD("101-1024b02", 0xc000, 0x2000, CRC(95004699) SHA1(ea79f43da73267344545df8ad61730f613876c2e) )
	ROM_LOAD("101-1024c01", 0xe000, 0x2000, CRC(03904e86) SHA1(bfa0dd9d8541e3ec359a247a3eba543501f727bc) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-german.bin", 0x0000, 0x2000, BAD_DUMP CRC(6c85e310) SHA1(20d1d6543c1e6a1f04184a2df2a468f33faec3ff) ) // taken from fexcelv, assume correct
ROM_END

ROM_START( super9ccfr )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("cb9.bin",     0x2000, 0x2000, CRC(421147e8) SHA1(ccf62f6f218e8992baf30973fe41b35e14a1cc1a) )
	ROM_LOAD("101-1024b03", 0xa000, 0x0800, CRC(e8c97455) SHA1(ed2958fc5474253ee8c2eaf27fc64226e12f80ea) )
	ROM_LOAD("101-1024b02", 0xc000, 0x2000, CRC(95004699) SHA1(ea79f43da73267344545df8ad61730f613876c2e) )
	ROM_LOAD("101-1024c01", 0xe000, 0x2000, CRC(03904e86) SHA1(bfa0dd9d8541e3ec359a247a3eba543501f727bc) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-french.bin", 0x0000, 0x2000, BAD_DUMP CRC(fe8c5c18) SHA1(2b64279ab3747ee81c86963c13e78321c6cfa3a3) ) // taken from fexcelv, assume correct
ROM_END


ROM_START( feasbu )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_bu.6", 0x8000, 0x0800, CRC(93dcc23b) SHA1(2eb8c5a85e566948bc256d6b1804694e6b0ffa6f) ) // ST M27C64A, unknown label
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("101-1052a02.4", 0xc000, 0x2000, CRC(859d69f1) SHA1(a8b057683369e2387f22fc7e916b6f3c75d44b21) ) // Mostek MK36C63N-5
	ROM_LOAD("101-1052a01.5", 0xe000, 0x2000, CRC(571a33a7) SHA1(43b110cf0918caf16643178f401e58b2dc73894f) ) // Mostek MK36C63N-5

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("101-32107.bin", 0x0000, 0x1000, CRC(f35784f9) SHA1(348e54a7fa1e8091f89ac656b4da22f28ca2e44d) ) // NEC D2332C
	ROM_RELOAD(               0x1000, 0x1000)
ROM_END

ROM_START( feasbusp )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_bu.6", 0x8000, 0x0800, CRC(93dcc23b) SHA1(2eb8c5a85e566948bc256d6b1804694e6b0ffa6f) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("101-1052a02.4", 0xc000, 0x2000, CRC(859d69f1) SHA1(a8b057683369e2387f22fc7e916b6f3c75d44b21) )
	ROM_LOAD("101-1052a01.5", 0xe000, 0x2000, CRC(571a33a7) SHA1(43b110cf0918caf16643178f401e58b2dc73894f) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-spanish.bin", 0x0000, 0x2000, BAD_DUMP CRC(8766e128) SHA1(78c7413bf240159720b131ab70bfbdf4e86eb1e9) ) // taken from vcc/fexcelv, assume correct
ROM_END

ROM_START( feasbug )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_bu.6", 0x8000, 0x0800, CRC(93dcc23b) SHA1(2eb8c5a85e566948bc256d6b1804694e6b0ffa6f) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("101-1052a02.4", 0xc000, 0x2000, CRC(859d69f1) SHA1(a8b057683369e2387f22fc7e916b6f3c75d44b21) )
	ROM_LOAD("101-1052a01.5", 0xe000, 0x2000, CRC(571a33a7) SHA1(43b110cf0918caf16643178f401e58b2dc73894f) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-german.bin", 0x0000, 0x2000, BAD_DUMP CRC(6c85e310) SHA1(20d1d6543c1e6a1f04184a2df2a468f33faec3ff) ) // taken from fexcelv, assume correct
ROM_END

ROM_START( feasbufr )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_bu.6", 0x8000, 0x0800, CRC(93dcc23b) SHA1(2eb8c5a85e566948bc256d6b1804694e6b0ffa6f) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("101-1052a02.4", 0xc000, 0x2000, CRC(859d69f1) SHA1(a8b057683369e2387f22fc7e916b6f3c75d44b21) )
	ROM_LOAD("101-1052a01.5", 0xe000, 0x2000, CRC(571a33a7) SHA1(43b110cf0918caf16643178f401e58b2dc73894f) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-french.bin", 0x0000, 0x2000, BAD_DUMP CRC(fe8c5c18) SHA1(2b64279ab3747ee81c86963c13e78321c6cfa3a3) ) // taken from fexcelv, assume correct
ROM_END

ROM_START( feasgla )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_gla.6", 0x8000, 0x0800, CRC(2fdddb4f) SHA1(6da0a328a45462f285ae6a0756f97c5a43148f97) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("eli_gla.4", 0xc000, 0x0800, CRC(f094e625) SHA1(fef84c6a3da504aac15988ec9af94417e5fedfbd) )
	ROM_CONTINUE( 0xd000, 0x0800 )
	ROM_CONTINUE( 0xc800, 0x0800 )
	ROM_CONTINUE( 0xd800, 0x0800 )
	ROM_LOAD("eli_gla.5", 0xe000, 0x0800, CRC(5f6845d1) SHA1(684eb16faf36a49560e5a73b55fd0022dc090e35) )
	ROM_CONTINUE( 0xf000, 0x0800 )
	ROM_CONTINUE( 0xe800, 0x0800 )
	ROM_CONTINUE( 0xf800, 0x0800 )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("101-32107.bin", 0x0000, 0x1000, CRC(f35784f9) SHA1(348e54a7fa1e8091f89ac656b4da22f28ca2e44d) ) // NEC D2332C
	ROM_RELOAD(               0x1000, 0x1000)
ROM_END

ROM_START( feasglasp )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_gla.6", 0x8000, 0x0800, CRC(2fdddb4f) SHA1(6da0a328a45462f285ae6a0756f97c5a43148f97) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("eli_gla.4", 0xc000, 0x0800, CRC(f094e625) SHA1(fef84c6a3da504aac15988ec9af94417e5fedfbd) )
	ROM_CONTINUE( 0xd000, 0x0800 )
	ROM_CONTINUE( 0xc800, 0x0800 )
	ROM_CONTINUE( 0xd800, 0x0800 )
	ROM_LOAD("eli_gla.5", 0xe000, 0x0800, CRC(5f6845d1) SHA1(684eb16faf36a49560e5a73b55fd0022dc090e35) )
	ROM_CONTINUE( 0xf000, 0x0800 )
	ROM_CONTINUE( 0xe800, 0x0800 )
	ROM_CONTINUE( 0xf800, 0x0800 )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-spanish.bin", 0x0000, 0x2000, BAD_DUMP CRC(8766e128) SHA1(78c7413bf240159720b131ab70bfbdf4e86eb1e9) ) // taken from vcc/fexcelv, assume correct
ROM_END

ROM_START( feasglag )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_gla.6", 0x8000, 0x0800, CRC(2fdddb4f) SHA1(6da0a328a45462f285ae6a0756f97c5a43148f97) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("eli_gla.4", 0xc000, 0x0800, CRC(f094e625) SHA1(fef84c6a3da504aac15988ec9af94417e5fedfbd) )
	ROM_CONTINUE( 0xd000, 0x0800 )
	ROM_CONTINUE( 0xc800, 0x0800 )
	ROM_CONTINUE( 0xd800, 0x0800 )
	ROM_LOAD("eli_gla.5", 0xe000, 0x0800, CRC(5f6845d1) SHA1(684eb16faf36a49560e5a73b55fd0022dc090e35) )
	ROM_CONTINUE( 0xf000, 0x0800 )
	ROM_CONTINUE( 0xe800, 0x0800 )
	ROM_CONTINUE( 0xf800, 0x0800 )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-german.bin", 0x0000, 0x2000, BAD_DUMP CRC(6c85e310) SHA1(20d1d6543c1e6a1f04184a2df2a468f33faec3ff) ) // taken from fexcelv, assume correct
ROM_END

ROM_START( feasglafr )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("eli_gla.6", 0x8000, 0x0800, CRC(2fdddb4f) SHA1(6da0a328a45462f285ae6a0756f97c5a43148f97) )
	ROM_CONTINUE( 0x9000, 0x0800 )
	ROM_CONTINUE( 0x8800, 0x0800 )
	ROM_CONTINUE( 0x9800, 0x0800 )
	ROM_LOAD("eli_gla.4", 0xc000, 0x0800, CRC(f094e625) SHA1(fef84c6a3da504aac15988ec9af94417e5fedfbd) )
	ROM_CONTINUE( 0xd000, 0x0800 )
	ROM_CONTINUE( 0xc800, 0x0800 )
	ROM_CONTINUE( 0xd800, 0x0800 )
	ROM_LOAD("eli_gla.5", 0xe000, 0x0800, CRC(5f6845d1) SHA1(684eb16faf36a49560e5a73b55fd0022dc090e35) )
	ROM_CONTINUE( 0xf000, 0x0800 )
	ROM_CONTINUE( 0xe800, 0x0800 )
	ROM_CONTINUE( 0xf800, 0x0800 )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-french.bin", 0x0000, 0x2000, BAD_DUMP CRC(fe8c5c18) SHA1(2b64279ab3747ee81c86963c13e78321c6cfa3a3) ) // taken from fexcelv, assume correct
ROM_END


ROM_START( feag2100 )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("el2100.1", 0x8000, 0x8000, CRC(9b62b7d5) SHA1(cfcaea2e36c2d52fe4a85c77dbc7fa135893860c) )
	ROM_LOAD("el2100.2", 0xc000, 0x2000, CRC(76fec42f) SHA1(34660edb8458919fd179e93fdab3fe428a6625d0) )
	ROM_LOAD("el2100.3", 0xe000, 0x2000, CRC(2079a506) SHA1(a7bb83138c7b6eff6ea96702d453a214697f4890) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-english.bin", 0x0000, 0x1000, BAD_DUMP CRC(f35784f9) SHA1(348e54a7fa1e8091f89ac656b4da22f28ca2e44d) ) // taken from csc, assume correct
	ROM_RELOAD(                 0x1000, 0x1000)
ROM_END

ROM_START( feag2100sp )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("el2100.1", 0x8000, 0x8000, CRC(9b62b7d5) SHA1(cfcaea2e36c2d52fe4a85c77dbc7fa135893860c) )
	ROM_LOAD("el2100.2", 0xc000, 0x2000, CRC(76fec42f) SHA1(34660edb8458919fd179e93fdab3fe428a6625d0) )
	ROM_LOAD("el2100.3", 0xe000, 0x2000, CRC(2079a506) SHA1(a7bb83138c7b6eff6ea96702d453a214697f4890) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-spanish.bin", 0x0000, 0x2000, BAD_DUMP CRC(8766e128) SHA1(78c7413bf240159720b131ab70bfbdf4e86eb1e9) ) // taken from vcc/fexcelv, assume correct
ROM_END

ROM_START( feag2100g )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("el2100.1", 0x8000, 0x8000, CRC(9b62b7d5) SHA1(cfcaea2e36c2d52fe4a85c77dbc7fa135893860c) )
	ROM_LOAD("el2100.2", 0xc000, 0x2000, CRC(76fec42f) SHA1(34660edb8458919fd179e93fdab3fe428a6625d0) )
	ROM_LOAD("el2100.3", 0xe000, 0x2000, CRC(2079a506) SHA1(a7bb83138c7b6eff6ea96702d453a214697f4890) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-german.bin", 0x0000, 0x2000, BAD_DUMP CRC(6c85e310) SHA1(20d1d6543c1e6a1f04184a2df2a468f33faec3ff) ) // taken from fexcelv, assume correct
ROM_END

ROM_START( feag2100fr )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("el2100.1", 0x8000, 0x8000, CRC(9b62b7d5) SHA1(cfcaea2e36c2d52fe4a85c77dbc7fa135893860c) )
	ROM_LOAD("el2100.2", 0xc000, 0x2000, CRC(76fec42f) SHA1(34660edb8458919fd179e93fdab3fe428a6625d0) )
	ROM_LOAD("el2100.3", 0xe000, 0x2000, CRC(2079a506) SHA1(a7bb83138c7b6eff6ea96702d453a214697f4890) )

	ROM_REGION( 0x2000, "speech", 0 )
	ROM_LOAD("vcc-french.bin", 0x0000, 0x2000, BAD_DUMP CRC(fe8c5c18) SHA1(2b64279ab3747ee81c86963c13e78321c6cfa3a3) ) // taken from fexcelv, assume correct
ROM_END


ROM_START( fscc9 ) // PCB label 510-1046D01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1034b01", 0xc000, 0x2000, CRC(65288753) SHA1(651f5ca5969ddd72a20cbebdec2de83c4bf10650) )
	ROM_LOAD("101-1034c02", 0xe000, 0x2000, CRC(238b092f) SHA1(7ddffc6dba822aee9d8ad6815b23024ed5cdfd26) )
ROM_END

ROM_START( fscc9b ) // PCB label 510-1046B01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1034a01", 0xc000, 0x2000, CRC(b845c458) SHA1(d3fda65dbd9fae44fa4b93f8207839d8fa0c367a) )
	ROM_LOAD("101-1034a02", 0xe000, 0x2000, CRC(ecfa0a4c) SHA1(738df99a250fad0b1da5ebeb8c92a9ad1461417b) )
ROM_END

ROM_START( fscc9c ) // PCB label 510-1046C01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1034a01", 0xc000, 0x2000, CRC(b845c458) SHA1(d3fda65dbd9fae44fa4b93f8207839d8fa0c367a) ) // HN48364P
	ROM_LOAD("101-1034b02", 0xe000, 0x2000, CRC(cbaf97d7) SHA1(7ed8e68bb74713d9e2ff1d9c037012320b7bfcbf) ) // "
ROM_END

ROM_START( fscc9ps )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("play64c1.bin", 0xc000, 0x2000, CRC(e96aa95d) SHA1(16d90cf0ef166aef579d442671290a2c43e24dfe) )
	ROM_LOAD("play64en.bin", 0xe000, 0x2000, CRC(6fa188d2) SHA1(1b9b0209c496c89ecb7f9ec07bfd9429ff9b275e) )
ROM_END


ROM_START( fscc12 ) // model 6086, PCB label 510-1084B01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1068a01",   0x8000, 0x2000, CRC(63c76cdd) SHA1(e0771c98d4483a6b1620791cb99a7e46b0db95c4) ) // SSS SCM23C65E4
	ROM_LOAD("tms2732ajl-45", 0xc000, 0x1000, CRC(45070a71) SHA1(8aeecff828f26fb7081902c757559903be272649) ) // TI TMS2732AJL-45
	ROM_LOAD("tmm2764d-2",    0xe000, 0x2000, CRC(183d3edc) SHA1(3296a4c3bce5209587d4a1694fce153558544e63) ) // Toshiba TMM2764D-2
ROM_END


ROM_START( fexcel ) // model 6080(B), PCB label 510.1117A02
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1080a01.ic5", 0x8000, 0x8000, CRC(846f8e40) SHA1(4e1d5b08d5ff3422192b54fa82cb3f505a69a971) )
ROM_END

ROM_START( fexceld ) // model 6093, PCB label 510.1117A02
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1080a01.ic5", 0x8000, 0x8000, CRC(846f8e40) SHA1(4e1d5b08d5ff3422192b54fa82cb3f505a69a971) ) // same rom as fexcel
ROM_END

ROM_START( fexcelv ) // model 6092, PCB label 510.1117A02, sound PCB 510.1117A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1080a01.ic5", 0x8000, 0x8000, CRC(846f8e40) SHA1(4e1d5b08d5ff3422192b54fa82cb3f505a69a971) ) // PCB1, M27256, same rom as fexcel

	ROM_REGION( 0x8000, "speech", 0 )
	ROM_LOAD("101-1081a01.ic2", 0x0000, 0x8000, CRC(c8ae1607) SHA1(6491ce6be60ed77f3dd931c0ca17616f13af943e) ) // PCB2, M27256
ROM_END

ROM_START( fexcel12 ) // model EP12, PCB label 510-1099A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1072a01.ic5", 0xc000, 0x4000, CRC(212b006d) SHA1(242ff851b0841cbec66bbada6a730da021010e2c) )
ROM_END

ROM_START( fexcel124 ) // model EP12, PCB label 510-1099A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1073a01.ic5", 0xc000, 0x4000, CRC(3e221534) SHA1(7516bc6a8aab9d8ac30ac1a9317630a6aa9ac1a0) )
ROM_END

ROM_START( fexcela ) // model 6080, PCB label 510-1099A01(manuf.1985) or 510-1099B01(manuf.1986)
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1072b01.ic5", 0xc000, 0x4000, CRC(fd2f6064) SHA1(f84bb98bdb9565a04891eb6820597d7aecc90c21) ) // RCA
ROM_END


ROM_START( fexcelp ) // model 6083, PCB label 510-1099A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1077a01.ic5", 0x8000, 0x8000, CRC(62006320) SHA1(1d6370973dbae42c54639b261cc81e32cdfc1d5d) )
ROM_END

ROM_START( fexcelpb ) // model 6083, PCB label 510-1099B01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("par_ex.ic5", 0x8000, 0x8000, CRC(0d17b0f0) SHA1(3a6070fd4718c62b62ff0f08637bb6eb84eb9a1c) ) // GI 27C256, no label, only 1 byte difference, assume bugfix in bookrom
ROM_END

ROM_START( granits ) // modified SC12 board, overclocked Par Excellence program
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("granit_s-4", 0x8000, 0x8000, CRC(274d6aff) SHA1(c8d943b2f15422ac62f539b568f5509cbce568a3) )
ROM_END

ROM_START( fdes2000 ) // model 6102, PCB label 510.1129A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1077a01.ic5", 0x8000, 0x8000, CRC(62006320) SHA1(1d6370973dbae42c54639b261cc81e32cdfc1d5d) ) // AMI, same label as fexcelp
ROM_END

ROM_START( fdes2100 ) // model 6103, PCB label 510.1129A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("101-1077a01.ic5", 0x8000, 0x8000, CRC(62006320) SHA1(1d6370973dbae42c54639b261cc81e32cdfc1d5d) ) // same as fdes2000
ROM_END


ROM_START( fdes2100d ) // model 6106, PCB label 510.1130A01. The 'rev B' dump came from a post-release bugfix by Fidelity
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("i9_orange.ic9", 0x8000, 0x8000, CRC(83fec02a) SHA1(6f43ab05bc605061989b05d0592dbd184efff9d4) ) // WSI 27C256L-12

	ROM_REGION( 0x8000, "user1", 0 )
	ROM_LOAD("bk3_white.ic10", 0x0000, 0x8000, CRC(3857cc35) SHA1(f073dafb9fd885c7ddb7fbff10e3653f343ef1c6) ) // WSI 27C256L-12
ROM_END

ROM_START( fdes2000d ) // model 6105, PCB label 510.1130A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("27c256.ic9", 0x8000, 0x8000, CRC(b136d1a1) SHA1(8438790a62f45284ff33a0255c5c89f526726d3e) ) // 27C256, no label

	ROM_REGION( 0x8000, "user1", ROMREGION_ERASEFF ) // no rom in ic10
ROM_END


ROM_START( fphantom ) // model 6100, PCB label 510.1128A01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("u_3c_yellow.u3", 0x8000, 0x8000, CRC(fb7c38ae) SHA1(a1aa7637705052cb4eec92644dc79aee7ba4d77c) ) // 27C256

	ROM_REGION( 0x8000, "user1", 0 )
	ROM_LOAD("u_4_white.u4",  0x0000, 0x8000, CRC(e4181ba2) SHA1(1f77d1867c6f566be98645fc252a01108f412c96) ) // 27C256
ROM_END


ROM_START( chesster ) // model 6120, PCB label 510.1141C01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("ch_1.3.ic9", 0x8000, 0x8000, CRC(8b42d1ad) SHA1(2161fc5ab2476fe7ca4ffc226e3cb329b8a57a01) ) // 27256, CH 1.3 on sticker

	ROM_REGION( 0x20000, "user1", 0 )
	ROM_LOAD("101-1091b02.ic10", 0x0000, 0x20000, CRC(fa370e88) SHA1(a937c8f1ec295cf9539d12466993974e40771493) ) // AMI, 27C010 or equivalent
ROM_END

ROM_START( chesstera ) // model 6120, PCB label 510.1141C01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("chesster.ic9", 0x8000, 0x8000, CRC(29f9a698) SHA1(4c83ca46fd5fc9c40302e9c7f16b4ae2c18b06e6) ) // M27C256B, sticker but no label

	ROM_REGION( 0x20000, "user1", 0 )
	ROM_LOAD("101-1091a02.ic10", 0x0000, 0x20000, CRC(2b4d243c) SHA1(921e51978facb502b207b4f64a73b1e74127e826) ) // AMI, 27C010 or equivalent
ROM_END

ROM_START( kishon ) // model 6120G or 6127(same), PCB label 510.1141C01
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD("kishon.ic9", 0x8000, 0x8000, CRC(121c007f) SHA1(652e9ea47b6bb1632d10eb0fcd7f98cdba22fce7) ) // 27C256

	ROM_REGION( 0x80000, "user1", 0 )
	ROM_LOAD("kishon_v2.6_1-14-91.ic10", 0x0000, 0x80000, CRC(50598869) SHA1(2087e0c2f40a2408fe217a6502c8c3a247bdd063) ) // Toshiba TC544000P-12, aka 101-1094A01
ROM_END



/******************************************************************************
    Drivers
******************************************************************************/

//    YEAR  NAME        PARENT   CMP MACHINE    INPUT      STATE            INIT      COMPANY, FULLNAME, FLAGS
CONS( 1981, reversic,   0,        0, rsc,       rsc,       fidel6502_state, 0,        "Fidelity Electronics", "Reversi Sensory Challenger (green version)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1981, csc,        0,        0, csc,       csc,       fidel6502_state, 0,        "Fidelity Electronics", "Champion Sensory Chess Challenger (English)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1981, cscsp,      csc,      0, csc,       cscg,      fidel6502_state, 0,        "Fidelity Electronics", "Champion Sensory Chess Challenger (Spanish)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1981, cscg,       csc,      0, csc,       cscg,      fidel6502_state, 0,        "Fidelity Electronics", "Champion Sensory Chess Challenger (German)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1981, cscfr,      csc,      0, csc,       cscg,      fidel6502_state, 0,        "Fidelity Electronics", "Champion Sensory Chess Challenger (French)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1983, super9cc,   0,        0, su9,       su9,       fidel6502_state, 0,        "Fidelity Electronics", "Super 9 Sensory Chess Challenger (English)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, super9ccsp, super9cc, 0, su9,       su9g,      fidel6502_state, 0,        "Fidelity Electronics", "Super 9 Sensory Chess Challenger (Spanish)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, super9ccg,  super9cc, 0, su9,       su9g,      fidel6502_state, 0,        "Fidelity Electronics", "Super 9 Sensory Chess Challenger (German)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, super9ccfr, super9cc, 0, su9,       su9g,      fidel6502_state, 0,        "Fidelity Electronics", "Super 9 Sensory Chess Challenger (French)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1983, feasbu,     0,        0, eas,       eas,       fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Budapest program, English)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, feasbusp,   feasbu,   0, eas,       easg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Budapest program, Spanish)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, feasbug,    feasbu,   0, eas,       easg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Budapest program, German)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, feasbufr,   feasbu,   0, eas,       easg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Budapest program, French)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1984, feasgla,    feasbu,   0, eas,       eas,       fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Glasgow program, English)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1984, feasglasp,  feasbu,   0, eas,       easg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Glasgow program, Spanish)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1984, feasglag,   feasbu,   0, eas,       easg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Glasgow program, German)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1984, feasglafr,  feasbu,   0, eas,       easg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite A/S Challenger (Glasgow program, French)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1986, feag2100,   0,        0, eag,       eag,       fidel6502_state, 0,        "Fidelity Electronics", "Elite Avant Garde 2100 (English)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1986, feag2100sp, feag2100, 0, eag,       eagg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite Avant Garde 2100 (Spanish)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1986, feag2100g,  feag2100, 0, eag,       eagg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite Avant Garde 2100 (German)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1986, feag2100fr, feag2100, 0, eag,       eagg,      fidel6502_state, 0,        "Fidelity Electronics", "Elite Avant Garde 2100 (French)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1982, fscc9,      0,        0, sc9d,      sc12,      fidel6502_state, 0,        "Fidelity Electronics", "Sensory Chess Challenger 9 (rev. D)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK ) // aka version "B"
CONS( 1982, fscc9b,     fscc9,    0, sc9b,      sc12,      fidel6502_state, 0,        "Fidelity Electronics", "Sensory Chess Challenger 9 (rev. B)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1982, fscc9c,     fscc9,    0, sc9c,      sc9c,      fidel6502_state, 0,        "Fidelity Electronics", "Sensory Chess Challenger 9 (rev. C)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1983, fscc9ps,    fscc9,    0, playmatic, playmatic, fidel6502_state, 0,        "Fidelity Electronics", "Sensory 9 Playmatic 'S'", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK ) // Fidelity West Germany

CONS( 1984, fscc12,     0,        0, sc12,      sc12,      fidel6502_state, 0,        "Fidelity Electronics", "Sensory Chess Challenger 12-B", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1987, fexcel,     0,        0, fexcelb,   fexcelb,   fidel6502_state, 0,        "Fidelity Electronics", "The Excellence (model 6080B)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1987, fexcelv,    fexcel,   0, fexcelv,   fexcelv,   fidel6502_state, 0,        "Fidelity Electronics", "Voice Excellence", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1987, fexceld,    fexcel,   0, fexceld,   fexcelb,   fidel6502_state, 0,        "Fidelity Electronics", "Excel Display", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1985, fexcel12,   fexcel,   0, fexcel,    fexcel,    fidel6502_state, 0,        "Fidelity Electronics", "The Excellence (model EP12, set 1)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK ) // 1st version of The Excellence
CONS( 1985, fexcel124,  fexcel,   0, fexcel4,   fexcel,    fidel6502_state, 0,        "Fidelity Electronics", "The Excellence (model EP12, set 2)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1985, fexcela,    fexcel,   0, fexcel,    fexcel,    fidel6502_state, 0,        "Fidelity Electronics", "The Excellence (model 6080)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1986, fexcelp,    0,        0, fexcelp,   fexcel,    fidel6502_state, 0,        "Fidelity Electronics", "The Par Excellence", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1986, fexcelpb,   fexcelp,  0, fexcelp,   fexcel,    fidel6502_state, 0,        "Fidelity Electronics", "The Par Excellence (rev. B)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1986, granits,    fexcelp,  0, granits,   fexcel,    fidel6502_state, 0,        "hack (RCS)", "Granit 'S'", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1988, fdes2000,   fexcelp,  0, fdes2000,  fdes,      fidel6502_state, 0,        "Fidelity Electronics", "Designer 2000", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK ) // Excellence series hardware
CONS( 1988, fdes2100,   fexcelp,  0, fdes2100,  fdes,      fidel6502_state, 0,        "Fidelity Electronics", "Designer 2100", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK ) // "

CONS( 1988, fdes2100d,  0,        0, fdes2100d, fdesdis,   fidel6502_state, fdesdis,  "Fidelity Electronics", "Designer 2100 Display (rev. B)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1988, fdes2000d,  fdes2100d,0, fdes2000d, fdesdis,   fidel6502_state, fdesdis,  "Fidelity Electronics", "Designer 2000 Display", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1988, fphantom,   0,        0, fphantom,  fphantom,  fidel6502_state, fphantom, "Fidelity Electronics", "Phantom (Fidelity)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK | MACHINE_MECHANICAL | MACHINE_NOT_WORKING )

CONS( 1990, chesster,   0,        0, chesster,  chesster,  fidel6502_state, chesster, "Fidelity Electronics", "Chesster Challenger (V1.3)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1990, chesstera,  chesster, 0, chesster,  chesster,  fidel6502_state, chesster, "Fidelity Electronics", "Chesster Challenger", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
CONS( 1991, kishon,     chesster, 0, kishon,    chesster,  fidel6502_state, chesster, "Fidelity Electronics", "Kishon Chesster", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )
