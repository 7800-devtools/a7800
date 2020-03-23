// license:BSD-3-Clause
// copyright-holders:hap, Sean Riddle
/***************************************************************************

  ** subclass of hh_tms1k_state (includes/hh_tms1k.h, drivers/hh_tms1k.cpp) **

  Texas Instruments or 1st-party TMS1xxx family handheld calculators (mostly
  single-chip). For a comprehensive list of MCU serials, see Joerg Woerner's
  datamath.org: http://www.datamath.org/IC_List.htm

  Refer to the calculators/toys official manuals on how to use them.


  TODO:
  - MCU clocks are unknown where noted
  - fake-press ON button when emulation starts for calculators that have
    it on the button matrix (doesn't look like any relies on it though)

***************************************************************************/

#include "emu.h"
#include "includes/hh_tms1k.h"

#include "speaker.h"

// internal artwork
#include "cmulti8.lh"
#include "dataman.lh"
#include "mathmarv.lh"
#include "ti1250.lh"
#include "ti1270.lh"
#include "ti25503.lh"
#include "ti30.lh"
#include "tisr16.lh"
#include "wizatron.lh"


class ticalc1x_state : public hh_tms1k_state
{
public:
	ticalc1x_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_tms1k_state(mconfig, type, tag)
	{ }
};



/***************************************************************************

  Minidrivers (subclass, I/O, Inputs, Machine Config)

***************************************************************************/

/***************************************************************************

  Canon Multi 8 (Palmtronic MD-8) / Canon Canola MD 810
  * TMS1070 MCU label TMC1079 (die label 1070B, 1079A)
  * 2-line cyan VFD display, each 9-digit 7seg + 1 custom (label 20-ST-22)
  * PCB label Canon EHI-0115-03

***************************************************************************/

class cmulti8_state : public ticalc1x_state
{
public:
	cmulti8_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

void cmulti8_state::prepare_display()
{
	set_display_segmask(0xfffff, 0xff);

	// M-digit is on in memory mode, upper row is off in single mode
	u32 m = (m_inp_matrix[10]->read() & 0x10) ? 0x100000 : 0;
	u32 mask = (m_inp_matrix[10]->read() & 0x20) ? 0xfffff : 0xffc00;

	// R10 selects display row
	u32 sel = (m_r & 0x400) ? (m_r & 0x3ff) : (m_r << 10 & 0xffc00);
	display_matrix(8, 21, m_o, (sel & mask) | m);
}

WRITE16_MEMBER(cmulti8_state::write_r)
{
	// R0-R10: input mux, select digit
	m_r = m_inp_mux = data;
	prepare_display();
}

WRITE16_MEMBER(cmulti8_state::write_o)
{
	// O0-O7: digit segments
	m_o = BITSWAP8(data,0,4,5,6,7,1,2,3);
	prepare_display();
}

READ8_MEMBER(cmulti8_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(11);
}


// config

static INPUT_PORTS_START( cmulti8 )
	PORT_START("IN.0") // R0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("% +/-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")

	PORT_START("IN.1") // R1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_X) PORT_NAME("RM") // recall memory
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")

	PORT_START("IN.2") // R2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("SC") // sign change
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")

	PORT_START("IN.3") // R3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_Z) PORT_NAME("CM") // clear memory
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")

	PORT_START("IN.4") // R4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_R) PORT_NAME(UTF8_SQUAREROOT)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")

	PORT_START("IN.5") // R5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_C) PORT_NAME("M+") // add to memory
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")

	PORT_START("IN.6") // R6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_V) PORT_NAME("RV") // reverse
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")

	PORT_START("IN.7") // R7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")

	PORT_START("IN.8") // R8
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")

	PORT_START("IN.9") // R9
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_DEL) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("CI/C")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")

	PORT_START("IN.10") // R10
	PORT_CONFNAME( 0x31, 0x20, "Mode" ) // bit 4 indicates M-digit on/off, bit 5 indicates upper row filament on/off
	PORT_CONFSETTING(    0x31, "Memory" )
	PORT_CONFSETTING(    0x01, "Single" )
	PORT_CONFSETTING(    0x20, "Process" )
	PORT_CONFNAME( 0x02, 0x00, "AM" ) // accumulate memory
	PORT_CONFSETTING(    0x00, DEF_STR( Off ) )
	PORT_CONFSETTING(    0x02, DEF_STR( On ) )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED )
INPUT_PORTS_END

static MACHINE_CONFIG_START( cmulti8 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1070, 250000) // approximation - RC osc. R=56K, C=68pf
	MCFG_TMS1XXX_READ_K_CB(READ8(cmulti8_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(cmulti8_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(cmulti8_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_cmulti8)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI SR-16 (1974, first consumer product with TMS1000 series MCU)
  * TMS1000 MCU label TMS1001NL (die label 1000, 1001A)
  * 12-digit 7seg LED display

  TI SR-16 II (1975 version)
  * TMS1000 MCU label TMS1016NL (die label 1000B, 1016A)
  * notes: cost-reduced 'sequel', [10^x] was removed, and [pi] was added.

***************************************************************************/

class tisr16_state : public ticalc1x_state
{
public:
	tisr16_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

void tisr16_state::prepare_display()
{
	// update leds state
	set_display_segmask(0xfff, 0xff);
	display_matrix(8, 12, m_o, m_r, false);

	// exponent sign is from R10 O1, and R10 itself only uses segment G
	m_display_state[11] = m_display_state[10] << 5 & 0x40;
	m_display_state[10] &= 0x40;
	display_update();
}

WRITE16_MEMBER(tisr16_state::write_r)
{
	// R0-R10: input mux, select digit
	m_r = m_inp_mux = data;
	prepare_display();
}

WRITE16_MEMBER(tisr16_state::write_o)
{
	// O0-O7: digit segments
	m_o = data;
	prepare_display();
}

READ8_MEMBER(tisr16_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(11);
}


// config

static INPUT_PORTS_START( tisr16 )
	PORT_START("IN.0") // R0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("RCL")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")

	PORT_START("IN.1") // R1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("CE")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")

	PORT_START("IN.2") // R2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")

	PORT_START("IN.3") // R3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")

	PORT_START("IN.4") // R4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_TILDE) PORT_NAME("EE")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")

	PORT_START("IN.5") // R5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_EQUALS) PORT_NAME(UTF8_CAPITAL_SIGMA)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("STO")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")

	PORT_START("IN.6") // R6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_X) PORT_NAME("1/x")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Y) PORT_NAME("y" UTF8_POW_X)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")

	PORT_START("IN.7") // R7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Q) PORT_NAME("x" UTF8_POW_2)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")

	PORT_START("IN.8") // R8
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_T) PORT_NAME("10" UTF8_POW_X)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_E) PORT_NAME("e" UTF8_POW_X)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")

	PORT_START("IN.9") // R9
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_R) PORT_NAME(UTF8_SQUAREROOT"x")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")

	PORT_START("IN.10") // R10
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_DEL) PORT_NAME("C")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_O) PORT_NAME("log")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_L) PORT_NAME("ln(x)")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED )
INPUT_PORTS_END

static INPUT_PORTS_START( tisr16ii )
	PORT_START("IN.0") // R0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_DEL) PORT_NAME("C")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("IN.1") // R1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("CD")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")

	PORT_START("IN.2") // R2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")

	PORT_START("IN.3") // R3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")

	PORT_START("IN.4") // R4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Q) PORT_NAME("x" UTF8_POW_2)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")

	PORT_START("IN.5") // R5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_O) PORT_NAME("log")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_TILDE) PORT_NAME("EE")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")

	PORT_START("IN.6") // R6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_R) PORT_NAME(UTF8_SQUAREROOT"x")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")

	PORT_START("IN.7") // R7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_L) PORT_NAME("ln(x)")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("STO")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")

	PORT_START("IN.8") // R8
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_X) PORT_NAME("1/x")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("RCL")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")

	PORT_START("IN.9") // R9
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_E) PORT_NAME("e" UTF8_POW_X)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_EQUALS) PORT_NAME(UTF8_CAPITAL_SIGMA)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")

	PORT_START("IN.10") // R10
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Y) PORT_NAME("y" UTF8_POW_X)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_P) PORT_NAME(UTF8_SMALL_PI)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
INPUT_PORTS_END

static MACHINE_CONFIG_START( tisr16 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1000, 300000) // approximation - RC osc. R=43K, C=68pf (note: tisr16ii MCU RC osc. is different: R=30K, C=100pf, same freq)
	MCFG_TMS1XXX_READ_K_CB(READ8(tisr16_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(tisr16_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(tisr16_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_tisr16)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI-1250/TI-1200 (1975 version), Spirit of '76
  * TMS0950 MCU label TMC0952NL, K0952 (die label 0950A 0952)
  * 9-digit 7seg LED display

  TI-1250/TI-1200 (1976 version), TI-1400, TI-1450, TI-1205, TI-1255, LADY 1200, ABLE
  * TMS0970 MCU label TMS0972NL ZA0348, JP0972A (die label 0970D-72A)
  * 8-digit 7seg LED display, or 9 digits with leftmost unused

  As seen listed above, the basic 4-function TMS0972 calculator MCU was used
  in many calculators. It was licensed to other manufacturers too, one funny
  example being a Mattel Barbie handheld calculator.

  Some cheaper models lacked the memory buttons (the function itself still works).
  The ABLE series was for educational purposes, with each having a small subset of
  available buttons.

  TI-1270
  * TMS0970 MCU label TMC0974NL ZA0355, DP0974A (die label 0970D-74A)
  * 8-digit 7seg LED display
  * notes: almost same hardware as TMS0972 TI-1250, minor scientific functions

***************************************************************************/

class ti1250_state : public ticalc1x_state
{
public:
	ti1250_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

WRITE16_MEMBER(ti1250_state::write_r)
{
	// R0-R8: select digit
	set_display_segmask(0xff, 0xff);
	set_display_segmask(0x100, 0x40); // R8 only has segment G connected
	display_matrix(8, 9, m_o, data);
}

WRITE16_MEMBER(ti1250_state::write_o)
{
	// O1-O5,O7: input mux
	// O0-O7: digit segments
	m_inp_mux = (data >> 1 & 0x1f) | (data >> 2 & 0x20);
	m_o = data;
}

READ8_MEMBER(ti1250_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(6);
}


// config

static INPUT_PORTS_START( ti1250 )
	PORT_START("IN.0") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("CE")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")

	PORT_START("IN.1") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")

	PORT_START("IN.2") // O3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")

	PORT_START("IN.3") // O4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)

	PORT_START("IN.4") // O5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_DEL) PORT_NAME("C")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("CS") // named either CS(Change Sign) or +/-
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("%")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)

	PORT_START("IN.5") // O7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_C) PORT_NAME("MC")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("MR")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_INSERT) PORT_NAME("M-")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("M+")
INPUT_PORTS_END

static INPUT_PORTS_START( ti1270 )
	PORT_INCLUDE( ti1250 )

	PORT_MODIFY("IN.0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_BACKSPACE) PORT_CODE(KEYCODE_DEL) PORT_NAME("CE/C")

	PORT_MODIFY("IN.4")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("STO")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("RCL")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_P) PORT_NAME(UTF8_SMALL_PI)

	PORT_MODIFY("IN.5")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_X) PORT_NAME("1/x")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Q) PORT_NAME("x" UTF8_POW_2)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_R) PORT_NAME(UTF8_SQUAREROOT"x")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")
INPUT_PORTS_END

static MACHINE_CONFIG_START( ti1250 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS0950, 200000) // approximation - RC osc. R=68K, C=68pf
	MCFG_TMS1XXX_READ_K_CB(READ8(ti1250_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(ti1250_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(ti1250_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_ti1250)

	/* no sound! */
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( ti1270, ti1250 )

	/* basic machine hardware */
	MCFG_CPU_REPLACE("maincpu", TMS0970, 250000) // approximation
	MCFG_TMS1XXX_READ_K_CB(READ8(ti1250_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(ti1250_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(ti1250_state, write_r))

	MCFG_DEFAULT_LAYOUT(layout_ti1270)
MACHINE_CONFIG_END





/***************************************************************************

  TI-2550 III/TI-1265 (both have same chip)
  * TMS1040 MCU label TMS1043NL ZA0352 (die label 1040A, 1043A)
  * 9-digit cyan VFD display

***************************************************************************/

class ti25503_state : public ticalc1x_state
{
public:
	ti25503_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

void ti25503_state::prepare_display()
{
	set_display_segmask(0x1ff, 0xff);
	display_matrix(8, 9, m_o, m_r);
}

WRITE16_MEMBER(ti25503_state::write_r)
{
	// R0-R6: input mux
	// R0-R8: select digit
	m_r = m_inp_mux = data;
	prepare_display();
}

WRITE16_MEMBER(ti25503_state::write_o)
{
	// O0-O7: digit segments
	m_o = data;
	prepare_display();
}

READ8_MEMBER(ti25503_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(7);
}


// config

static INPUT_PORTS_START( ti25503 )
	PORT_START("IN.0") // R0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("CE")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")

	PORT_START("IN.1") // R1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")

	PORT_START("IN.2") // R2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")

	PORT_START("IN.3") // R3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)

	PORT_START("IN.4") // R4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_DEL) PORT_NAME("C")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("%")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)

	PORT_START("IN.5") // R5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_C) PORT_NAME("CM")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_END) PORT_NAME("MR")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_INSERT) PORT_NAME("M-")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("M+")

	PORT_START("IN.6") // R6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_V) PORT_NAME("RV")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_R) PORT_NAME(UTF8_SQUAREROOT"x")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_Q) PORT_NAME("x" UTF8_POW_2)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_X) PORT_NAME("1/x")
INPUT_PORTS_END

static MACHINE_CONFIG_START( ti25503 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1000, 250000) // approximation
	MCFG_TMS1XXX_READ_K_CB(READ8(ti25503_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(ti25503_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(ti25503_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_ti25503)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI-1000 (1977 version)
  * TMS1990 MCU label TMC1991NL (die label 1991-91A)
  * 8-digit 7seg LED display

  TI-1000 (1978 version)
  * TMS1990 MCU label TMC1992-4NL **not dumped yet

***************************************************************************/

class ti1000_state : public ticalc1x_state
{
public:
	ti1000_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

WRITE16_MEMBER(ti1000_state::write_r)
{
	// R0-R7: select digit
	set_display_segmask(0xff, 0xff);
	display_matrix(8, 8, m_o, data);
}

WRITE16_MEMBER(ti1000_state::write_o)
{
	// O0-O3,O5(?): input mux
	// O0-O7: digit segments
	m_inp_mux = (data & 0xf) | (data >> 1 & 0x10);
	m_o = BITSWAP8(data,7,4,3,2,1,0,6,5);
}

READ8_MEMBER(ti1000_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(5);
}


// config

static INPUT_PORTS_START( ti1000 )
	PORT_START("IN.0") // O0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")

	PORT_START("IN.1") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")

	PORT_START("IN.2") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)

	// note: even though power buttons are on the matrix, they are not CPU-controlled
	PORT_START("IN.3") // O3 or O4?
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGDN) PORT_NAME("Off") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)false)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("%")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)

	PORT_START("IN.4") // O5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGUP) PORT_CODE(KEYCODE_DEL) PORT_NAME("On/C") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
INPUT_PORTS_END

static MACHINE_CONFIG_START( ti1000 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1990, 250000) // approximation
	MCFG_TMS1XXX_READ_K_CB(READ8(ti1000_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(ti1000_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(ti1000_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_ti1270)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI WIZ-A-TRON
  * TMS0970 MCU label TMC0907NL ZA0379, DP0907BS (die label 0970F-07B)
  * 9-digit 7seg LED display(one custom digit)

***************************************************************************/

class wizatron_state : public ticalc1x_state
{
public:
	wizatron_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	virtual DECLARE_WRITE16_MEMBER(write_o);
	virtual DECLARE_WRITE16_MEMBER(write_r);
	virtual DECLARE_READ8_MEMBER(read_k);
};

// handlers

WRITE16_MEMBER(wizatron_state::write_r)
{
	// 6th digit is custom(not 7seg), for math symbols, like this:
	//   \./    GAB
	//   ---     F
	//   /.\    EDC

	// 3rd digit only has A and G for =, though some newer hardware revisions
	// (goes for both wizatron and lilprof) use a custom equals-sign digit here
	set_display_segmask(8, 0x41);

	// R0-R8: select digit
	set_display_segmask(0x1ff^8, 0x7f);
	display_matrix(7, 9, m_o, data);
}

WRITE16_MEMBER(wizatron_state::write_o)
{
	// O1-O4: input mux
	// O0-O6: digit segments A-G
	// O7: N/C
	m_inp_mux = data >> 1 & 0xf;
	m_o = data & 0x7f;
}

READ8_MEMBER(wizatron_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(4);
}


// config

static INPUT_PORTS_START( wizatron )
	PORT_START("IN.0") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_DEL) PORT_NAME("Clear")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")

	PORT_START("IN.1") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")

	PORT_START("IN.2") // O3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)

	PORT_START("IN.3") // O4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
INPUT_PORTS_END

static MACHINE_CONFIG_START( wizatron )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS0970, 250000) // approximation
	MCFG_TMS1XXX_READ_K_CB(READ8(wizatron_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(wizatron_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(wizatron_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_wizatron)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI Little Professor (1976 version)
  * TMS0970 MCU label TMS0975NL ZA0356, GP0975CS (die label 0970D-75C)
  * 9-digit 7seg LED display(one custom digit)

  The hardware is nearly identical to Wiz-A-Tron (or vice versa, since this
  one is older).

***************************************************************************/

class lilprof_state : public wizatron_state
{
public:
	lilprof_state(const machine_config &mconfig, device_type type, const char *tag)
		: wizatron_state(mconfig, type, tag)
	{ }

	virtual DECLARE_WRITE16_MEMBER(write_o) override;
	virtual DECLARE_READ8_MEMBER(read_k) override;
};

// handlers

WRITE16_MEMBER(lilprof_state::write_o)
{
	// O1-O4,O7: input mux
	// O0-O6: digit segments A-G
	m_inp_mux = (data >> 1 & 0xf) | (data >> 3 & 0x10);
	m_o = data;
}

READ8_MEMBER(lilprof_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(5);
}


// config

static INPUT_PORTS_START( lilprof )
	PORT_INCLUDE( wizatron )

	PORT_MODIFY("IN.0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_S) PORT_CODE(KEYCODE_DEL) PORT_NAME("Set")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("Go")

	PORT_START("IN.4") // O7
	PORT_CONFNAME( 0x0f, 0x01, "Level")
	PORT_CONFSETTING(    0x01, "1" )
	PORT_CONFSETTING(    0x02, "2" )
	PORT_CONFSETTING(    0x04, "3" )
	PORT_CONFSETTING(    0x08, "4" )
INPUT_PORTS_END

static MACHINE_CONFIG_START( lilprof )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS0970, 250000) // approximation
	MCFG_TMS1XXX_READ_K_CB(READ8(lilprof_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(lilprof_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(wizatron_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_wizatron)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI Little Professor (1978 version)
  * TMS1990 MCU label TMC1993NL (die label 1990C-c3C)
  * 9-digit 7seg LED display(one custom digit)

  1978 re-release, with on/off and level select on buttons instead of
  switches. The casing was slightly revised in 1980 again, but same rom.

***************************************************************************/

class lilprof78_state : public ticalc1x_state
{
public:
	lilprof78_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

WRITE16_MEMBER(lilprof78_state::write_r)
{
	// update leds state
	u8 seg = BITSWAP8(m_o,7,4,3,2,1,0,6,5) & 0x7f;
	u16 r = (data & 7) | (data << 1 & 0x1f0);
	set_display_segmask(0x1ff, 0x7f);
	display_matrix(7, 9, seg, r, false);

	// 3rd digit A/G(equals sign) is from O7
	m_display_state[3] = (r != 0 && m_o & 0x80) ? 0x41 : 0;

	// 6th digit is a custom 7seg for math symbols (see wizatron_state write_r)
	m_display_state[6] = BITSWAP8(m_display_state[6],7,6,1,4,2,3,5,0);
	display_update();
}

WRITE16_MEMBER(lilprof78_state::write_o)
{
	// O0-O3,O5(?): input mux
	// O0-O6: digit segments A-G
	// O7: 6th digit
	m_inp_mux = (data & 0xf) | (data >> 1 & 0x10);
	m_o = data;
}

READ8_MEMBER(lilprof78_state::read_k)
{
	// K: multiplexed inputs
	return read_inputs(5);
}


// config

static INPUT_PORTS_START( lilprof78 )
	PORT_START("IN.0") // O0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")

	PORT_START("IN.1") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)

	PORT_START("IN.2") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)

	// note: even though power buttons are on the matrix, they are not CPU-controlled
	PORT_START("IN.3") // O3 or O4?
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PGDN) PORT_NAME("Off") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)false)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_S) PORT_NAME("Set")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_L) PORT_NAME("Level")

	PORT_START("IN.4") // O5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PGUP) PORT_CODE(KEYCODE_DEL) PORT_NAME("On") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("Go")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
INPUT_PORTS_END

static MACHINE_CONFIG_START( lilprof78 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1990, 250000) // approximation
	MCFG_TMS1XXX_READ_K_CB(READ8(lilprof78_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(lilprof78_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(lilprof78_state, write_r))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_wizatron)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI DataMan
  * TMS1980 MCU label TMC1982NL (die label 1980A 82B)
  * 10-digit cyan VFD display(3 digits are custom)

***************************************************************************/

class dataman_state : public ticalc1x_state
{
public:
	dataman_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	virtual void prepare_display();
	virtual DECLARE_WRITE16_MEMBER(write_o);
	virtual DECLARE_WRITE16_MEMBER(write_r);
	virtual DECLARE_READ8_MEMBER(read_k);
};

// handlers

void dataman_state::prepare_display()
{
	// note the extra segment on R9
	set_display_segmask(0x1ff, 0x7f);
	display_matrix(8, 9, m_o | (m_r >> 2 & 0x80), m_r & 0x1ff);
}

WRITE16_MEMBER(dataman_state::write_r)
{
	// R0-R4: input mux
	// R0-R8: select digit
	// R9: =(equals sign) segment
	m_r = m_inp_mux = data;
	prepare_display();
}

WRITE16_MEMBER(dataman_state::write_o)
{
	// O0-O6: digit segments A-G
	m_o = BITSWAP8(data,7,1,6,5,4,3,2,0) & 0x7f;
	prepare_display();
}

READ8_MEMBER(dataman_state::read_k)
{
	// K: multiplexed inputs (note: the Vss row is always on)
	return m_inp_matrix[5]->read() | read_inputs(5);
}


// config

static INPUT_PORTS_START( dataman )
	PORT_START("IN.0") // R0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_M) PORT_NAME("Memory Bank")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_G) PORT_NAME("Go")

	PORT_START("IN.1") // R1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")

	PORT_START("IN.2") // R2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")

	PORT_START("IN.3") // R3
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")

	PORT_START("IN.4") // R4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_F) PORT_NAME("Force Out")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_N) PORT_NAME("Number Guesser")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_W) PORT_NAME("Wipe Out")

	// note: even though power buttons are on the matrix, they are not CPU-controlled
	PORT_START("IN.5") // Vss!
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PGUP) PORT_CODE(KEYCODE_U) PORT_NAME("On/User Entry") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PGDN) PORT_NAME("Off") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)false)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("?")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_E) PORT_NAME("Electro Flash")
INPUT_PORTS_END

static MACHINE_CONFIG_START( dataman )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1980, 300000) // patent says 300kHz
	MCFG_TMS1XXX_READ_K_CB(READ8(dataman_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(dataman_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(dataman_state, write_r))
	MCFG_TMS1XXX_POWER_OFF_CB(WRITELINE(hh_tms1k_state, auto_power_off))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_dataman)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  TI Math Marvel
  * TMS1980 MCU label TMC1986A-NL (die label 1980A 86A)
  * 9-digit cyan VFD display(2 digits are custom), 1-bit sound

  This is the same hardware as DataMan, with R8 connected to a piezo.

***************************************************************************/

class mathmarv_state : public dataman_state
{
public:
	mathmarv_state(const machine_config &mconfig, device_type type, const char *tag)
		: dataman_state(mconfig, type, tag)
	{ }

	virtual DECLARE_WRITE16_MEMBER(write_r) override;
};

// handlers

WRITE16_MEMBER(mathmarv_state::write_r)
{
	// R8: speaker out
	m_speaker->level_w(data >> 8 & 1);

	// rest is same as dataman
	dataman_state::write_r(space, offset, data);
}


// config

static INPUT_PORTS_START( mathmarv )
	PORT_INCLUDE( dataman )

	PORT_MODIFY("IN.4") // R4
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_Q) PORT_NAME("Quest")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_C) PORT_NAME("Checker")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_R) PORT_NAME("Review")

	PORT_MODIFY("IN.5") // Vss!
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_PGUP) PORT_CODE(KEYCODE_N) PORT_NAME("On/Numberific") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_Z) PORT_NAME("Zap")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_CODE(KEYCODE_F) PORT_NAME("Flash")
INPUT_PORTS_END

static MACHINE_CONFIG_START( mathmarv )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS1980, 300000) // assume same as dataman
	MCFG_TMS1XXX_READ_K_CB(READ8(dataman_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(dataman_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(mathmarv_state, write_r))
	MCFG_TMS1XXX_POWER_OFF_CB(WRITELINE(hh_tms1k_state, auto_power_off))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_mathmarv)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  TMC098x series Majestic-line calculators

  TI-30, SR-40, TI-15(less buttons) and several by Koh-I-Noor
  * TMS0980 MCU label TMC0981NL (die label 0980B-81F)
  * 9-digit 7seg LED display

  Of note is a peripheral by Schoenherr, called the Braillotron. It acts as
  a docking station to the TI-30, with an additional display made of magnetic
  refreshable Braille cells. The TI-30 itself is slightly modified to wire
  the original LED display to a 25-pin D-Sub connector.

  TI Business Analyst, TI Business Analyst-I, TI Money Manager, TI-31, TI-41
  * TMS0980 MCU label TMC0982NL (die label 0980B-82F)

  TI Programmer
  * TMS0980 MCU label ZA0675NL, JP0983AT (die label 0980B-83)

***************************************************************************/

class ti30_state : public ticalc1x_state
{
public:
	ti30_state(const machine_config &mconfig, device_type type, const char *tag)
		: ticalc1x_state(mconfig, type, tag)
	{ }

	DECLARE_WRITE16_MEMBER(write_o);
	DECLARE_WRITE16_MEMBER(write_r);
	DECLARE_READ8_MEMBER(read_k);
};

// handlers

WRITE16_MEMBER(ti30_state::write_r)
{
	// R0-R8: select digit
	set_display_segmask(0x1fe, 0xff);
	set_display_segmask(0x001, 0xe2); // 1st digit only has segments B,F,G,DP
	display_matrix(8, 9, m_o, data);
}

WRITE16_MEMBER(ti30_state::write_o)
{
	// O0-O2,O4-O7: input mux
	// O0-O7: digit segments
	m_inp_mux = (data & 7) | (data >> 1 & 0x78);
	m_o = BITSWAP8(data,7,5,2,1,4,0,6,3);
}

READ8_MEMBER(ti30_state::read_k)
{
	// K: multiplexed inputs (note: the Vss row is always on)
	return m_inp_matrix[7]->read() | read_inputs(7);
}


// config

static INPUT_PORTS_START( ti30 )
	PORT_START("IN.0") // O0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Y) PORT_NAME("y" UTF8_POW_X)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_K) PORT_NAME("K")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_O) PORT_NAME("log")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_TILDE) PORT_NAME("EE" UTF8_DOWN)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_L) PORT_NAME("ln(x)")

	PORT_START("IN.1") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("STO")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")

	PORT_START("IN.2") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("RCL")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")

	PORT_START("IN.3") // O4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_P) PORT_NAME(UTF8_SMALL_PI)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_OPENBRACE) PORT_NAME("(")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("%")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_NAME(")")

	PORT_START("IN.4") // O5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_EQUALS) PORT_NAME("SUM")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")

	PORT_START("IN.5") // O6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_D) PORT_NAME("DRG")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_I) PORT_NAME("INV")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_C) PORT_NAME("cos")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_S) PORT_NAME("sin")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_T) PORT_NAME("tan")

	PORT_START("IN.6") // O7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_E) PORT_NAME("EXC")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")

	// note: even though power buttons are on the matrix, they are not CPU-controlled
	PORT_START("IN.7") // Vss!
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGUP) PORT_CODE(KEYCODE_DEL) PORT_NAME("On/C") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_X) PORT_NAME("1/x")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_R) PORT_NAME(UTF8_SQUAREROOT"x")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Q) PORT_NAME("x" UTF8_POW_2)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGDN) PORT_NAME("Off") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)false)
INPUT_PORTS_END

static INPUT_PORTS_START( tiprog )
	PORT_START("IN.0") // O0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_K) PORT_NAME("K")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("SHF")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_E) PORT_NAME("E")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_D) PORT_NAME("d")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_F) PORT_NAME("F")

	PORT_START("IN.1") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_O) PORT_NAME("OR")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")

	PORT_START("IN.2") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_N) PORT_NAME("AND")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")

	PORT_START("IN.3") // O4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_TILDE) PORT_NAME("1'sC")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_B) PORT_NAME("b")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_A) PORT_NAME("A")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_C) PORT_NAME("C")

	PORT_START("IN.4") // O5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_X) PORT_NAME("XOR")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")

	PORT_START("IN.5") // O6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_NAME(")")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("STO")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_EQUALS) PORT_NAME("SUM")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("RCL")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_OPENBRACE) PORT_NAME("(")

	PORT_START("IN.6") // O7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("CE")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")

	// note: even though power buttons are on the matrix, they are not CPU-controlled
	PORT_START("IN.7") // Vss!
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_DEL) PORT_CODE(KEYCODE_PGUP) PORT_NAME("C/ON") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_G) PORT_NAME("DEC")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_J) PORT_NAME("OCT")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_H) PORT_NAME("HEX")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGDN) PORT_NAME("Off") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)false)
INPUT_PORTS_END

static INPUT_PORTS_START( tibusan )
	// PORT_NAME lists functions under [2nd] as secondaries.
	PORT_START("IN.0") // O0
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Y) PORT_NAME("y" UTF8_POW_X"  " UTF8_POW_X"" UTF8_SQUAREROOT"y") // 2nd one implies xth root of y
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("%  " UTF8_CAPITAL_DELTA"%")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_S) PORT_NAME("SEL")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_C) PORT_NAME("CST")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_M) PORT_NAME("MAR")

	PORT_START("IN.1") // O1
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ASTERISK) PORT_NAME(UTF8_MULTIPLY)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_HOME) PORT_NAME("STO  m")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_8) PORT_CODE(KEYCODE_8_PAD) PORT_NAME("8")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_7) PORT_CODE(KEYCODE_7_PAD) PORT_NAME("7")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_9) PORT_CODE(KEYCODE_9_PAD) PORT_NAME("9")

	PORT_START("IN.2") // O2
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS_PAD) PORT_NAME("-")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_END) PORT_NAME("RCL  b")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_5) PORT_CODE(KEYCODE_5_PAD) PORT_NAME("5")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_4) PORT_CODE(KEYCODE_4_PAD) PORT_NAME("4")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_6) PORT_CODE(KEYCODE_6_PAD) PORT_NAME("6")

	PORT_START("IN.3") // O4
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_SLASH_PAD) PORT_NAME(UTF8_DIVIDE)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_EQUALS) PORT_NAME(UTF8_CAPITAL_SIGMA"+  " UTF8_CAPITAL_SIGMA"-")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_OPENBRACE) PORT_NAME("(  AN-CI\"")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_COMMA) PORT_NAME("x<>y  L.R.")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_NAME(")  1/x")

	PORT_START("IN.4") // O5
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PLUS_PAD) PORT_NAME("+")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_X) PORT_NAME("SUM  x" UTF8_PRIME)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_2) PORT_CODE(KEYCODE_2_PAD) PORT_NAME("2")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_1) PORT_CODE(KEYCODE_1_PAD) PORT_NAME("1")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_3) PORT_CODE(KEYCODE_3_PAD) PORT_NAME("3")

	PORT_START("IN.5") // O6
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_F) PORT_NAME("FV")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_N) PORT_NAME("N")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_P) PORT_NAME("PMT")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_I) PORT_NAME("%i")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_V) PORT_NAME("PV")

	PORT_START("IN.6") // O7
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_NAME("=")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_E) PORT_NAME("EXC  x" UTF8_PRIME)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_STOP) PORT_CODE(KEYCODE_DEL_PAD) PORT_NAME(".")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_0) PORT_CODE(KEYCODE_0_PAD) PORT_NAME("0")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_MINUS) PORT_NAME("+/-")

	// note: even though power buttons are on the matrix, they are not CPU-controlled
	PORT_START("IN.7") // Vss!
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGUP) PORT_CODE(KEYCODE_DEL) PORT_NAME("On/C") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)true)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("2nd")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Q) PORT_NAME("x" UTF8_POW_2"  " UTF8_SQUAREROOT"x")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_L) PORT_NAME("ln(x)  e" UTF8_POW_X)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_PGDN) PORT_NAME("Off") PORT_CHANGED_MEMBER(DEVICE_SELF, hh_tms1k_state, power_button, (void *)false)
INPUT_PORTS_END

static MACHINE_CONFIG_START( ti30 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS0980, 400000) // guessed
	MCFG_TMS1XXX_READ_K_CB(READ8(ti30_state, read_k))
	MCFG_TMS1XXX_WRITE_O_CB(WRITE16(ti30_state, write_o))
	MCFG_TMS1XXX_WRITE_R_CB(WRITE16(ti30_state, write_r))
	MCFG_TMS1XXX_POWER_OFF_CB(WRITELINE(hh_tms1k_state, auto_power_off))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_tms1k_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_ti30)

	/* no sound! */
MACHINE_CONFIG_END





/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( cmulti8 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tmc1079nl", 0x0000, 0x0400, CRC(202c5ed8) SHA1(0143975cac20cb4a4e9f659ca0535e8a9056f5bb) )

	ROM_REGION( 867, "maincpu:mpla", 0 )
	ROM_LOAD( "tms1000_common2_micro.pla", 0, 867, CRC(d33da3cf) SHA1(13c4ebbca227818db75e6db0d45b66ba5e207776) )
	ROM_REGION( 365, "maincpu:opla", 0 )
	ROM_LOAD( "tms1000_cmulti8_output.pla", 0, 365, CRC(e999cece) SHA1(c5012877cd030a4dc66228f109fa23eec1867873) )
ROM_END


ROM_START( tisr16 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tms1001nl", 0x0000, 0x0400, CRC(b7ce3c1d) SHA1(95cdb0c6be31043f4fe06314ed41c0ca1337bc46) )

	ROM_REGION( 867, "maincpu:mpla", 0 )
	ROM_LOAD( "tms1000_sr16_micro.pla", 0, 867, CRC(5b35019c) SHA1(730d3b9041ed76d57fbedd73b009477fe432b386) )
	ROM_REGION( 365, "maincpu:opla", 0 )
	ROM_LOAD( "tms1000_sr16_output.pla", 0, 365, CRC(29b08739) SHA1(d55f01e40a2d493d45ea422f12e63b01bcde08fb) )
ROM_END


ROM_START( tisr16ii )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tms1016nl", 0x0000, 0x0400, CRC(c07a7b27) SHA1(34ea4d3b59871e08db74f8c5bfb7ff00d1f0adc7) )

	ROM_REGION( 867, "maincpu:mpla", 0 )
	ROM_LOAD( "tms1000_sr16ii_micro.pla", 0, 867, CRC(31b43e95) SHA1(6864e4c20f3affffcd3810dcefbc9484dd781547) )
	ROM_REGION( 365, "maincpu:opla", 0 )
	ROM_LOAD( "tms1000_sr16ii_output.pla", 0, 365, CRC(c45dfbd0) SHA1(5d588c1abc317134b51eb08ac3953f1009d80056) )
ROM_END


ROM_START( ti1250 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tmc0952nl", 0x0000, 0x0400, CRC(fc0cee65) SHA1(1480e4553181f081281d3b78457721b9ecb20173) )

	ROM_REGION( 867, "maincpu:mpla", 0 )
	ROM_LOAD( "tms1000_ti1250_micro.pla", 0, 867, CRC(cb3fd2d6) SHA1(82cf36a65dfc3ccb9dd08e48f45ac4d90f693238) )
	ROM_REGION( 195, "maincpu:opla", 0 )
	ROM_LOAD( "tms0950_ti1250_output.pla", 0, 195, CRC(31570eb8) SHA1(c1cb17c31367b65aa777925459515c3d5c565508) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common2_segment.pla", 0, 157, CRC(c03cccd8) SHA1(08bc4b597686a7aa8b2c9f43b85b62747ffd455b) )
ROM_END

ROM_START( ti125076 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tms0972nl_za0348", 0x0000, 0x0400, CRC(6e3f8add) SHA1(a249209e2a92f5016e33b7ad2c6c2660df1df959) )

	ROM_REGION( 782, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0970_common1_instr.pla", 0, 782, CRC(05306ef8) SHA1(60a0a3c49ce330bce0c27f15f81d61461d0432ce) )
	ROM_REGION( 860, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0970_common1_micro.pla", 0, 860, CRC(6ff5d51d) SHA1(59d3e5de290ba57694068ddba78d21a0c1edf427) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_ti1270_output.pla", 0, 352, CRC(472f95a0) SHA1(32adb17285f2f3f93a4b027a3dd2156ec48000ec) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common2_segment.pla", 0, 157, CRC(c03cccd8) SHA1(08bc4b597686a7aa8b2c9f43b85b62747ffd455b) )
ROM_END


ROM_START( ti1270 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tmc0974nl_za0355", 0x0000, 0x0400, CRC(48e09b4b) SHA1(17f27167164df223f9f06082ece4c3fc3900eda3) )

	ROM_REGION( 782, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0970_common1_instr.pla", 0, 782, CRC(05306ef8) SHA1(60a0a3c49ce330bce0c27f15f81d61461d0432ce) )
	ROM_REGION( 860, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0970_common1_micro.pla", 0, 860, CRC(6ff5d51d) SHA1(59d3e5de290ba57694068ddba78d21a0c1edf427) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_ti1270_output.pla", 0, 352, CRC(472f95a0) SHA1(32adb17285f2f3f93a4b027a3dd2156ec48000ec) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common2_segment.pla", 0, 157, CRC(c03cccd8) SHA1(08bc4b597686a7aa8b2c9f43b85b62747ffd455b) )
ROM_END


ROM_START( ti25503 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tms1043nl_za0352", 0x0000, 0x0400, CRC(434c2684) SHA1(ff566f1991f63cfe057879674e6bc7ccd580a919) )

	ROM_REGION( 867, "maincpu:mpla", 0 )
	ROM_LOAD( "tms1000_ti25503_micro.pla", 0, 867, CRC(65d274ae) SHA1(33d77efe38f8b067096c643d71263bb5adde0ca9) )
	ROM_REGION( 365, "maincpu:opla", 0 )
	ROM_LOAD( "tms1000_ti25503_output.pla", 0, 365, CRC(ac43b768) SHA1(5eb19b493328c73edab73e44591afda0fbe4965f) )
ROM_END


ROM_START( ti1000 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tmc1991nl", 0x0000, 0x0400, CRC(2da5381d) SHA1(b5dc14553db2068ed48e130e5ec9109930d8cda9) )

	ROM_REGION( 782, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0970_common1_instr.pla", 0, 782, CRC(05306ef8) SHA1(60a0a3c49ce330bce0c27f15f81d61461d0432ce) )
	ROM_REGION( 860, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0970_common2_micro.pla", 0, 860, CRC(7f50ab2e) SHA1(bff3be9af0e322986f6e545b567c97d70e135c93) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_ti1000_output.pla", 0, 352, CRC(a936631e) SHA1(1f900b12a41419d6e1ffbddd5cf72be3adaa4435) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common3_segment.pla", 0, 157, CRC(b5b3153f) SHA1(e0145c729695a4f962970aee0571d42cee6f5a9e) )
ROM_END


ROM_START( wizatron )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tmc0907nl_za0379", 0x0000, 0x0400, CRC(5a6af094) SHA1(b1f27e1f13f4db3b052dd50fb08dbf9c4d8db26e) )

	ROM_REGION( 782, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0970_common1_instr.pla", 0, 782, CRC(05306ef8) SHA1(60a0a3c49ce330bce0c27f15f81d61461d0432ce) )
	ROM_REGION( 860, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0970_common2_micro.pla", 0, 860, CRC(7f50ab2e) SHA1(bff3be9af0e322986f6e545b567c97d70e135c93) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_wizatron_output.pla", 0, 352, CRC(c0ee5c04) SHA1(e9fadcef688309adbe6c1c0545aac6883ce4a1ac) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common2_segment.pla", 0, 157, CRC(c03cccd8) SHA1(08bc4b597686a7aa8b2c9f43b85b62747ffd455b) )
ROM_END


ROM_START( lilprof )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tms0975nl_za0356", 0x0000, 0x0400, CRC(fef9dd39) SHA1(5c9614c9c5092d55dabeee2d6e0387d50d6ad4d5) )

	ROM_REGION( 782, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0970_common1_instr.pla", 0, 782, CRC(05306ef8) SHA1(60a0a3c49ce330bce0c27f15f81d61461d0432ce) )
	ROM_REGION( 860, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0970_common1_micro.pla", 0, 860, CRC(6ff5d51d) SHA1(59d3e5de290ba57694068ddba78d21a0c1edf427) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_lilprof_output.pla", 0, 352, CRC(73f9ca93) SHA1(9d6c559e2c1886c62bcd57e3c3aa897e8829b4d2) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common2_segment.pla", 0, 157, CRC(c03cccd8) SHA1(08bc4b597686a7aa8b2c9f43b85b62747ffd455b) )
ROM_END

ROM_START( lilprof78 )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "tmc1993nl", 0x0000, 0x0400, CRC(e941316b) SHA1(7e1542045d1e731cea81a639c9ac9e91bb233b15) )

	ROM_REGION( 782, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0970_common1_instr.pla", 0, 782, CRC(05306ef8) SHA1(60a0a3c49ce330bce0c27f15f81d61461d0432ce) )
	ROM_REGION( 860, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0970_common2_micro.pla", 0, 860, CRC(7f50ab2e) SHA1(bff3be9af0e322986f6e545b567c97d70e135c93) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_lilprof78_output.pla", 0, 352, CRC(b7416cc0) SHA1(57891ffeaf34aafe8a915086c6d2feb78f5113af) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common3_segment.pla", 0, 157, CRC(b5b3153f) SHA1(e0145c729695a4f962970aee0571d42cee6f5a9e) )
ROM_END


ROM_START( dataman )
	ROM_REGION( 0x1000, "maincpu", 0 )
	ROM_LOAD16_WORD( "tmc1982nl", 0x0000, 0x1000, CRC(3521f53f) SHA1(c46fe7fe20715fdf5aed65833fb867cfd3938062) ) // matches patent US4340374

	ROM_REGION( 1246, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0980_common1_instr.pla", 0, 1246, CRC(42db9a38) SHA1(2d127d98028ec8ec6ea10c179c25e447b14ba4d0) )
	ROM_REGION( 2127, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0270_common2_micro.pla", 0, 2127, CRC(86737ac1) SHA1(4aa0444f3ddf88738ea74aec404c684bf54eddba) )
	ROM_REGION( 525, "maincpu:opla", 0 )
	ROM_LOAD( "tms1980_dataman_output.pla", 0, 525, CRC(5fc6f451) SHA1(11475c785c34eab5b13c5dc67f413c709cd4bd4d) )
ROM_END


ROM_START( mathmarv )
	ROM_REGION( 0x1000, "maincpu", 0 )
	ROM_LOAD16_WORD( "tmc1986anl", 0x0000, 0x1000, CRC(79fda72d) SHA1(137852b29d9136459f78e29e7810195a956a5903) )

	ROM_REGION( 1246, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0980_common1_instr.pla", 0, 1246, CRC(42db9a38) SHA1(2d127d98028ec8ec6ea10c179c25e447b14ba4d0) )
	ROM_REGION( 2127, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0270_common2_micro.pla", 0, 2127, CRC(86737ac1) SHA1(4aa0444f3ddf88738ea74aec404c684bf54eddba) )
	ROM_REGION( 525, "maincpu:opla", 0 )
	ROM_LOAD( "tms1980_mathmarv_output.pla", 0, 525, CRC(5fc6f451) SHA1(11475c785c34eab5b13c5dc67f413c709cd4bd4d) )
ROM_END


ROM_START( ti30 )
	ROM_REGION( 0x1000, "maincpu", 0 )
	ROM_LOAD16_WORD( "tmc0981nl", 0x0000, 0x1000, CRC(41298a14) SHA1(06f654c70add4044a612d3a38b0c2831c188fd0c) )

	ROM_REGION( 1246, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0980_common1_instr.pla", 0, 1246, CRC(42db9a38) SHA1(2d127d98028ec8ec6ea10c179c25e447b14ba4d0) )
	ROM_REGION( 1982, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0980_common1_micro.pla", 0, 1982, CRC(3709014f) SHA1(d28ee59ded7f3b9dc3f0594a32a98391b6e9c961) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_ti30_output.pla", 0, 352, CRC(00475f99) SHA1(70e04c1472639bd35d4adaab0b9f1ae4a0e394be) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common1_segment.pla", 0, 157, CRC(399aa481) SHA1(72c56c58fde3fbb657d69647a9543b5f8fc74279) )
ROM_END


ROM_START( tibusan )
	ROM_REGION( 0x1000, "maincpu", 0 )
	ROM_LOAD16_WORD( "tmc0982nl", 0x0000, 0x1000, CRC(6954560a) SHA1(6c153a0c9239a811e3514a43d809964c06f8f88e) )

	ROM_REGION( 1246, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0980_common1_instr.pla", 0, 1246, CRC(42db9a38) SHA1(2d127d98028ec8ec6ea10c179c25e447b14ba4d0) )
	ROM_REGION( 1982, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0980_common1_micro.pla", 0, 1982, CRC(3709014f) SHA1(d28ee59ded7f3b9dc3f0594a32a98391b6e9c961) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_ti30_output.pla", 0, 352, CRC(00475f99) SHA1(70e04c1472639bd35d4adaab0b9f1ae4a0e394be) )
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common1_segment.pla", 0, 157, CRC(399aa481) SHA1(72c56c58fde3fbb657d69647a9543b5f8fc74279) )
ROM_END


ROM_START( tiprog )
	ROM_REGION( 0x1000, "maincpu", 0 )
	ROM_LOAD16_WORD( "za0675nl", 0x0000, 0x1000, CRC(82355854) SHA1(03fab373bce04df8ea3fe25352525e8539213626) ) // tmc0983

	ROM_REGION( 1246, "maincpu:ipla", 0 )
	ROM_LOAD( "tms0980_common1_instr.pla", 0, 1246, CRC(42db9a38) SHA1(2d127d98028ec8ec6ea10c179c25e447b14ba4d0) )
	ROM_REGION( 1982, "maincpu:mpla", 0 )
	ROM_LOAD( "tms0980_tiprog_micro.pla", 0, 1982, CRC(57043284) SHA1(0fa06d5865830ecdb3d870271cb92ac917bed3ca) )
	ROM_REGION( 352, "maincpu:opla", 0 )
	ROM_LOAD( "tms0980_tiprog_output.pla", 0, 352, BAD_DUMP CRC(125c4ee6) SHA1(b8d865c42fd37c3d9b92c5edbecfc831be558597) ) // corrected by hand
	ROM_REGION( 157, "maincpu:spla", 0 )
	ROM_LOAD( "tms0980_common1_segment.pla", 0, 157, CRC(399aa481) SHA1(72c56c58fde3fbb657d69647a9543b5f8fc74279) )
ROM_END



//    YEAR  NAME       PARENT  CMP MACHINE    INPUT      STATE         INIT  COMPANY, FULLNAME, FLAGS
COMP( 1977, cmulti8,   0,       0, cmulti8,   cmulti8,   cmulti8_state,   0, "Canon", "Multi 8 (Canon)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )

COMP( 1974, tisr16,    0,       0, tisr16,    tisr16,    tisr16_state,    0, "Texas Instruments", "SR-16", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1975, tisr16ii,  0,       0, tisr16,    tisr16ii,  tisr16_state,    0, "Texas Instruments", "SR-16 II", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )

COMP( 1975, ti1250,    0,       0, ti1250,    ti1250,    ti1250_state,    0, "Texas Instruments", "TI-1250 (1975 version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1976, ti125076,  ti1250,  0, ti1270,    ti1250,    ti1250_state,    0, "Texas Instruments", "TI-1250 (1976 version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1976, ti1270,    0,       0, ti1270,    ti1270,    ti1250_state,    0, "Texas Instruments", "TI-1270", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )

COMP( 1976, ti25503,   0,       0, ti25503,   ti25503,   ti25503_state,   0, "Texas Instruments", "TI-2550 III", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )

COMP( 1977, ti1000,    0,       0, ti1000,    ti1000,    ti1000_state,    0, "Texas Instruments", "TI-1000 (1977 version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )

COMP( 1977, wizatron,  0,       0, wizatron,  wizatron,  wizatron_state,  0, "Texas Instruments", "Wiz-A-Tron", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1976, lilprof,   0,       0, lilprof,   lilprof,   lilprof_state,   0, "Texas Instruments", "Little Professor (1976 version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1978, lilprof78, lilprof, 0, lilprof78, lilprof78, lilprof78_state, 0, "Texas Instruments", "Little Professor (1978 version)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )

COMP( 1977, dataman,   0,       0, dataman,   dataman,   dataman_state,   0, "Texas Instruments", "DataMan", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1980, mathmarv,  0,       0, mathmarv,  mathmarv,  mathmarv_state,  0, "Texas Instruments", "Math Marvel", MACHINE_SUPPORTS_SAVE )

COMP( 1976, ti30,      0,       0, ti30,      ti30,      ti30_state,      0, "Texas Instruments", "TI-30", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1976, tibusan,   0,       0, ti30,      tibusan,   ti30_state,      0, "Texas Instruments", "TI Business Analyst", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1977, tiprog,    0,       0, ti30,      tiprog,    ti30_state,      0, "Texas Instruments", "TI Programmer", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
