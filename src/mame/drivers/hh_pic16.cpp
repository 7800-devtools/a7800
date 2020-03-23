// license:BSD-3-Clause
// copyright-holders:hap, Sean Riddle, Kevin Horton
/***************************************************************************

  GI PIC 16xx-driven dedicated handhelds or other simple devices.

  known chips:

  serial  device  etc.
-----------------------------------------------------------
 *020     1650    19??, GI Economega IV TV PPL Tuning System Control
 @024     1655    1979, Toytronic? Football
 @033     1655A   1979, Toytronic Football (newer)
 @036     1655A   1979, Ideal Maniac
 @043     1655A   1979, Caprice Pro-Action Baseball
 @051     1655A   1979, Tandy Electronic Basketball
 @053     1655A   1979, Atari Touch Me
 @0??     1655A   1979, Tiger Half Court Computer Basketball/Sears Electronic Basketball (custom label)
 @061     1655A   1980, Lakeside Le Boom
 *081     1655A   19??, Ramtex Space Invaders/Block Buster
 @094     1655A   1980, GAF Melody Madness
 @110     1650A   1979, Tiger/Tandy Rocket Pinball
 @133     1650A   1981, U.S. Games Programmable Baseball/Tandy 2-Player Baseball
 @144     1650A   1981, U.S. Games/Tandy 2-Player Football
 *192     1650    19??, <unknown> phone dialer (have dump)
 *255     1655    19??, <unknown> talking clock (have dump)
 *518     1650A   19??, GI Teleview Control Chip (features differ per program)
 *519     1650A   19??, "
 *532     1650A   19??, "
 *533     1650A   19??, "
 *536     1650    1982, GI Teleview Autodialer/Terminal Identifier

  (* denotes not yet emulated by MAME, @ denotes it's in this driver)


  TODO:
  - tweak MCU frequency for games when video/audio recording surfaces(YouTube etc.)
  - some of the games rely on the fact that faster/longer strobed leds appear brighter,
    eg. hccbaskb(player led), ..
  - leboom discrete sound for volume decay (simulated for now)
  - ttfball: discrete sound part, for volume gating?
  - what's the relation between hccbaskb and tbaskb? Is one the bootleg
    of the other? Or are they both made by the same subcontractor?

***************************************************************************/

#include "emu.h"
#include "cpu/pic16c5x/pic16c5x.h"
#include "machine/clock.h"
#include "sound/spkrdev.h"
#include "speaker.h"

#include "hccbaskb.lh"
#include "leboom.lh" // clickable
#include "maniac.lh" // clickable
#include "melodym.lh" // clickable
#include "rockpin.lh"
#include "tbaskb.lh"
#include "touchme.lh" // clickable
#include "ttfball.lh"
#include "us2pfball.lh"

#include "hh_pic16_test.lh" // common test-layout - use external artwork


class hh_pic16_state : public driver_device
{
public:
	hh_pic16_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_inp_matrix(*this, "IN.%u", 0),
		m_speaker(*this, "speaker"),
		m_display_wait(33),
		m_display_maxy(1),
		m_display_maxx(0)
	{ }

	// devices
	required_device<cpu_device> m_maincpu;
	optional_ioport_array<6> m_inp_matrix; // max 6
	optional_device<speaker_sound_device> m_speaker;

	// misc common
	u8 m_a;                         // MCU port A write data
	u8 m_b;                         // " B
	u8 m_c;                         // " C
	u8 m_d;                         // " D
	u16 m_inp_mux;                  // multiplexed inputs mask

	u16 read_inputs(int columns);

	// display common
	int m_display_wait;             // led/lamp off-delay in milliseconds (default 33ms)
	int m_display_maxy;             // display matrix number of rows
	int m_display_maxx;             // display matrix number of columns (max 31 for now)

	u32 m_display_state[0x20];      // display matrix rows data (last bit is used for always-on)
	u16 m_display_segmask[0x20];    // if not 0, display matrix row is a digit, mask indicates connected segments
	u32 m_display_cache[0x20];      // (internal use)
	u8 m_display_decay[0x20][0x20]; // (internal use)

	TIMER_DEVICE_CALLBACK_MEMBER(display_decay_tick);
	void display_update();
	void set_display_size(int maxx, int maxy);
	void set_display_segmask(u32 digits, u32 mask);
	void display_matrix(int maxx, int maxy, u32 setx, u32 sety, bool update = true);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
};


// machine start/reset

void hh_pic16_state::machine_start()
{
	// zerofill
	memset(m_display_state, 0, sizeof(m_display_state));
	memset(m_display_cache, ~0, sizeof(m_display_cache));
	memset(m_display_decay, 0, sizeof(m_display_decay));
	memset(m_display_segmask, 0, sizeof(m_display_segmask));

	m_a = 0;
	m_b = 0;
	m_c = 0;
	m_d = 0;
	m_inp_mux = ~0;

	// register for savestates
	save_item(NAME(m_display_maxy));
	save_item(NAME(m_display_maxx));
	save_item(NAME(m_display_wait));

	save_item(NAME(m_display_state));
	/* save_item(NAME(m_display_cache)); */ // don't save!
	save_item(NAME(m_display_decay));
	save_item(NAME(m_display_segmask));

	save_item(NAME(m_a));
	save_item(NAME(m_b));
	save_item(NAME(m_c));
	save_item(NAME(m_d));
	save_item(NAME(m_inp_mux));
}

void hh_pic16_state::machine_reset()
{
}



/***************************************************************************

  Helper Functions

***************************************************************************/

// The device may strobe the outputs very fast, it is unnoticeable to the user.
// To prevent flickering here, we need to simulate a decay.

void hh_pic16_state::display_update()
{
	u32 active_state[0x20];

	for (int y = 0; y < m_display_maxy; y++)
	{
		active_state[y] = 0;

		for (int x = 0; x <= m_display_maxx; x++)
		{
			// turn on powered segments
			if (m_display_state[y] >> x & 1)
				m_display_decay[y][x] = m_display_wait;

			// determine active state
			u32 ds = (m_display_decay[y][x] != 0) ? 1 : 0;
			active_state[y] |= (ds << x);
		}
	}

	// on difference, send to output
	for (int y = 0; y < m_display_maxy; y++)
		if (m_display_cache[y] != active_state[y])
		{
			if (m_display_segmask[y] != 0)
				output().set_digit_value(y, active_state[y] & m_display_segmask[y]);

			const int mul = (m_display_maxx <= 10) ? 10 : 100;
			for (int x = 0; x <= m_display_maxx; x++)
			{
				int state = active_state[y] >> x & 1;
				char buf1[0x10]; // lampyx
				char buf2[0x10]; // y.x

				if (x == m_display_maxx)
				{
					// always-on if selected
					sprintf(buf1, "lamp%da", y);
					sprintf(buf2, "%d.a", y);
				}
				else
				{
					sprintf(buf1, "lamp%d", y * mul + x);
					sprintf(buf2, "%d.%d", y, x);
				}
				output().set_value(buf1, state);
				output().set_value(buf2, state);
			}
		}

	memcpy(m_display_cache, active_state, sizeof(m_display_cache));
}

TIMER_DEVICE_CALLBACK_MEMBER(hh_pic16_state::display_decay_tick)
{
	// slowly turn off unpowered segments
	for (int y = 0; y < m_display_maxy; y++)
		for (int x = 0; x <= m_display_maxx; x++)
			if (m_display_decay[y][x] != 0)
				m_display_decay[y][x]--;

	display_update();
}

void hh_pic16_state::set_display_size(int maxx, int maxy)
{
	m_display_maxx = maxx;
	m_display_maxy = maxy;
}

void hh_pic16_state::set_display_segmask(u32 digits, u32 mask)
{
	// set a segment mask per selected digit, but leave unselected ones alone
	for (int i = 0; i < 0x20; i++)
	{
		if (digits & 1)
			m_display_segmask[i] = mask;
		digits >>= 1;
	}
}

void hh_pic16_state::display_matrix(int maxx, int maxy, u32 setx, u32 sety, bool update)
{
	set_display_size(maxx, maxy);

	// update current state
	u32 mask = (1 << maxx) - 1;
	for (int y = 0; y < maxy; y++)
		m_display_state[y] = (sety >> y & 1) ? ((setx & mask) | (1 << maxx)) : 0;

	if (update)
		display_update();
}


// generic input handlers

u16 hh_pic16_state::read_inputs(int columns)
{
	// active low
	u16 ret = ~0;

	// read selected input rows
	for (int i = 0; i < columns; i++)
		if (~m_inp_mux >> i & 1)
			ret &= m_inp_matrix[i]->read();

	return ret;
}



/***************************************************************************

  Minidrivers (subclass, I/O, Inputs, Machine Config)

***************************************************************************/

/***************************************************************************

  Atari Touch Me
  * PIC 1655A-053
  * 2 7seg LEDs + 4 other LEDs, 1-bit sound

  This is the handheld version of the 1974 arcade game.

  known revisions:
  - Model BH-100 GI C013233 Rev 2 Atari W 1979: PIC 1655A-053
  - Model BH-100 C013150 Rev 6 Atari 1979: AMI C10745 (custom ASIC)

***************************************************************************/

class touchme_state : public hh_pic16_state
{
public:
	touchme_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	void update_speaker();
	DECLARE_READ8_MEMBER(read_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
};

// handlers

void touchme_state::prepare_display()
{
	set_display_segmask(3, 0x7f);
	display_matrix(7, 7, m_c, ~m_b & 0x7b);
}

void touchme_state::update_speaker()
{
	m_speaker->level_w((m_b >> 7 & 1) | (m_c >> 6 & 2));
}

READ8_MEMBER(touchme_state::read_a)
{
	// A: multiplexed inputs
	return read_inputs(3) & 0xf;
}

WRITE8_MEMBER(touchme_state::write_b)
{
	// B0-B2: input mux
	m_inp_mux = data & 7;

	// B0,B1: digit select
	// B3-B6: leds
	m_b = data;
	prepare_display();

	// B7: speaker lead 1
	update_speaker();
}

WRITE8_MEMBER(touchme_state::write_c)
{
	// C0-C6: digit segments
	m_c = data;
	prepare_display();

	// C7: speaker lead 2
	update_speaker();
}


// config

static INPUT_PORTS_START( touchme )
	PORT_START("IN.0") // B0 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_NAME("Last")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_NAME("High")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON7 ) PORT_NAME("Skill")

	PORT_START("IN.1") // B1 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Blue Button")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("Yellow Button")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("Red Button")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("Green Button")

	PORT_START("IN.2") // B2 port A
	PORT_CONFNAME( 0x07, 0x01^0x07, "Game Select")
	PORT_CONFSETTING(    0x01^0x07, "1" )
	PORT_CONFSETTING(    0x02^0x07, "2" )
	PORT_CONFSETTING(    0x04^0x07, "3" )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END

static const s16 touchme_speaker_levels[] = { 0, 0x7fff, -0x8000, 0 };

static MACHINE_CONFIG_START( touchme )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 300000) // approximation - RC osc. R=100K, C=47pF
	MCFG_PIC16C5x_READ_A_CB(READ8(touchme_state, read_a))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(touchme_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(touchme_state, write_c))

	MCFG_DEVICE_ADD("clock", CLOCK, 300000/4) // PIC CLKOUT, tied to RTCC
	MCFG_CLOCK_SIGNAL_HANDLER(INPUTLINE("maincpu", PIC16C5x_RTCC))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_touchme)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SPEAKER_LEVELS(4, touchme_speaker_levels)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Caprice Pro-Action Baseball (manufactured by Calfax)
  * PIC 1655A-043
  * 1 7seg LED + 36 other LEDs, CD4028, 1-bit sound

***************************************************************************/

class pabball_state : public hh_pic16_state
{
public:
	pabball_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);

	DECLARE_INPUT_CHANGED_MEMBER(reset_button);
};

// handlers

void pabball_state::prepare_display()
{
	// CD4028 BCD to decimal decoder
	u16 sel = m_c & 0xf;
	if (sel & 8) sel &= 9;
	sel = 1 << sel;

	// CD4028 9 is 7seg
	set_display_segmask(0x200, 0xff);
	display_matrix(8, 10, m_b, sel);
}

WRITE8_MEMBER(pabball_state::write_b)
{
	// B: led data
	m_b = ~data;
	prepare_display();
}

WRITE8_MEMBER(pabball_state::write_c)
{
	// C2: RTCC pin
	m_maincpu->set_input_line(PIC16C5x_RTCC, data >> 2 & 1);

	// C7: speaker out
	m_speaker->level_w(data >> 7 & 1);

	// C0-C3: CD4028 A-D
	m_c = data;
	prepare_display();
}


// config

static INPUT_PORTS_START( pabball )
	PORT_START("IN.0") // port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL PORT_NAME("P2 Curve Left")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_COCKTAIL PORT_NAME("P2 Curve Right")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_COCKTAIL PORT_NAME("P2 Straight")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN.1") // port C
	PORT_BIT( 0xcf, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("P1 Hit")
	PORT_CONFNAME( 0x20, 0x00, DEF_STR( Players ) )
	PORT_CONFSETTING(    0x00, "1" )
	PORT_CONFSETTING(    0x20, "2" )

	PORT_START("RESET")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_NAME("P1 Reset") PORT_CHANGED_MEMBER(DEVICE_SELF, pabball_state, reset_button, nullptr)
INPUT_PORTS_END

INPUT_CHANGED_MEMBER(pabball_state::reset_button)
{
	// reset button is directly tied to MCLR pin
	m_maincpu->set_input_line(INPUT_LINE_RESET, newval ? ASSERT_LINE : CLEAR_LINE);
}

static MACHINE_CONFIG_START( pabball )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 1200000) // approximation - RC osc. R=18K, C=27pF
	MCFG_PIC16C5x_READ_A_CB(IOPORT("IN.0"))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(pabball_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(IOPORT("IN.1"))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(pabball_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_hh_pic16_test)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  GAF Melody Madness
  * PIC 1655A-094
  * 2 lamps under tube, 1-bit sound

  Melody Madness is a tabletop music memory game, shaped like a jukebox.
  It can also be played as a simple electronic piano.

***************************************************************************/

class melodym_state : public hh_pic16_state
{
public:
	melodym_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_READ8_MEMBER(read_c);
	DECLARE_WRITE8_MEMBER(write_c);
};

// handlers

WRITE8_MEMBER(melodym_state::write_b)
{
	// B2-B6: input mux
	m_inp_mux = data >> 2 & 0x1f;
}

READ8_MEMBER(melodym_state::read_c)
{
	// C0-C4: multiplexed inputs
	return read_inputs(5) | 0xe0;
}

WRITE8_MEMBER(melodym_state::write_c)
{
	// C6: both lamps
	m_display_wait = 2;
	display_matrix(1, 1, ~data >> 6 & 1, 1);

	// C7: speaker out
	m_speaker->level_w(~data >> 7 & 1);
}


// config

static INPUT_PORTS_START( melodym )
	PORT_START("IN.0") // B2 port C
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_NAME("Button 1")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_NAME("Button 2")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_NAME("Button 3")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_NAME("Button 4")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_NAME("Button 5")

	PORT_START("IN.1") // B3 port C
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_Q) PORT_NAME("Button 6")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_W) PORT_NAME("Button 7")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_E) PORT_NAME("Button 8")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_R) PORT_NAME("Button 9")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_T) PORT_NAME("Button 10")

	PORT_START("IN.2") // B4 port C
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_A) PORT_NAME("Button 11")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_S) PORT_NAME("Button 12")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED ) // there is no button 13
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_F) PORT_NAME("Button 14")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_G) PORT_NAME("Button 15")

	PORT_START("IN.3") // B5 port C
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_Z) PORT_NAME("Button 16")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_X) PORT_NAME("Button 17")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_C) PORT_NAME("Button 18")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_V) PORT_NAME("Button 19")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_B) PORT_NAME("Button 20")

	PORT_START("IN.4") // B6 port C
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_N) PORT_NAME("Button 21")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_M) PORT_NAME("Button 22")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_COMMA) PORT_NAME("Button 23")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_STOP) PORT_NAME("Button 24")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_SLASH) PORT_NAME("Button 25")

	PORT_START("IN.5") // port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_NAME("Novice")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_NAME("Whiz")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_9) PORT_NAME("Pro")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_0) PORT_NAME("Note")
INPUT_PORTS_END

static MACHINE_CONFIG_START( melodym )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 1000000) // approximation
	MCFG_PIC16C5x_READ_A_CB(IOPORT("IN.5"))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(melodym_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(READ8(melodym_state, read_c))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(melodym_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_melodym)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Ideal Maniac, by Ralph Baer
  * PIC 1655A-036
  * 2 7seg LEDs, 1-bit sound

  Maniac is a reflex game for 2-4 players. There are 4 challenges:
  1: Musical Maniac: Press the button as soon as the music stops.
  2: Sounds Abound: Count the number of tones in the song, then press the button
     after the same amount of beeps.
  3: Look Twice: Press the button after the game repeats the first pattern.
  4: Your Time Is Up: Press the button after estimating the duration of the tone.

***************************************************************************/

class maniac_state : public hh_pic16_state
{
public:
	maniac_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	void update_speaker();
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
};

// handlers

void maniac_state::prepare_display()
{
	m_display_state[0] = ~m_b & 0x7f;
	m_display_state[1] = ~m_c & 0x7f;

	set_display_segmask(3, 0x7f);
	set_display_size(7, 2);
	display_update();
}

void maniac_state::update_speaker()
{
	m_speaker->level_w((m_b >> 7 & 1) | (m_c >> 6 & 2));
}

WRITE8_MEMBER(maniac_state::write_b)
{
	// B0-B6: left 7seg
	m_b = data;
	prepare_display();

	// B7: speaker lead 1
	update_speaker();
}

WRITE8_MEMBER(maniac_state::write_c)
{
	// C0-C6: right 7seg
	m_c = data;
	prepare_display();

	// C7: speaker lead 2
	update_speaker();
}


// config

static INPUT_PORTS_START( maniac )
	PORT_START("IN.0") // port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1) // top button, increment clockwise
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(3)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(4)
INPUT_PORTS_END

static const s16 maniac_speaker_levels[] = { 0, 0x7fff, -0x8000, 0 };

static MACHINE_CONFIG_START( maniac )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 1000000) // approximation - RC osc. R=~13.4K, C=470pF
	MCFG_PIC16C5x_READ_A_CB(IOPORT("IN.0"))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(maniac_state, write_b))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(maniac_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_maniac)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SPEAKER_LEVELS(4, maniac_speaker_levels)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Lakeside Le Boom
  * PIC 1655A-061
  * 1 led, 1-bit sound with RC circuit for volume decay

  This is a tabletop timebomb defusion game. It's shaped like an aerial bomb,
  colored black on USA version, yellow on dual-language Canadian version.
  The game starts 'ticking' when the player opens the keypad door. To begin,
  select the game mode, rows(keypad size), and fuse duration.

  Game modes as described on the box:
  1: Eliminate the buttons one by one in the order set out by the computer. Press
     one twice and you'll be sorry!
  2: For 2 or more players. Take turns pressing the buttons, remember which ones.
     Press a button a second time and watch out, it's all over.
  3: The computer picks one secret button that stops the fuse. You must press it
     on your 5th turn. Listen to the clues and you'll do fine.
  4: The computer picks a secret combination. Find it first by listening to the
     clues. Find the right order and you'll get it to fizzle out.

***************************************************************************/

class leboom_state : public hh_pic16_state
{
public:
	leboom_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	DECLARE_READ8_MEMBER(read_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);

	void speaker_decay_reset();
	TIMER_DEVICE_CALLBACK_MEMBER(speaker_decay_sim);
	double m_speaker_volume;

protected:
	virtual void machine_start() override;
};

// handlers

void leboom_state::speaker_decay_reset()
{
	if (~m_c & 0x80)
		m_speaker_volume = 1.0;

	m_speaker->set_output_gain(0, m_speaker_volume);
}

TIMER_DEVICE_CALLBACK_MEMBER(leboom_state::speaker_decay_sim)
{
	// volume decays when speaker is off (divisor and timer period determine duration)
	speaker_decay_reset();
	m_speaker_volume /= 1.015;
}

READ8_MEMBER(leboom_state::read_a)
{
	// A: multiplexed inputs
	return read_inputs(6) & 0xf;
}

WRITE8_MEMBER(leboom_state::write_b)
{
	// B0-B5: input mux
	m_inp_mux = data & 0x3f;
}

WRITE8_MEMBER(leboom_state::write_c)
{
	// C4: single led
	display_matrix(1, 1, data >> 4 & 1, 1);

	// C7: speaker on
	m_c = data;
	speaker_decay_reset();

	// C6: speaker out
	m_speaker->level_w(data >> 6 & 1);
}


// config

static INPUT_PORTS_START( leboom )
	PORT_START("IN.0") // B0 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_NAME("Red Button 1")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_Q) PORT_NAME("Red Button 2")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_A) PORT_NAME("Red Button 3")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_Z) PORT_NAME("Red Button 4")

	PORT_START("IN.1") // B1 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_NAME("Red-Red Button")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_W) PORT_NAME("Red-Green Button")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_S) PORT_NAME("Red-Yellow Button")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_X) PORT_NAME("Red-Blue Button")

	PORT_START("IN.2") // B2 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_NAME("Shortest")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_E) PORT_NAME("Short")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_D) PORT_NAME("Long")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_C) PORT_NAME("Longest")

	PORT_START("IN.3") // B3 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_NAME("Yellow Button 1")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_R) PORT_NAME("Yellow Button 2")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_F) PORT_NAME("Yellow Button 3")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_V) PORT_NAME("Yellow Button 4")

	PORT_START("IN.4") // B4 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_NAME("Blue Button 1")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_T) PORT_NAME("Blue Button 2")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_G) PORT_NAME("Blue Button 3")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_B) PORT_NAME("Blue Button 4")

	PORT_START("IN.5") // B5 port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_NAME("Blue Button 5")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_Y) PORT_NAME("Blue Button 6")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_H) PORT_NAME("Blue Button 7")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_N) PORT_NAME("Blue Button 8")
INPUT_PORTS_END

void leboom_state::machine_start()
{
	hh_pic16_state::machine_start();

	// zerofill/init
	m_speaker_volume = 0;
	save_item(NAME(m_speaker_volume));
}

static MACHINE_CONFIG_START( leboom )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 1000000) // approximation
	MCFG_PIC16C5x_READ_A_CB(READ8(leboom_state, read_a))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(leboom_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(leboom_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_leboom)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_TIMER_DRIVER_ADD_PERIODIC("speaker_decay", leboom_state, speaker_decay_sim, attotime::from_msec(25))
MACHINE_CONFIG_END





/***************************************************************************

  Tandy Electronic Basketball (model 60-2146)
  * PIC 1655A-51
  * 2 7seg LEDs + 21 other LEDs, 1-bit sound

  The ROM is nearly identical to hccbaskb, the shell/overlay is the same as
  U.S. Games/Tandy Trick Shot Basketball.

***************************************************************************/

class tbaskb_state : public hh_pic16_state
{
public:
	tbaskb_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_READ8_MEMBER(read_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
};

// handlers

void tbaskb_state::prepare_display()
{
	// B4,B5 are 7segs
	set_display_segmask(0x30, 0x7f);
	display_matrix(7, 6, m_c, m_b);
}

READ8_MEMBER(tbaskb_state::read_a)
{
	// A2: skill switch, A3: multiplexed inputs
	return m_inp_matrix[5]->read() | read_inputs(5) | 3;
}

WRITE8_MEMBER(tbaskb_state::write_b)
{
	// B0: RTCC pin
	m_maincpu->set_input_line(PIC16C5x_RTCC, data & 1);

	// B0-B4: input mux
	m_inp_mux = ~data & 0x1f;

	// B0-B5: led select
	m_b = data;
	prepare_display();
}

WRITE8_MEMBER(tbaskb_state::write_c)
{
	// C7: speaker out
	m_speaker->level_w(data >> 7 & 1);

	// C0-C6: led data
	m_c = ~data;
	prepare_display();
}


// config

static INPUT_PORTS_START( tbaskb )
	PORT_START("IN.0") // B0 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_16WAY

	PORT_START("IN.1") // B1 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_16WAY

	PORT_START("IN.2") // B2 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_16WAY

	PORT_START("IN.3") // B3 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_16WAY

	PORT_START("IN.4") // B4 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON1 )

	PORT_START("IN.5") // port A2
	PORT_CONFNAME( 0x04, 0x04, DEF_STR( Difficulty ) )
	PORT_CONFSETTING(    0x04, "1" )
	PORT_CONFSETTING(    0x00, "2" )
INPUT_PORTS_END

static MACHINE_CONFIG_START( tbaskb )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 950000) // approximation - RC osc. R=18K, C=47pF
	MCFG_PIC16C5x_READ_A_CB(READ8(tbaskb_state, read_a))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(tbaskb_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(tbaskb_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_tbaskb)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Tiger Electronics Rocket Pinball (model 7-460)
  * PIC 1650A-110, 69-11397
  * 3 7seg LEDs + 44 other LEDs, 1-bit sound

  known releases:
  - Hong Kong: Rocket Pinball
  - USA(1): Rocket Pinball (model 60-2140), published by Tandy
  - USA(2): Cosmic Pinball (model 49-65456), published by Sears

***************************************************************************/

class rockpin_state : public hh_pic16_state
{
public:
	rockpin_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_WRITE8_MEMBER(write_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
	DECLARE_WRITE8_MEMBER(write_d);
};

// handlers

void rockpin_state::prepare_display()
{
	// 3 7seg leds from ports A and B
	set_display_segmask(7, 0x7f);
	display_matrix(7, 3, m_b, m_a, false);

	// 44 leds from ports C and D
	for (int y = 0; y < 6; y++)
		m_display_state[y+3] = (m_d >> y & 1) ? m_c : 0;

	set_display_size(8, 3+6);
	display_update();
}

WRITE8_MEMBER(rockpin_state::write_a)
{
	// A3,A4: speaker out
	m_speaker->level_w(data >> 3 & 3);

	// A0-A2: select digit
	m_a = ~data & 7;
	prepare_display();
}

WRITE8_MEMBER(rockpin_state::write_b)
{
	// B0-B6: digit segments
	m_b = data & 0x7f;
	prepare_display();
}

WRITE8_MEMBER(rockpin_state::write_c)
{
	// C0-C7: led data
	m_c = ~data;
	prepare_display();
}

WRITE8_MEMBER(rockpin_state::write_d)
{
	// D0-D5: led select
	m_d = ~data;
	prepare_display();
}


// config

static INPUT_PORTS_START( rockpin )
	PORT_START("IN.0") // port A
	PORT_BIT( 0x1f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("Right Flipper")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Left Flipper")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("Ball")
INPUT_PORTS_END

static const s16 rockpin_speaker_levels[] = { 0, 0x7fff, -0x8000, 0 };

static MACHINE_CONFIG_START( rockpin )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1650, 450000) // approximation - RC osc. R=47K, C=47pF
	MCFG_PIC16C5x_READ_A_CB(IOPORT("IN.0"))
	MCFG_PIC16C5x_WRITE_A_CB(WRITE8(rockpin_state, write_a))
	MCFG_PIC16C5x_READ_B_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(rockpin_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(rockpin_state, write_c))
	MCFG_PIC16C5x_READ_D_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_D_CB(WRITE8(rockpin_state, write_d))

	MCFG_DEVICE_ADD("clock", CLOCK, 450000/4) // PIC CLKOUT, tied to RTCC
	MCFG_CLOCK_SIGNAL_HANDLER(INPUTLINE("maincpu", PIC16C5x_RTCC))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_rockpin)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SPEAKER_LEVELS(4, rockpin_speaker_levels)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Tiger Electronics Half Court Computer Basketball (model 7-470)
  * PIC 1655A(no serial), 69-11557
  * 2 7seg LEDs + 26 other LEDs, 1-bit sound

  known releases:
  - Hong Kong: Half Court Computer Basketball
  - USA: Electronic Basketball (model 49-65453), published by Sears

***************************************************************************/

class hccbaskb_state : public hh_pic16_state
{
public:
	hccbaskb_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_READ8_MEMBER(read_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
};

// handlers

void hccbaskb_state::prepare_display()
{
	// B5,B6 are 7segs
	set_display_segmask(0x60, 0x7f);
	display_matrix(7, 7, m_c, m_b);
}

READ8_MEMBER(hccbaskb_state::read_a)
{
	// A2: skill switch, A3: multiplexed inputs
	return m_inp_matrix[5]->read() | read_inputs(5) | 3;
}

WRITE8_MEMBER(hccbaskb_state::write_b)
{
	// B0: RTCC pin
	m_maincpu->set_input_line(PIC16C5x_RTCC, data & 1);

	// B0-B4: input mux
	m_inp_mux = ~data & 0x1f;

	// B7: speaker out
	m_speaker->level_w(data >> 7 & 1);

	// B0-B6: led select
	m_b = data;
	prepare_display();
}

WRITE8_MEMBER(hccbaskb_state::write_c)
{
	// C0-C6: led data
	m_c = ~data;
	prepare_display();
}


// config

static INPUT_PORTS_START( hccbaskb )
	PORT_START("IN.0") // B0 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_16WAY

	PORT_START("IN.1") // B1 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_16WAY

	PORT_START("IN.2") // B2 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_16WAY

	PORT_START("IN.3") // B3 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_16WAY

	PORT_START("IN.4") // B4 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON1 )

	PORT_START("IN.5") // port A2
	PORT_CONFNAME( 0x04, 0x04, DEF_STR( Difficulty ) )
	PORT_CONFSETTING(    0x04, "1" )
	PORT_CONFSETTING(    0x00, "2" )
INPUT_PORTS_END

static MACHINE_CONFIG_START( hccbaskb )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 950000) // approximation - RC osc. R=15K, C=47pF
	MCFG_PIC16C5x_READ_A_CB(READ8(hccbaskb_state, read_a))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(hccbaskb_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(hccbaskb_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_hccbaskb)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Toytronic Football (set 1)
  * PIC 1655A-033
  * 4511 7seg BCD decoder, 7 7seg LEDs + 27 other LEDs, 1-bit sound

  (no brand) Football (set 2)
  * PIC 1655-024
  * rest same as above, 1 less button

  Hello and welcome to another Mattel Football clone, there are so many of these.
  The 1655-024 one came from an unbranded handheld, but comparison suggests that
  it's the 'prequel' of 1655A-033.

***************************************************************************/

class ttfball_state : public hh_pic16_state
{
public:
	ttfball_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_READ8_MEMBER(read_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
};

// handlers

void ttfball_state::prepare_display()
{
	// C0-C2: led data
	// C0-C3: 4511 A-D, C4: digit segment DP
	// C5: select digits or led matrix
	const u8 _4511_map[16] = { 0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7c,0x07,0x7f,0x67,0,0,0,0,0,0 };
	u16 led_data = (m_c & 0x20) ? (_4511_map[m_c & 0xf] | (~m_c << 3 & 0x80)) : (~m_c << 8 & 0x700);

	set_display_segmask(0x7f, 0xff);
	display_matrix(11, 9, led_data, m_b | (m_c << 1 & 0x100));
}

READ8_MEMBER(ttfball_state::read_a)
{
	// A3: multiplexed inputs, A0-A2: other inputs
	return m_inp_matrix[5]->read() | read_inputs(5);
}

WRITE8_MEMBER(ttfball_state::write_b)
{
	// B0: RTCC pin
	m_maincpu->set_input_line(PIC16C5x_RTCC, data & 1);

	// B0,B1,B3,B7: input mux low
	m_inp_mux = (m_inp_mux & 0x10) | (~data & 3) | (~data >> 1 & 4) | (~data >> 4 & 8);

	// B0-B7: led select (see above)
	m_b = data;
	prepare_display();
}

WRITE8_MEMBER(ttfball_state::write_c)
{
	// C6: speaker out
	m_speaker->level_w(data >> 6 & 1);

	// C7: input mux high
	m_inp_mux = (m_inp_mux & 0xf) | (data >> 3 & 0x10);

	// C0-C7: led data/select (see above)
	m_c = data;
	prepare_display();
}


// config

static INPUT_PORTS_START( ttfball )
	PORT_START("IN.0") // B0 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_16WAY

	PORT_START("IN.1") // B1 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_16WAY

	PORT_START("IN.2") // B3 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_16WAY

	PORT_START("IN.3") // B7 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_16WAY

	PORT_START("IN.4") // C7 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Kick")

	PORT_START("IN.5") // port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 ) PORT_NAME("Status")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_START2 ) PORT_NAME("Score")
	PORT_CONFNAME( 0x04, 0x04, DEF_STR( Difficulty ) )
	PORT_CONFSETTING(    0x04, "1" )
	PORT_CONFSETTING(    0x00, "2" )
INPUT_PORTS_END

static INPUT_PORTS_START( ttfballa )
	PORT_START("IN.0") // B0 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Kick")

	PORT_START("IN.1") // B1 port A3
	PORT_BIT( 0x08, 0x08, IPT_SPECIAL ) PORT_CONDITION("FAKE", 0x03, EQUALS, 0x00) // left/right

	PORT_START("IN.2") // B3 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_16WAY

	PORT_START("IN.3") // B7 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_16WAY

	PORT_START("IN.4") // C7 port A3
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN.5") // port A
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 ) PORT_NAME("Status")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_START2 ) PORT_NAME("Score")
	PORT_CONFNAME( 0x04, 0x04, DEF_STR( Difficulty ) )
	PORT_CONFSETTING(    0x04, "1" )
	PORT_CONFSETTING(    0x00, "2" )

	PORT_START("FAKE") // fake port for left/right combination
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_16WAY
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_16WAY
INPUT_PORTS_END

static MACHINE_CONFIG_START( ttfball )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1655, 1000000) // approximation - RC osc. R=27K(set 1) or 33K(set 2), C=68pF
	MCFG_PIC16C5x_READ_A_CB(READ8(ttfball_state, read_a))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(ttfball_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(ttfball_state, write_c))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_ttfball)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  U.S. Games Programmable Baseball
  * PIC 1650A-133
  * 3 7seg LEDs + 36 other LEDs, 1-bit sound

  known releases:
  - USA(1): Programmable Baseball
  - USA(2): Electronic 2-Player Baseball (model 60-2157), published by Tandy

***************************************************************************/

class uspbball_state : public hh_pic16_state
{
public:
	uspbball_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_WRITE8_MEMBER(write_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
	DECLARE_WRITE8_MEMBER(write_d);
};

// handlers

void uspbball_state::prepare_display()
{
	// D0-D2 are 7segs
	set_display_segmask(7, 0x7f);
	display_matrix(16, 6, m_c << 8 | m_b, m_d);
}

WRITE8_MEMBER(uspbball_state::write_a)
{
	// A0: speaker out
	m_speaker->level_w(data & 1);
}

WRITE8_MEMBER(uspbball_state::write_b)
{
	// B: digit segment data
	m_b = BITSWAP8(data,0,1,2,3,4,5,6,7);
	prepare_display();
}

WRITE8_MEMBER(uspbball_state::write_c)
{
	// C: led data
	m_c = ~data;
	prepare_display();
}

WRITE8_MEMBER(uspbball_state::write_d)
{
	// D0-D5: led/digit select
	m_d = ~data;
	prepare_display();
}


// config

static INPUT_PORTS_START( uspbball )
	PORT_START("IN.0") // port A
	PORT_BIT( 0x03, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL PORT_NAME("P2 Curve Right")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_COCKTAIL PORT_NAME("P2 Slow")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_COCKTAIL PORT_NAME("P2 Fast")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_COCKTAIL PORT_NAME("P2 Curve Left")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_COCKTAIL PORT_NAME("P2 Change Up/Fielder")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("P1 Batter")

	PORT_START("IN.1") // port D
	PORT_BIT( 0x7f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_CONFNAME( 0x80, 0x80, DEF_STR( Players ) )
	PORT_CONFSETTING(    0x80, "1" )
	PORT_CONFSETTING(    0x00, "2" )
INPUT_PORTS_END

static MACHINE_CONFIG_START( uspbball )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1650, 1000000) // approximation - RC osc. R=22K, C=47pF
	MCFG_PIC16C5x_READ_A_CB(IOPORT("IN.0"))
	MCFG_PIC16C5x_WRITE_A_CB(WRITE8(uspbball_state, write_a))
	MCFG_PIC16C5x_READ_B_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(uspbball_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(uspbball_state, write_c))
	MCFG_PIC16C5x_READ_D_CB(IOPORT("IN.1"))
	MCFG_PIC16C5x_WRITE_D_CB(WRITE8(uspbball_state, write_d))

	MCFG_DEVICE_ADD("clock", CLOCK, 1000000/4) // PIC CLKOUT, tied to RTCC
	MCFG_CLOCK_SIGNAL_HANDLER(INPUTLINE("maincpu", PIC16C5x_RTCC))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_hh_pic16_test)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  U.S. Games Electronic 2-Player Football
  * PIC 1650A-144
  * 8 7seg LEDs + 2 other LEDs, 1-bit sound

  known releases:
  - USA(1): Electronic 2-Player Football
  - USA(2): Electronic 2-Player Football (model 60-2156), published by Tandy

***************************************************************************/

class us2pfball_state : public hh_pic16_state
{
public:
	us2pfball_state(const machine_config &mconfig, device_type type, const char *tag)
		: hh_pic16_state(mconfig, type, tag)
	{ }

	void prepare_display();
	DECLARE_READ8_MEMBER(read_a);
	DECLARE_WRITE8_MEMBER(write_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
	DECLARE_WRITE8_MEMBER(write_d);
};

// handlers

void us2pfball_state::prepare_display()
{
	set_display_segmask(0xff, 0x7f);
	display_matrix(7, 10, m_c, m_d | (m_a << 6 & 0x300));
}

READ8_MEMBER(us2pfball_state::read_a)
{
	// A0,A1: multiplexed inputs, A4-A7: other inputs
	return (read_inputs(4) & 3) | (m_inp_matrix[4]->read() & 0xf0) | 0x0c;
}

WRITE8_MEMBER(us2pfball_state::write_a)
{
	// A2,A3: leds
	m_a = data;
	prepare_display();
}

WRITE8_MEMBER(us2pfball_state::write_b)
{
	// B0-B3: input mux
	m_inp_mux = data & 0xf;
}

WRITE8_MEMBER(us2pfball_state::write_c)
{
	// C7: speaker out
	m_speaker->level_w(data >> 7 & 1);

	// C0-C6: digit segments
	m_c = data;
	prepare_display();
}

WRITE8_MEMBER(us2pfball_state::write_d)
{
	// D0-D7: digit select
	m_d = ~data;
	prepare_display();
}


// config

static INPUT_PORTS_START( us2pfball )
	PORT_START("IN.0") // B0 port A low
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_16WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2) PORT_16WAY

	PORT_START("IN.1") // B1 port A low
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_16WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2) PORT_16WAY

	PORT_START("IN.2") // B2 port A low
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_16WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(2) PORT_16WAY

	PORT_START("IN.3") // B3 port A low
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_16WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2) PORT_16WAY

	PORT_START("IN.4") // port A high
	PORT_CONFNAME( 0x10, 0x10, DEF_STR( Players ) )
	PORT_CONFSETTING(    0x10, "1" )
	PORT_CONFSETTING(    0x00, "2" )
	PORT_CONFNAME( 0x20, 0x20, DEF_STR( Difficulty ) )
	PORT_CONFSETTING(    0x20, "1" ) // college
	PORT_CONFSETTING(    0x00, "2" ) // pro
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_SELECT ) PORT_TOGGLE PORT_NAME("Play Selector") // pass
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Kick/Pass") // K/P

	PORT_START("IN.5") // port B
	PORT_BIT( 0x7f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START ) PORT_NAME("Status/Score") // S
INPUT_PORTS_END

static MACHINE_CONFIG_START( us2pfball )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PIC1650, 800000) // approximation - RC osc. R=39K, C=75pF
	MCFG_PIC16C5x_READ_A_CB(READ8(us2pfball_state, read_a))
	MCFG_PIC16C5x_WRITE_A_CB(WRITE8(us2pfball_state, write_a))
	MCFG_PIC16C5x_READ_B_CB(IOPORT("IN.5"))
	MCFG_PIC16C5x_WRITE_B_CB(WRITE8(us2pfball_state, write_b))
	MCFG_PIC16C5x_READ_C_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_C_CB(WRITE8(us2pfball_state, write_c))
	MCFG_PIC16C5x_READ_D_CB(CONSTANT(0xff))
	MCFG_PIC16C5x_WRITE_D_CB(WRITE8(us2pfball_state, write_d))

	MCFG_DEVICE_ADD("clock", CLOCK, 800000/4) // PIC CLKOUT, tied to RTCC
	MCFG_CLOCK_SIGNAL_HANDLER(INPUTLINE("maincpu", PIC16C5x_RTCC))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("display_decay", hh_pic16_state, display_decay_tick, attotime::from_msec(1))
	MCFG_DEFAULT_LAYOUT(layout_us2pfball)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END





/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( touchme )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-053", 0x0000, 0x0400, CRC(f0858f0a) SHA1(53ffe111d43db1c110847590350ef62f02ed5e0e) )
ROM_END


ROM_START( pabball )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-043", 0x0000, 0x0400, CRC(43c9b765) SHA1(888a431bab9bcb241c14f33f70863fa2ad89c96b) )
ROM_END


ROM_START( melodym )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-094", 0x0000, 0x0400, CRC(6d35bd7b) SHA1(20e326085878f69a9d4ef1651ef4443f27188567) )
ROM_END


ROM_START( maniac )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-036", 0x0000, 0x0400, CRC(a96f7011) SHA1(e97ae44d3c1e74c7e1024bb0bdab03eecdc9f827) )
ROM_END


ROM_START( leboom )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-061", 0x0000, 0x0400, CRC(5880eea1) SHA1(e3795b347fd5df9de084da36e33f6b70fbc0b0ae) )
ROM_END


ROM_START( tbaskb )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-051", 0x0000, 0x0400, CRC(92534b40) SHA1(7055e32846c913e68f7d35f279cd537f6325f4f2) )
ROM_END


ROM_START( rockpin )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1650a-110_69-11397", 0x0000, 0x0400, CRC(d5396e77) SHA1(952feaff70fde53a9eda84c54704520d50749e78) )
ROM_END


ROM_START( hccbaskb )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "69-11557", 0x0000, 0x0400, CRC(56e81079) SHA1(1933f87f82c4c53f953534dba7757c9afc52d5bc) )
ROM_END


ROM_START( ttfball )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655a-033", 0x0000, 0x0400, CRC(2b500501) SHA1(f7fe464663c56e2181a31a1dc5f1f5239df57bed) )
ROM_END

ROM_START( ttfballa )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1655-024", 0x0000, 0x0400, CRC(9091102f) SHA1(ef72759f20b5a99e0366863caad1e26be114263f) )
ROM_END


ROM_START( uspbball )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1650a-133", 0x0000, 0x0400, CRC(479e98be) SHA1(67437177b059dfa6e01940da26daf997cec96ead) )
ROM_END


ROM_START( us2pfball )
	ROM_REGION( 0x0400, "maincpu", 0 )
	ROM_LOAD( "pic_1650a-144", 0x0000, 0x0400, CRC(ef3677c9) SHA1(33f89c79e7e090710681dffe09eddaf66b5cb794) )
ROM_END



//    YEAR  NAME       PARENT  CMP MACHINE    INPUT      STATE         INIT  COMPANY, FULLNAME, FLAGS
CONS( 1979, touchme,   0,       0, touchme,   touchme,   touchme_state,   0, "Atari", "Touch Me (handheld, Rev 2)", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1979, pabball,   0,       0, pabball,   pabball,   pabball_state,   0, "Caprice / Calfax", "Pro-Action Baseball", MACHINE_SUPPORTS_SAVE | MACHINE_NOT_WORKING )

CONS( 1980, melodym,   0,       0, melodym,   melodym,   melodym_state,   0, "GAF", "Melody Madness", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1979, maniac,    0,       0, maniac,    maniac,    maniac_state,    0, "Ideal", "Maniac", MACHINE_SUPPORTS_SAVE | MACHINE_CLICKABLE_ARTWORK )

CONS( 1980, leboom,    0,       0, leboom,    leboom,    leboom_state,    0, "Lakeside", "Le Boom", MACHINE_SUPPORTS_SAVE | MACHINE_IMPERFECT_SOUND | MACHINE_CLICKABLE_ARTWORK )

CONS( 1979, tbaskb,    0,       0, tbaskb,    tbaskb,    tbaskb_state,    0, "Tandy Radio Shack", "Electronic Basketball (Tandy)", MACHINE_SUPPORTS_SAVE )

CONS( 1979, rockpin,   0,       0, rockpin,   rockpin,   rockpin_state,   0, "Tiger Electronics", "Rocket Pinball", MACHINE_SUPPORTS_SAVE )
CONS( 1979, hccbaskb,  0,       0, hccbaskb,  hccbaskb,  hccbaskb_state,  0, "Tiger Electronics", "Half Court Computer Basketball", MACHINE_SUPPORTS_SAVE )

CONS( 1979, ttfball,   0,       0, ttfball,   ttfball,   ttfball_state,   0, "Toytronic", "Football (Toytronic, set 1)", MACHINE_SUPPORTS_SAVE | MACHINE_IMPERFECT_SOUND )
CONS( 1979, ttfballa,  ttfball, 0, ttfball,   ttfballa,  ttfball_state,   0, "Toytronic", "Football (Toytronic, set 2)", MACHINE_SUPPORTS_SAVE )

CONS( 1981, uspbball,  0,       0, uspbball,  uspbball,  uspbball_state,  0, "U.S. Games", "Programmable Baseball", MACHINE_SUPPORTS_SAVE | MACHINE_NOT_WORKING )
CONS( 1981, us2pfball, 0,       0, us2pfball, us2pfball, us2pfball_state, 0, "U.S. Games", "Electronic 2-Player Football", MACHINE_SUPPORTS_SAVE )
