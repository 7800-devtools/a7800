// license:BSD-3-Clause
// copyright-holders:Andrew Gardner
/*

    TODO:

    - LDV1000 mode
    - PR7820 INT/_EXT line
    - coin counter
    - convert SSI-263 to a sound device
    - dump laserdisc

*/

#include "emu.h"
#include "cpu/cop400/cop400.h"
#include "cpu/z80/z80.h"
#include "machine/ldstub.h"
#include "machine/ldv1000.h"
#include "speaker.h"

#include "thayers.lh"


#define LOG 0

struct ssi263_t
{
	uint8_t dr;
	uint8_t p;
	uint16_t i;
	uint8_t r;
	uint8_t t;
	uint8_t c;
	uint8_t a;
	uint8_t f;
	uint8_t mode;
};

class thayers_state : public driver_device
{
public:
	enum
	{
		TIMER_INTRQ_TICK,
		TIMER_SSI263_PHONEME_TICK
	};

	thayers_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_pr7820(*this, "laserdisc")
		, m_ldv1000(*this, "ldv1000")
		, m_maincpu(*this, "maincpu")
		, m_row(*this, "ROW.%u", 0)
	{
	}

	optional_device<pioneer_pr7820_device> m_pr7820;
	optional_device<pioneer_ldv1000_device> m_ldv1000;
	uint8_t m_laserdisc_data;
	int m_rx_bit;
	int m_keylatch;
	uint8_t m_keydata;
	bool m_kbdata;
	bool m_kbclk;
	uint8_t m_cop_data_latch;
	int m_cop_data_latch_enable;
	uint8_t m_cop_l;
	uint8_t m_cop_cmd_latch;
	int m_timer_int;
	int m_data_rdy_int;
	int m_ssi_data_request;
	int m_cart_present;
	int m_pr7820_enter;
	struct ssi263_t m_ssi263;
	DECLARE_WRITE8_MEMBER(intrq_w);
	DECLARE_READ8_MEMBER(irqstate_r);
	DECLARE_WRITE8_MEMBER(timer_int_ack_w);
	DECLARE_WRITE8_MEMBER(data_rdy_int_ack_w);
	DECLARE_WRITE8_MEMBER(cop_d_w);
	DECLARE_READ8_MEMBER(cop_data_r);
	DECLARE_WRITE8_MEMBER(cop_data_w);
	DECLARE_READ8_MEMBER(cop_l_r);
	DECLARE_WRITE8_MEMBER(cop_l_w);
	DECLARE_READ8_MEMBER(cop_g_r);
	DECLARE_WRITE8_MEMBER(control_w);
	DECLARE_WRITE8_MEMBER(cop_g_w);
	DECLARE_READ_LINE_MEMBER(kbdata_r);
	DECLARE_WRITE_LINE_MEMBER(kbclk_w);
	DECLARE_WRITE8_MEMBER(control2_w);
	DECLARE_READ8_MEMBER(dsw_b_r);
	DECLARE_READ8_MEMBER(laserdsc_data_r);
	DECLARE_WRITE8_MEMBER(laserdsc_data_w);
	DECLARE_WRITE8_MEMBER(laserdsc_control_w);
	DECLARE_WRITE8_MEMBER(den1_w);
	DECLARE_WRITE8_MEMBER(den2_w);
	DECLARE_WRITE8_MEMBER(ssi263_register_w);
	DECLARE_READ8_MEMBER(ssi263_register_r);
	DECLARE_CUSTOM_INPUT_MEMBER(laserdisc_enter_r);
	DECLARE_CUSTOM_INPUT_MEMBER(laserdisc_ready_r);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	void check_interrupt();
	required_device<cpu_device> m_maincpu;
	required_ioport_array<10> m_row;

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};

static const uint8_t led_map[16] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x00 };

/* Interrupts */

void thayers_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
	case TIMER_INTRQ_TICK:
		m_maincpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
		break;
	case TIMER_SSI263_PHONEME_TICK:
		m_ssi_data_request = 0;
		check_interrupt();
		break;
	default:
		assert_always(false, "Unknown id in thayers_state::device_timer");
	}
}

void thayers_state::check_interrupt()
{
	if (!m_timer_int || !m_data_rdy_int || !m_ssi_data_request)
	{
		m_maincpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);
	}
	else
	{
		m_maincpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
	}
}

WRITE8_MEMBER(thayers_state::intrq_w)
{
	// T = 1.1 * R30 * C53 = 1.1 * 750K * 0.01uF = 8.25 ms

	m_maincpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);

	timer_set(attotime::from_usec(8250), TIMER_INTRQ_TICK);
}

READ8_MEMBER(thayers_state::irqstate_r)
{
	/*

	    bit     description

	    0
	    1
	    2       SSI263 A/_R
	    3       tied to +5V
	    4       _TIMER INT
	    5       _DATA RDY INT
	    6       _CART PRES
	    7

	*/

	return m_cart_present << 6 | (m_data_rdy_int << 5) | (m_timer_int << 4) | 0x08 | (m_ssi_data_request << 2);
}

WRITE8_MEMBER(thayers_state::timer_int_ack_w)
{
	if (LOG) logerror("%s %s TIMER INT ACK\n", machine().time().as_string(), machine().describe_context());

	m_timer_int = 1;

	check_interrupt();
}

WRITE8_MEMBER(thayers_state::data_rdy_int_ack_w)
{
	if (LOG) logerror("%s %s DATA RDY INT ACK\n", machine().time().as_string(), machine().describe_context());

	m_data_rdy_int = 1;

	check_interrupt();
}

WRITE8_MEMBER(thayers_state::cop_d_w)
{
	/*

	    bit     description

	    D0      _TIMER INT
	    D1      _DATA RDY INT
	    D2
	    D3

	*/

	if (!BIT(data, 0))
	{
		if (LOG) logerror("%s %s TIMER INT\n", machine().time().as_string(), machine().describe_context());
		m_timer_int = 0;
	}

	if (!BIT(data, 1))
	{
		if (LOG) logerror("%s %s DATA RDY INT\n", machine().time().as_string(), machine().describe_context());
		m_data_rdy_int = 0;
	}

	check_interrupt();
}

/* COP Communication */

READ8_MEMBER(thayers_state::cop_data_r)
{
	if (!m_cop_data_latch_enable)
	{
		return m_cop_data_latch;
	}
	else
	{
		return m_cop_l;
	}
}

WRITE8_MEMBER(thayers_state::cop_data_w)
{
	m_cop_data_latch = data;
	if (LOG) logerror("COP DATA %02x\n", m_cop_data_latch);
}

READ8_MEMBER(thayers_state::cop_l_r)
{
	if (!m_cop_data_latch_enable)
	{
		return m_cop_data_latch;
	}
	else
	{
		return 0;
	}
}

WRITE8_MEMBER(thayers_state::cop_l_w)
{
	m_cop_l = data;
	if (LOG) logerror("COP L %02x\n", m_cop_l);
}

READ8_MEMBER(thayers_state::cop_g_r)
{
	/*

	    bit     description

	    G0      U16 Q0
	    G1      U16 Q1
	    G2      U16 Q2
	    G3

	*/

	return m_cop_cmd_latch;
}

WRITE8_MEMBER(thayers_state::control_w)
{
	/*

	    bit     description

	    0
	    1       _CS128A
	    2       _BANKSEL1
	    3
	    4
	    5       COP G0
	    6       COP G1
	    7       COP G2

	*/

	m_cop_cmd_latch = (data >> 5) & 0x07;
	if (LOG) logerror("COP G0..2 %u\n", m_cop_cmd_latch);
}

WRITE8_MEMBER(thayers_state::cop_g_w)
{
	/*

	    bit     description

	    G0
	    G1
	    G2
	    G3      U17 enable

	*/

	m_cop_data_latch_enable = BIT(data, 3);
	if (LOG) logerror("U17 enable %u\n", m_cop_data_latch_enable);
}

/* Keyboard */

READ_LINE_MEMBER(thayers_state::kbdata_r)
{
	if (LOG) logerror("%s KBDATA %u BIT %u\n",machine().time().as_string(),m_kbdata,m_rx_bit);
	return m_kbdata;
}

WRITE_LINE_MEMBER(thayers_state::kbclk_w)
{
	if (m_kbclk != state) {
		if (LOG) logerror("%s %s KBCLK %u\n", machine().time().as_string(), machine().describe_context(),state);
	}

	if (!m_kbclk && state) {
		m_rx_bit++;

		// 1, 1, 0, 1, Q9, P3, P2, P1, P0, 0
		switch (m_rx_bit)
		{
		case 0: case 1: case 3:
			m_kbdata = 1;
			break;

		case 2: case 9:
			m_kbdata = 0;
			break;

		case 10:
			m_rx_bit = 0;
			m_kbdata = 1;

			m_keylatch++;

			if (m_keylatch == 10) {
				m_keylatch = 0;
			}

			m_keydata = m_row[m_keylatch]->read();

			if (LOG) logerror("keylatch %u\n",m_keylatch);
			break;

		case 4:
			m_kbdata = (m_keylatch == 9);
			break;

		default:
			m_kbdata = BIT(m_keydata, 3);
			m_keydata <<= 1;

			if (LOG) logerror("keydata %02x shift\n",m_keydata);
			break;
		}
	}

	m_kbclk = state;
}

/* I/O Board */

WRITE8_MEMBER(thayers_state::control2_w)
{
	/*

	    bit     description

	    0
	    1       _RESOI (?)
	    2       _ENCARTDET
	    3
	    4
	    5
	    6
	    7

	*/

	if ((!BIT(data, 2)) & m_cart_present)
	{
		m_maincpu->set_input_line(INPUT_LINE_NMI, HOLD_LINE);
	}
}

READ8_MEMBER(thayers_state::dsw_b_r)
{
	return (ioport("COIN")->read() & 0xf0) | (ioport("DSWB")->read() & 0x0f);
}

READ8_MEMBER(thayers_state::laserdsc_data_r)
{
	if (m_ldv1000 != nullptr) return m_ldv1000->status_r();
	if (m_pr7820 != nullptr) return m_pr7820->data_r();
	return 0;
}

WRITE8_MEMBER(thayers_state::laserdsc_data_w)
{
	m_laserdisc_data = data;
}

WRITE8_MEMBER(thayers_state::laserdsc_control_w)
{
	/*

	    bit     description

	    0
	    1
	    2
	    3
	    4       coin counter
	    5       U16 output enable
	    6       ENTER if switch B5 closed
	    7       INT/_EXT

	*/

	machine().bookkeeping().coin_counter_w(0, BIT(data, 4));

	if (BIT(data, 5))
	{
		if (m_ldv1000 != nullptr)
		{
			m_ldv1000->data_w(m_laserdisc_data);
			m_ldv1000->enter_w(BIT(data, 7) ? CLEAR_LINE : ASSERT_LINE);
		}
		if (m_pr7820 != nullptr)
		{
			m_pr7820->data_w(m_laserdisc_data);
			m_pr7820_enter = BIT(data, 6) ? CLEAR_LINE : ASSERT_LINE;
			m_pr7820->enter_w(m_pr7820_enter);
			// BIT(data, 7) is INT/_EXT, but there is no such input line in laserdsc.h
		}
	}
}

WRITE8_MEMBER(thayers_state::den1_w)
{
	/*

	    bit     description

	    0       DD0
	    1       DD1
	    2       DD2
	    3       DD3
	    4       DA0
	    5       DA1
	    6       DA2
	    7       DA3

	*/

	output().set_digit_value(data >> 4, led_map[data & 0x0f]);
}

WRITE8_MEMBER(thayers_state::den2_w)
{
	/*

	    bit     description

	    0       DD0
	    1       DD1
	    2       DD2
	    3       DD3
	    4       DA0
	    5       DA1
	    6       DA2
	    7       DA3

	*/

	output().set_digit_value(8 + (data >> 4), led_map[data & 0x0f]);
}

/* SSI-263 */

/*

    The following information is from the SSI-263A data sheet.

    Thayer's Quest uses an SSI-263, so this might be inaccurate, but it works for now

*/

#define SSI263_CLOCK (XTAL_4MHz/2)

static const char SSI263_PHONEMES[0x40][5] =
{
	"PA", "E", "E1", "Y", "YI", "AY", "IE", "I", "A", "AI", "EH", "EH1", "AE", "AE1", "AH", "AH1", "W", "O", "OU", "OO", "IU", "IU1", "U", "U1", "UH", "UH1", "UH2", "UH3", "ER", "R", "R1", "R2",
	"L", "L1", "LF", "W", "B", "D", "KV", "P", "T", "K", "HV", "HVC", "HF", "HFC", "HN", "Z", "S", "J", "SCH", "V", "F", "THV", "TH", "M", "N", "NG", ":A", ":OH", ":U", ":UH", "E2", "LB"
};

WRITE8_MEMBER(thayers_state::ssi263_register_w)
{
	struct ssi263_t &ssi263 = m_ssi263;
	switch (offset)
	{
	case 0:
		{
		int frame_time = ((4096 * (16 - ssi263.r)) / 2); // us, /2 should actually be /SSI263_CLOCK, but this way we get microseconds directly
		int phoneme_time = frame_time * (4 - ssi263.dr); // us

		// duration/phoneme register
		ssi263.dr = (data >> 5) & 0x03;
		ssi263.p = data & 0x3f;

		m_ssi_data_request = 1;
		check_interrupt();

		switch (ssi263.mode)
		{
		case 0:
		case 1:
			// phoneme timing response
			timer_set(attotime::from_usec(phoneme_time), TIMER_SSI263_PHONEME_TICK);
			break;
		case 2:
			// frame timing response
			timer_set(attotime::from_usec(frame_time), TIMER_SSI263_PHONEME_TICK);
			break;
		case 3:
			// disable A/_R output
			break;
		}

		//logerror("SSI263 Phoneme Duration: %u\n", ssi263.dr);
		//logerror("SSI263 Phoneme: %02x %s\n", ssi263.p, SSI263_PHONEMES[ssi263.p]);
		if (LOG && ssi263.p) printf("%s ", SSI263_PHONEMES[ssi263.p]);
		}
		break;

	case 1:
		// inflection register
		ssi263.i = (data << 3) | (ssi263.i & 0x403);

		//logerror("SSI263 Inflection: %u\n", ssi263.i);
		break;

	case 2:
		// rate/inflection register
		ssi263.i = (BIT(data, 4) << 11) | (ssi263.i & 0x7f8) | (data & 0x07);
		ssi263.r = data >> 4;

		//logerror("SSI263 Inflection: %u\n", ssi263.i);
		//logerror("SSI263 Rate: %u\n", ssi263.r);
		break;

	case 3:
		// control/articulation/amplitude register
		if (ssi263.c && !BIT(data, 7))
		{
			ssi263.mode = ssi263.dr;

			switch (ssi263.mode)
			{
			case 0:
				//logerror("SSI263 Phoneme Timing Response, Transitioned Inflection\n");
				break;

			case 1:
				//logerror("SSI263 Phoneme Timing Response, Immediate Inflection\n");
				break;

			case 2:
				// activate A/_R
				//logerror("SSI263 Frame Timing Response, Immediate Inflection\n");
				break;

			case 3:
				// disable A/_R output
				//logerror("SSI263 A/R Output Disabled\n");
				break;
			}
		}

		ssi263.c = BIT(data, 7);
		ssi263.t = (data >> 4) & 0x07;
		ssi263.a = data & 0x0f;

		//logerror("SSI263 Control: %u\n", ssi263.c);
		//logerror("SSI263 Articulation: %u\n", ssi263.t);
		//logerror("SSI263 Amplitude: %u\n", ssi263.a);
		break;

	case 4:
	case 5:
	case 6:
	case 7:
		// filter frequency register
		ssi263.f = data;
		//logerror("SSI263 Filter Frequency: %u\n", ssi263.f);
		break;
	}
}

READ8_MEMBER(thayers_state::ssi263_register_r)
{
	// D7 becomes an output, as the inverted state of A/_R. The register address bits are ignored.

	return !m_ssi_data_request << 7;
}

/* Memory Maps */

static ADDRESS_MAP_START( thayers_map, AS_PROGRAM, 8, thayers_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_RAM
	AM_RANGE(0xc000, 0xdfff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( thayers_io_map, AS_IO, 8, thayers_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x07) AM_READWRITE(ssi263_register_r, ssi263_register_w)
	AM_RANGE(0x20, 0x20) AM_WRITE(control_w)
	AM_RANGE(0x40, 0x40) AM_READWRITE(irqstate_r, control2_w)
	AM_RANGE(0x80, 0x80) AM_READWRITE(cop_data_r, cop_data_w)
	AM_RANGE(0xa0, 0xa0) AM_WRITE(timer_int_ack_w)
	AM_RANGE(0xc0, 0xc0) AM_WRITE(data_rdy_int_ack_w)
	AM_RANGE(0xf0, 0xf0) AM_READ(laserdsc_data_r)
	AM_RANGE(0xf1, 0xf1) AM_READ(dsw_b_r)
	AM_RANGE(0xf2, 0xf2) AM_READ_PORT("DSWA")
	AM_RANGE(0xf3, 0xf3) AM_WRITE(intrq_w)
	AM_RANGE(0xf4, 0xf4) AM_WRITE(laserdsc_data_w)
	AM_RANGE(0xf5, 0xf5) AM_WRITE(laserdsc_control_w)
	AM_RANGE(0xf6, 0xf6) AM_WRITE(den1_w)
	AM_RANGE(0xf7, 0xf7) AM_WRITE(den2_w)
ADDRESS_MAP_END

/* Input Ports */

CUSTOM_INPUT_MEMBER(thayers_state::laserdisc_enter_r)
{
	if (m_pr7820 != nullptr) return m_pr7820_enter;
	if (m_ldv1000 != nullptr) return (m_ldv1000->status_strobe_r() == ASSERT_LINE) ? 0 : 1;
	return 0;
}

CUSTOM_INPUT_MEMBER(thayers_state::laserdisc_ready_r)
{
	if (m_pr7820 != nullptr) return (m_pr7820->ready_r() == ASSERT_LINE) ? 0 : 1;
	if (m_ldv1000 != nullptr) return (m_ldv1000->command_strobe_r() == ASSERT_LINE) ? 0 : 1;
	return 0;
}

static INPUT_PORTS_START( thayers )
	PORT_START("DSWA")
	PORT_DIPNAME( 0x07, 0x07, "Time Per Coin" ) PORT_DIPLOCATION( "A:3,2,1" )
	PORT_DIPSETTING(    0x07, "110 Seconds" )
	PORT_DIPSETTING(    0x06, "95 Seconds" )
	PORT_DIPSETTING(    0x05, "80 Seconds" )
	PORT_DIPSETTING(    0x04, "70 Seconds" )
	PORT_DIPSETTING(    0x03, "60 Seconds" )
	PORT_DIPSETTING(    0x02, "45 Seconds" )
	PORT_DIPSETTING(    0x01, "30 Seconds" )
	PORT_DIPSETTING(    0x00, DEF_STR ( Free_Play ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Coinage ) ) PORT_DIPLOCATION( "A:4" )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_1C ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Lives ) ) PORT_DIPLOCATION( "A:5" )
	PORT_DIPSETTING(    0x10, "3" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION( "A:6" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, "Attract Mode Audio" ) PORT_DIPLOCATION( "A:7" )
	PORT_DIPSETTING(    0x40, "Always Playing" )
	PORT_DIPSETTING(    0x00, "One Out of 8 Times" )
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "A:8" )

	PORT_START("DSWB")
	PORT_SERVICE_DIPLOC( 0x01, 0x01, "B:1" )
	PORT_DIPUNUSED_DIPLOC( 0x02, IP_ACTIVE_LOW, "B:2" )
	PORT_DIPUNUSED_DIPLOC( 0x04, IP_ACTIVE_LOW, "B:3" )
	PORT_DIPNAME( 0x18, 0x00, "LD Player" ) PORT_DIPLOCATION( "B:5,4" )
	PORT_DIPSETTING(    0x18, "LDV-1000" )
	PORT_DIPSETTING(    0x00, "PR-7820" )
	PORT_DIPUNUSED_DIPLOC( 0xe0, IP_ACTIVE_LOW, "B:8,7,6" )

	PORT_START("COIN")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, thayers_state,laserdisc_enter_r, nullptr)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, thayers_state,laserdisc_ready_r, nullptr)

	PORT_START("ROW.0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("1 YES") PORT_CODE(KEYCODE_1)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_Q)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F1 CLEAR") PORT_CODE(KEYCODE_F1) PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_F2)

	PORT_START("ROW.1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("2 ITEMS") PORT_CODE(KEYCODE_2)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("W AMULET") PORT_CODE(KEYCODE_W)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_A)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Z SPELL OF RELEASE") PORT_CODE(KEYCODE_Z)

	PORT_START("ROW.2")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("3 DROP ITEM") PORT_CODE(KEYCODE_3)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("E BLACK MACE") PORT_CODE(KEYCODE_E)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("S DAGGER") PORT_CODE(KEYCODE_S)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("X SCEPTER") PORT_CODE(KEYCODE_X)

	PORT_START("ROW.3")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("4 GIVE SCORE") PORT_CODE(KEYCODE_4)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("R BLOOD SWORD") PORT_CODE(KEYCODE_R)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("D GREAT CIRCLET") PORT_CODE(KEYCODE_D)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("C SPELL OF SEEING") PORT_CODE(KEYCODE_C)

	PORT_START("ROW.4")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("5 REPLAY") PORT_CODE(KEYCODE_5)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("T CHALICE") PORT_CODE(KEYCODE_T)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F HUNTING HORN") PORT_CODE(KEYCODE_F)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("V SHIELD") PORT_CODE(KEYCODE_V)

	PORT_START("ROW.5")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("6 COMBINE ACTION") PORT_CODE(KEYCODE_6)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Y COINS") PORT_CODE(KEYCODE_Y)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("G LONG BOW") PORT_CODE(KEYCODE_G)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("B SILVER WHEAT") PORT_CODE(KEYCODE_B)

	PORT_START("ROW.6")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("7 SAVE GAME") PORT_CODE(KEYCODE_7)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("U COLD FIRE") PORT_CODE(KEYCODE_U)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("H MEDALLION") PORT_CODE(KEYCODE_H)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("N STAFF") PORT_CODE(KEYCODE_N)

	PORT_START("ROW.7")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("8 UPDATE") PORT_CODE(KEYCODE_8)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("I CROWN") PORT_CODE(KEYCODE_I)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("J ONYX SEAL") PORT_CODE(KEYCODE_J)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("M SPELL OF UNDERSTANDING") PORT_CODE(KEYCODE_M)

	PORT_START("ROW.8")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("9 HINT") PORT_CODE(KEYCODE_9)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("O CRYSTAL") PORT_CODE(KEYCODE_O)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("K ORB OF QUOID") PORT_CODE(KEYCODE_K)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F4 SPACE") PORT_CODE(KEYCODE_F4) PORT_CODE(KEYCODE_SPACE)

	PORT_START("ROW.9")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("0 NO") PORT_CODE(KEYCODE_0)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_P)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_CODE(KEYCODE_L)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("F3 ENTER") PORT_CODE(KEYCODE_F3) PORT_CODE(KEYCODE_ENTER)
INPUT_PORTS_END

/* Machine Initialization */

void thayers_state::machine_start()
{
	memset(&m_ssi263, 0, sizeof(m_ssi263));
}

void thayers_state::machine_reset()
{
	m_laserdisc_data = 0;

	m_rx_bit = 0;
	m_kbdata = 1;
	m_keylatch = 0;
	m_keydata = m_row[m_keylatch]->read();

	m_cop_data_latch = 0;
	m_cop_data_latch_enable = 0;
	m_cop_l = 0;
	m_cop_cmd_latch = 0;

	m_timer_int = 1;
	m_data_rdy_int = 1;
	m_ssi_data_request = 1;

	m_cart_present = 0;
	m_pr7820_enter = 0;
}

/* Machine Driver */

static MACHINE_CONFIG_START( thayers )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_4MHz)
	MCFG_CPU_PROGRAM_MAP(thayers_map)
	MCFG_CPU_IO_MAP(thayers_io_map)

	MCFG_CPU_ADD("mcu", COP421, XTAL_4MHz/2) // COP421L-PCA/N
	MCFG_COP400_CONFIG( COP400_CKI_DIVISOR_16, COP400_CKO_OSCILLATOR_OUTPUT, false )
	MCFG_COP400_READ_L_CB(READ8(thayers_state, cop_l_r))
	MCFG_COP400_WRITE_L_CB(WRITE8(thayers_state, cop_l_w))
	MCFG_COP400_READ_G_CB(READ8(thayers_state, cop_g_r))
	MCFG_COP400_WRITE_G_CB(WRITE8(thayers_state, cop_g_w))
	MCFG_COP400_WRITE_D_CB(WRITE8(thayers_state, cop_d_w))
	MCFG_COP400_READ_SI_CB(READLINE(thayers_state, kbdata_r))
	MCFG_COP400_WRITE_SO_CB(WRITELINE(thayers_state, kbclk_w))

	MCFG_LASERDISC_PR7820_ADD("laserdisc")

	/* video hardware */
	MCFG_LASERDISC_SCREEN_ADD_NTSC("screen", "laserdisc")

	MCFG_PALETTE_ADD("palette", 256)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	// SSI 263 @ 2MHz

	MCFG_SOUND_MODIFY("laserdisc")
	MCFG_SOUND_ROUTE(0, "lspeaker", 1.0)
	MCFG_SOUND_ROUTE(1, "rspeaker", 1.0)
MACHINE_CONFIG_END

/* ROMs */

ROM_START( thayers )
	ROM_REGION( 0xe000, "maincpu", 0 )
	ROM_LOAD( "tq_u33.bin", 0x0000, 0x8000, CRC(82df5d89) SHA1(58dfd62bf8c5a55d1eba397d2c284e99a4685a3f) )
	ROM_LOAD( "tq_u1.bin",  0xc000, 0x2000, CRC(e8e7f566) SHA1(df7b83ef465c65446c8418bc6007447693b75021) )

	ROM_REGION( 0x400, "mcu", 0 )
	ROM_LOAD( "tq_cop.bin", 0x000, 0x400, CRC(6748e6b3) SHA1(5d7d1ecb57c1501ef6a2d9691eecc9970586606b) )

	DISK_REGION( "laserdisc" )
	DISK_IMAGE_READONLY( "thayers", 0, NO_DUMP )
ROM_END

ROM_START( thayersa )
	ROM_REGION( 0xe000, "maincpu", 0 )
	ROM_LOAD( "tq_u33.bin", 0x0000, 0x8000, CRC(82df5d89) SHA1(58dfd62bf8c5a55d1eba397d2c284e99a4685a3f) )
	ROM_LOAD( "tq_u1.bin",  0xc000, 0x2000, CRC(33817e25) SHA1(f9750da863dd57fe2f5b6e8fce9c6695dc5c9adc) ) // sldh

	ROM_REGION( 0x400, "mcu", 0 )
	ROM_LOAD( "tq_cop.bin", 0x000, 0x400, CRC(6748e6b3) SHA1(5d7d1ecb57c1501ef6a2d9691eecc9970586606b) )

	DISK_REGION( "laserdisc" )
	DISK_IMAGE_READONLY( "thayers", 0, NO_DUMP )
ROM_END

/* Game Drivers */

//     YEAR  NAME      PARENT   MACHINE  INPUT    STATE          INIT  MONITOR  COMPANY               FULLNAME                   FLAGS                                   LAYOUT
GAMEL( 1984, thayers,  0,       thayers, thayers, thayers_state, 0,    ROT0,    "RDI Video Systems",  "Thayer's Quest (set 1)",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND, layout_thayers)
GAMEL( 1984, thayersa, thayers, thayers, thayers, thayers_state, 0,    ROT0,    "RDI Video Systems",  "Thayer's Quest (set 2)",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND, layout_thayers)
