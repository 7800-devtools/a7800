// license:BSD-3-Clause
// copyright-holders:Vas Crabb
/*
High-speed CMOS 6805-compatible microcontrollers

The M68HC05 family uses the M6805 instruction set with a few additions
but runs at two clocks per machine cycle, and has incompatible on-board
peripherals.  It comes in mask ROM (M68HC05), EPROM (M68HC705) and
EEPROM (M68HC805) variants.  The suffix gives some indication of the
memory sizes and on-board peripherals, but there's not a lot of
consistency across the ROM/EPROM/EEPROM variants.

All devices in this family have a 16-bit free-running counter fed from
the internal clock.  The counter value can be captured on an input edge,
and an output can be automatically set when the counter reaches a
certain value.
*/
#include "emu.h"
#include "m68hc05.h"
#include "m6805defs.h"


/****************************************************************************
 * Configurable logging
 ****************************************************************************/

#define LOG_GENERAL (1U <<  0)
#define LOG_INT     (1U <<  1)
#define LOG_IOPORT  (1U <<  2)
#define LOG_TIMER   (1U <<  3)
#define LOG_COP     (1U <<  4)

//#define VERBOSE (LOG_GENERAL | LOG_INT | LOG_IOPORT | LOG_TIMER | LOG_COP)
//#define LOG_OUTPUT_FUNC printf
#include "logmacro.h"

#define LOGINT(...)     LOGMASKED(LOG_INT,    __VA_ARGS__)
#define LOGIOPORT(...)  LOGMASKED(LOG_IOPORT, __VA_ARGS__)
#define LOGTIMER(...)   LOGMASKED(LOG_TIMER,  __VA_ARGS__)
#define LOGCOP(...)     LOGMASKED(LOG_COP,    __VA_ARGS__)


namespace {

std::pair<u16, char const *> const m68hc05c4_syms[] = {
	{ 0x0000, "PORTA"  }, { 0x0001, "PORTB"  }, { 0x0002, "PORTC"  }, { 0x0003, "PORTD"  },
	{ 0x0004, "DDRA"   }, { 0x0005, "DDRB"   }, { 0x0006, "DDRC"   },
	{ 0x000a, "SPCR"   }, { 0x000b, "SPSR"   }, { 0x000c, "SPDR"   },
	{ 0x000d, "BAUD"   }, { 0x000e, "SCCR1"  }, { 0x000f, "SCCR2"  }, { 0x0010, "SCSR"   }, { 0x0011, "SCDR"   },
	{ 0x0012, "TCR"    }, { 0x0013, "TSR"    },
	{ 0x0014, "ICRH"   }, { 0x0015, "ICRL"   }, { 0x0016, "OCRH"   }, { 0x0017, "OCRL"   },
	{ 0x0018, "TRH"    }, { 0x0019, "TRL"    }, { 0x001a, "ATRH"   }, { 0x001b, "ATRL"   } };

std::pair<u16, char const *> const m68hc705c8a_syms[] = {
	{ 0x0000, "PORTA"  }, { 0x0001, "PORTB" }, { 0x0002, "PORTC" }, { 0x0003, "PORTD" },
	{ 0x0004, "DDRA"   }, { 0x0005, "DDRB"  }, { 0x0006, "DDRC"  },
	{ 0x000a, "SPCR"   }, { 0x000b, "SPSR"  }, { 0x000c, "SPDR"  },
	{ 0x000d, "BAUD"   }, { 0x000e, "SCCR1" }, { 0x000f, "SCCR2" }, { 0x0010, "SCSR"  }, { 0x0011, "SCDR"  },
	{ 0x0012, "TCR"    }, { 0x0013, "TSR"   },
	{ 0x0014, "ICRH"   }, { 0x0015, "ICRL"  }, { 0x0016, "OCRH"  }, { 0x0017, "OCRL"  },
	{ 0x0018, "TRH"    }, { 0x0019, "TRL"   }, { 0x001a, "ATRH"  }, { 0x001b, "ATRL"  },
	{ 0x001c, "PROG"   },
	{ 0x001d, "COPRST" }, { 0x001e, "COPCR" } };


ROM_START( m68hc705c8a )
	ROM_REGION(0x00f0, "bootstrap", 0)
	ROM_LOAD("bootstrap.bin", 0x0000, 0x00f0, NO_DUMP)
ROM_END


//constexpr u16 M68HC05_VECTOR_SPI        = 0xfff4;
//constexpr u16 M68HC05_VECTOR_SCI        = 0xfff6;
constexpr u16 M68HC05_VECTOR_TIMER      = 0xfff8;
constexpr u16 M68HC05_VECTOR_IRQ        = 0xfffa;
constexpr u16 M68HC05_VECTOR_SWI        = 0xfffc;
//constexpr u16 M68HC05_VECTOR_RESET      = 0xfffe;

constexpr u16 M68HC05_INT_IRQ           = u16(1) << 0;
constexpr u16 M68HC05_INT_TIMER         = u16(1) << 1;

constexpr u16 M68HC05_INT_MASK          = M68HC05_INT_IRQ | M68HC05_INT_TIMER;

} // anonymous namespace



/****************************************************************************
 * Global variables
 ****************************************************************************/

DEFINE_DEVICE_TYPE(M68HC05C4,   m68hc05c4_device,   "m68hc05c4",   "MC68HC05C4")
DEFINE_DEVICE_TYPE(M68HC05C8,   m68hc05c8_device,   "m68hc05c8",   "MC68HC05C8")
DEFINE_DEVICE_TYPE(M68HC705C8A, m68hc705c8a_device, "m68hc705c8a", "MC68HC705C8A")



/****************************************************************************
 * M68HC05 base device
 ****************************************************************************/

m68hc05_device::m68hc05_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock,
		device_type type,
		address_map_delegate internal_map)
	: m6805_base_device(
			mconfig,
			tag,
			owner,
			clock,
			type,
			{ s_hc_ops, s_hc_cycles, 13, 0x00ff, 0x00c0, M68HC05_VECTOR_SWI },
			internal_map)
	, m_port_cb_r{ *this, *this, *this, *this }
	, m_port_cb_w{ *this, *this, *this, *this }
	, m_port_bits{ 0xff, 0xff, 0xff, 0xff }
	, m_port_interrupt{ 0x00, 0x00, 0x00, 0x00 }
	, m_port_input{ 0xff, 0xff, 0xff, 0xff }
	, m_port_latch{ 0xff, 0xff, 0xff, 0xff }
	, m_port_ddr{ 0x00, 0x00, 0x00, 0x00 }
	, m_port_irq_state(false)
	, m_irq_line_state(false)
	, m_irq_latch(0)
	, m_tcmp_cb(*this)
	, m_tcap_state(false)
	, m_tcr(0x00)
	, m_tsr(0x00), m_tsr_seen(0x00)
	, m_prescaler(0x00)
	, m_counter(0xfffc), m_icr(0x0000), m_ocr(0x0000)
	, m_inhibit_cap(false), m_inhibit_cmp(false)
	, m_trl_buf{ 0xfc, 0xfc }
	, m_trl_latched{ false, false }
	, m_pcop_cnt(0)
	, m_ncop_cnt(0)
	, m_coprst(0x00)
	, m_copcr(0x00)
	, m_ncope(0)
{
}


void m68hc05_device::set_port_bits(std::array<u8, PORT_COUNT> const &bits)
{
	if (configured() || started())
		throw emu_fatalerror("Attempt to set physical port bits after configuration");

	for (unsigned i = 0; PORT_COUNT > i; ++i)
		m_port_bits[i] = bits[i];
}

void m68hc05_device::set_port_interrupt(std::array<u8, PORT_COUNT> const &interrupt)
{
	u8 diff(0x00);
	for (unsigned i = 0; PORT_COUNT > i; ++i)
	{
		diff |= (m_port_interrupt[i] ^ interrupt[i]) & ~m_port_ddr[i];
		m_port_interrupt[i] = interrupt[i];
		if (interrupt[i] && !m_port_cb_r[i].isnull())
			logerror("PORT%c has interrupts enabled with pulled inputs, behaviour may be incorrect\n", 'A' + i);
	}
	if (diff) update_port_irq();
}

READ8_MEMBER(m68hc05_device::port_r)
{
	offset &= PORT_COUNT - 1;
	if (!machine().side_effect_disabled() && !m_port_cb_r[offset].isnull())
	{
		u8 const newval(m_port_cb_r[offset](space, 0, ~m_port_ddr[offset] & m_port_bits[offset]) & m_port_bits[offset]);
		u8 const diff(newval ^ m_port_input[offset]);
		if (diff)
		{
			LOGIOPORT("read PORT%c: new input = %02X & %02X (was %02X)\n",
					char('A' + offset), newval, ~m_port_ddr[offset] & m_port_bits[offset], m_port_input[offset]);
		}
		m_port_input[offset] = newval;
		if (diff & m_port_interrupt[offset] & ~m_port_ddr[offset])
			update_port_irq();
	}
	return port_value(offset);
}

WRITE8_MEMBER(m68hc05_device::port_latch_w)
{
	offset &= PORT_COUNT - 1;
	data &= m_port_bits[offset];
	u8 const diff = m_port_latch[offset] ^ data;
	if (diff)
	{
		LOGIOPORT("write PORT%c latch: %02X & %02X (was %02X)\n",
				char('A' + offset), data, m_port_ddr[offset], m_port_latch[offset]);
	}
	m_port_latch[offset] = data;
	if (diff & m_port_ddr[offset])
		m_port_cb_w[offset](space, 0, port_value(offset), m_port_ddr[offset]);
}

READ8_MEMBER(m68hc05_device::port_ddr_r)
{
	return m_port_ddr[offset & (PORT_COUNT - 1)];
}

WRITE8_MEMBER(m68hc05_device::port_ddr_w)
{
	offset &= PORT_COUNT - 1;
	data &= m_port_bits[offset];
	u8 const diff(data ^ m_port_ddr[offset]);
	if (diff)
	{
		LOGIOPORT("write DDR%c: %02X (was %02X)\n", char('A' + offset), data, m_port_ddr[offset]);
		m_port_ddr[offset] = data;
		if (diff & m_port_interrupt[offset])
		{
			if (!m_port_cb_r[offset].isnull())
			{
				u8 const newval(m_port_cb_r[offset](space, 0, ~m_port_ddr[offset] & m_port_bits[offset]) & m_port_bits[offset]);
				u8 const diff(newval ^ m_port_input[offset]);
				if (diff)
				{
					LOGIOPORT("read PORT%c: new input = %02X & %02X (was %02X)\n",
							char('A' + offset), newval, ~m_port_ddr[offset] & m_port_bits[offset], m_port_input[offset]);
				}
				m_port_input[offset] = newval;
			}
			update_port_irq();
		}
		m_port_cb_w[offset](space, 0, port_value(offset), m_port_ddr[offset]);
	}
}


READ8_MEMBER(m68hc05_device::tcr_r)
{
	return m_tcr;
}

WRITE8_MEMBER(m68hc05_device::tcr_w)
{
	data &= 0xe3;
	LOGTIMER("write TCR: ICIE=%u OCIE=%u TOIE=%u IEDG=%u OLVL=%u\n",
			BIT(data, 7), BIT(data, 6), BIT(data, 5), BIT(data, 1), BIT(data, 0));
	m_tcr = data;
	if (m_tcr & m_tsr & 0xe0)
		m_pending_interrupts |= M68HC05_INT_TIMER;
	else
		m_pending_interrupts &= ~M68HC05_INT_TIMER;
}

READ8_MEMBER(m68hc05_device::tsr_r)
{
	if (!machine().side_effect_disabled())
	{
		u8 const events(m_tsr & ~m_tsr_seen);
		if (events)
		{
			LOGTIMER("read TSR: seen%s%s%s\n",
					BIT(events, 7) ? " ICF" : "", BIT(events, 6) ? " OCF" : "", BIT(events, 5) ? " TOF" : "");
		}
		m_tsr_seen = m_tsr;
	}
	return m_tsr;
}

READ8_MEMBER(m68hc05_device::icr_r)
{
	// reading IRCH inhibits capture until ICRL is read
	// reading ICRL after reading TCR with ICF set clears ICF

	u8 const low(BIT(offset, 0));
	if (!machine().side_effect_disabled())
	{
		if (low)
		{
			if (BIT(m_tsr_seen, 7))
			{
				LOGTIMER("read ICRL, clear ICF\n");
				m_tsr &= 0x7f;
				m_tsr_seen &= 0x7f;
				if (!(m_tcr & m_tsr & 0xe0)) m_pending_interrupts &= ~M68HC05_INT_TIMER;
			}
			if (m_inhibit_cap) LOGTIMER("read ICRL, enable capture\n");
			m_inhibit_cap = false;
		}
		else
		{
			if (!m_inhibit_cap) LOGTIMER("read ICRH, inhibit capture\n");
			m_inhibit_cap = true;
		}
	}
	return u8(m_icr >> (low ? 0 : 8));
}

READ8_MEMBER(m68hc05_device::ocr_r)
{
	// reading OCRL after reading TCR with OCF set clears OCF

	u8 const low(BIT(offset, 0));
	if (!machine().side_effect_disabled() && low && BIT(m_tsr_seen, 6))
	{
		LOGTIMER("read OCRL, clear OCF\n");
		m_tsr &= 0xbf;
		m_tsr_seen &= 0xbf;
		if (!(m_tcr & m_tsr & 0xe0)) m_pending_interrupts &= ~M68HC05_INT_TIMER;
	}
	return u8(m_ocr >> (low ? 0 : 8));
}

WRITE8_MEMBER(m68hc05_device::ocr_w)
{
	// writing ORCH inhibits compare until OCRL is written
	// writing OCRL after reading TCR with OCF set clears OCF

	u8 const low(BIT(offset, 0));
	if (!machine().side_effect_disabled())
	{
		if (low)
		{
			if (BIT(m_tsr_seen, 6))
			{
				LOGTIMER("write OCRL, clear OCF\n");
				m_tsr &= 0xbf;
				m_tsr_seen &= 0xbf;
				if (!(m_tcr & m_tsr & 0xe0)) m_pending_interrupts &= ~M68HC05_INT_TIMER;
			}
			if (m_inhibit_cmp) LOGTIMER("write OCRL, enable compare\n");
			m_inhibit_cmp = false;
		}
		else
		{
			if (!m_inhibit_cmp) LOGTIMER("write OCRH, inhibit compare\n");
			m_inhibit_cmp = true;
		}
	}
	m_ocr = (m_ocr & (low ? 0xff00 : 0x00ff)) | (u16(data) << (low ? 0 : 8));
}

READ8_MEMBER(m68hc05_device::timer_r)
{
	// reading [A]TRH returns current counter MSB and latches [A]TRL buffer
	// reading [A]TRL returns current [A]TRL buffer and completes read sequence
	// reading TRL after reading TSR with TOF set clears TOF
	// reading ATRL doesn't affect TOF

	u8 const low(BIT(offset, 0));
	u8 const alt(BIT(offset, 1));
	if (low)
	{
		if (!machine().side_effect_disabled())
		{
			if (m_trl_latched[alt]) LOGTIMER("read %sTRL, read sequence complete\n", alt ? "A" : "");
			m_trl_latched[alt] = false;
			if (!alt && BIT(m_tsr_seen, 5))
			{
				LOGTIMER("read TRL, clear TOF\n");
				m_tsr &= 0xdf;
				m_tsr_seen &= 0xdf;
				if (!(m_tcr & m_tsr & 0xe0)) m_pending_interrupts &= ~M68HC05_INT_TIMER;
			}
		}
		return m_trl_buf[alt];
	}
	else
	{
		if (!machine().side_effect_disabled() && !m_trl_latched[alt])
		{
			LOGTIMER("read %sTRH, latch %sTRL\n", alt ? "A" : "", alt ? "A" : "");
			m_trl_latched[alt] = true;
			m_trl_buf[alt] = u8(m_counter);
		}
		return u8(m_counter >> 8);
	}
}


WRITE8_MEMBER(m68hc05_device::coprst_w)
{
	LOGCOP("write COPRST=%02x%s\n", data, ((0xaa == data) && (0x55 == m_coprst)) ? ", reset" : "");
	if (0x55 == data)
	{
		m_coprst = data;
	}
	else if (0xaa == data)
	{
		if (0x55 == m_coprst) m_pcop_cnt &= 0x00007fff;
		m_coprst = data;
	}
}

READ8_MEMBER(m68hc05_device::copcr_r)
{
	if (copcr_copf()) LOGCOP("read COPCR, clear COPF\n");
	u8 const result(m_copcr);
	m_copcr &= 0xef;
	return result;
}

WRITE8_MEMBER(m68hc05_device::copcr_w)
{
	LOGCOP("write COPCR: CME=%u PCOPE=%u [%s] CM=%u\n",
			BIT(data, 3), BIT(data, 2), (!copcr_pcope() && BIT(data, 2)) ? "set" : "ignored", data & 0x03);
	m_copcr = (m_copcr & 0xf4) | (data & 0x0f); // PCOPE is set-only, hence the mask overlap
}

WRITE8_MEMBER(m68hc05_device::copr_w)
{
	LOGCOP("write COPR: COPC=%u\n", BIT(data, 0));
	if (!BIT(data, 0)) m_ncop_cnt = 0;
}


void m68hc05_device::device_start()
{
	m6805_base_device::device_start();

	// resolve callbacks
	for (devcb_read8 &cb : m_port_cb_r) cb.resolve();
	for (devcb_write8 &cb : m_port_cb_w) cb.resolve_safe();
	m_tcmp_cb.resolve_safe();

	// save digital I/O
	save_item(NAME(m_port_interrupt));
	save_item(NAME(m_port_input));
	save_item(NAME(m_port_latch));
	save_item(NAME(m_port_ddr));
	save_item(NAME(m_port_irq_state));
	save_item(NAME(m_irq_line_state));
	save_item(NAME(m_irq_latch));

	// save timer/counter
	save_item(NAME(m_tcap_state));
	save_item(NAME(m_tcr));
	save_item(NAME(m_tsr));
	save_item(NAME(m_tsr_seen));
	save_item(NAME(m_prescaler));
	save_item(NAME(m_counter));
	save_item(NAME(m_icr));
	save_item(NAME(m_ocr));
	save_item(NAME(m_inhibit_cap));;
	save_item(NAME(m_inhibit_cmp));
	save_item(NAME(m_trl_buf));
	save_item(NAME(m_trl_latched));

	// save COP watchdogs
	save_item(NAME(m_pcop_cnt));
	save_item(NAME(m_ncop_cnt));
	save_item(NAME(m_coprst));
	save_item(NAME(m_copcr));
	save_item(NAME(m_ncope));

	// digital I/O state unaffected by reset
	std::fill(std::begin(m_port_interrupt), std::end(m_port_interrupt), 0x00);
	std::fill(std::begin(m_port_input), std::end(m_port_input), 0xff);
	std::fill(std::begin(m_port_latch), std::end(m_port_latch), 0xff);
	m_irq_line_state = false;

	// timer state unaffected by reset
	m_tcap_state = false;
	m_tcr = 0x00;
	m_tsr = 0x00;
	m_icr = 0x0000;
	m_ocr = 0x0000;

	// COP watchdog state unaffected by reset
	m_pcop_cnt = 0;
	m_coprst = 0x00;
	m_copcr = 0x00;
	m_ncope = 0;

	// expose most basic state to debugger
	state_add(M68HC05_IRQLATCH, "IRQLATCH", m_irq_latch).mask(0x01);
}

void m68hc05_device::device_reset()
{
	m6805_base_device::device_reset();

	// digital I/O reset
	std::fill(std::begin(m_port_ddr), std::end(m_port_ddr), 0x00);
	m_irq_latch = 0;
	update_port_irq();

	// timer reset
	m_tcr &= 0x02;
	m_tsr_seen = 0x00;
	m_prescaler = 0;
	m_counter = 0xfffc;
	m_inhibit_cap = m_inhibit_cmp = false;
	m_trl_buf[0] = m_trl_buf[1] = u8(m_counter);
	m_trl_latched[0] = m_trl_latched[1] = false;

	// COP watchdog reset
	m_ncop_cnt = 0;
	m_copcr &= 0x10;
}


void m68hc05_device::execute_set_input(int inputnum, int state)
{
	switch (inputnum)
	{
	case M68HC05_IRQ_LINE:
		if ((CLEAR_LINE != state) && !m_irq_line_state)
		{
			LOGINT("/IRQ edge%s\n", (m_port_irq_state || m_irq_latch) ? "" : ", set IRQ latch");
			if (!m_port_irq_state)
			{
				m_irq_latch = 1;
				m_pending_interrupts |= M68HC05_INT_IRQ;
			}
		}
		m_irq_line_state = ASSERT_LINE == state;
		break;
	case M68HC05_TCAP_LINE:
		if ((bool(state) != m_tcap_state) && (bool(state) == tcr_iedg()))
		{
			LOGTIMER("input capture %04X%s\n", m_counter, m_inhibit_cap ? " [inhibited]" : "");
			if (!m_inhibit_cap)
			{
				m_tsr |= 0x80;
				m_icr = m_counter;
				if (m_tcr & m_tsr & 0xe0) m_pending_interrupts |= M68HC05_INT_TIMER;
			}
		}
		m_tcap_state = bool(state);
		break;
	default:
		fatalerror("m68hc05[%s]: unknown input line %d", tag(), inputnum);
	}
}

u64 m68hc05_device::execute_clocks_to_cycles(u64 clocks) const
{
	return (clocks + 1) / 2;
}

u64 m68hc05_device::execute_cycles_to_clocks(u64 cycles) const
{
	return cycles * 2;
}


offs_t m68hc05_device::disasm_disassemble(
		std::ostream &stream,
		offs_t pc,
		const u8 *oprom,
		const u8 *opram,
		u32 options)
{
	return CPU_DISASSEMBLE_NAME(m68hc05)(this, stream, pc, oprom, opram, options);
}


void m68hc05_device::interrupt()
{
	if ((m_pending_interrupts & M68HC05_INT_MASK) && !(CC & IFLAG))
	{
		pushword(m_pc);
		pushbyte(m_x);
		pushbyte(m_a);
		pushbyte(m_cc);
		SEI;
		standard_irq_callback(0);

		if (m_pending_interrupts & M68HC05_INT_IRQ)
		{
			LOGINT("servicing external interrupt\n");
			m_irq_latch = 0;
			m_pending_interrupts &= ~M68HC05_INT_IRQ;
			rm16(M68HC05_VECTOR_IRQ, m_pc);
		}
		else if (m_pending_interrupts & M68HC05_INT_TIMER)
		{
			LOGINT("servicing timer interrupt\n");
			rm16(M68HC05_VECTOR_TIMER, m_pc);
		}
		else
		{
			fatalerror("m68hc05[%s]: unknown pending interrupt(s) %x", tag(), m_pending_interrupts);
		}
		m_icount -= 10;
		burn_cycles(10);
	}
}

bool m68hc05_device::test_il()
{
	return m_irq_line_state;
}

void m68hc05_device::burn_cycles(unsigned count)
{
	// calculate new timer values (fixed prescaler of four)
	unsigned const ps_opt(4);
	unsigned const ps_mask((1 << ps_opt) - 1);
	unsigned const increments((count + (m_prescaler & ps_mask)) >> ps_opt);
	u32 const new_counter(u32(m_counter) + increments);
	bool const timer_rollover((0x010000 > m_counter) && (0x010000 <= new_counter));
	bool const output_compare_match((m_ocr > m_counter) && (m_ocr <= new_counter));
	m_prescaler = (count + m_prescaler) & ps_mask;
	m_counter = u16(new_counter);
	if (timer_rollover)
	{
		LOGTIMER("timer rollover\n");
		m_tsr |= 0x20;
	}
	if (output_compare_match)
	{
		LOGTIMER("output compare match %s\n", m_inhibit_cmp ? " [inhibited]" : "");
		if (!m_inhibit_cmp)
		{
			m_tsr |= 0x40;
			m_tcmp_cb(tcr_olvl() ? 1 : 0);
		}
	}
	if (m_tcr & m_tsr & 0xe0) m_pending_interrupts |= M68HC05_INT_TIMER;

	// run programmable COP
	u32 const pcop_timeout(u32(1) << ((copcr_cm() << 1) + 15));
	if (copcr_pcope() && (pcop_timeout <= ((m_pcop_cnt & (pcop_timeout - 1)) + count)))
	{
		LOGCOP("PCOP reset\n");
		m_copcr |= 0x10;
		set_input_line(INPUT_LINE_RESET, PULSE_LINE);
	}
	m_pcop_cnt = (m_pcop_cnt + count) & ((u32(1) << 21) - 1);

	// run non-programmable COP
	m_ncop_cnt += count;
	if ((u32(1) << 17) <= m_ncop_cnt)
	{
		set_input_line(INPUT_LINE_RESET, PULSE_LINE);
		LOGCOP("NCOP reset\n");
	}
	m_ncop_cnt &= (u32(1) << 17) - 1;
}


void m68hc05_device::add_port_state(std::array<bool, PORT_COUNT> const &ddr)
{
	for (unsigned i = 0; PORT_COUNT > i; ++i)
	{
		if (m_port_bits[i])
			state_add(M68HC05_LATCHA + i, util::string_format("LATCH%c", 'A' + i).c_str(), m_port_latch[i]).mask(m_port_bits[i]);
	}
	for (unsigned i = 0; PORT_COUNT > i; ++i)
	{
		if (ddr[i] && m_port_bits[i])
			state_add(M68HC05_DDRA + i, util::string_format("DDR%c", 'A' + i).c_str(), m_port_ddr[i]).mask(m_port_bits[i]);
	}
}

void m68hc05_device::add_timer_state()
{
	state_add(M68HC05_TCR, "TCR", m_tcr).mask(0x7f);
	state_add(M68HC05_TSR, "TSR", m_tsr).mask(0xff);
	state_add(M68HC05_ICR, "ICR", m_icr).mask(0xffff);
	state_add(M68HC05_OCR, "OCR", m_ocr).mask(0xffff);
	state_add(M68HC05_PS, "PS", m_prescaler).mask(0x03);
	state_add(M68HC05_TR, "TR", m_counter).mask(0xffff);
}

void m68hc05_device::add_pcop_state()
{
	state_add(M68HC05_COPRST, "COPRST", m_coprst).mask(0xff);
	state_add(M68HC05_COPCR, "COPCR", m_copcr).mask(0x1f);
	state_add(M68HC05_PCOP, "PCOP", m_pcop_cnt).mask(0x001fffff);
}

void m68hc05_device::add_ncop_state()
{
	state_add(M68HC05_NCOPE, "NCOPE", m_ncope).mask(0x01);
	state_add(M68HC05_NCOP, "NCOP", m_ncop_cnt).mask(0x0001ffff);
}


u8 m68hc05_device::port_value(unsigned offset) const
{
	return (m_port_latch[offset] & m_port_ddr[offset]) | (m_port_input[offset] & ~m_port_ddr[offset]);
}

void m68hc05_device::update_port_irq()
{
	u8 state(0x00);
	for (unsigned i = 0; i < PORT_COUNT; ++i)
		state |= m_port_interrupt[i] & ~m_port_ddr[i] & ~m_port_input[i];

	if (bool(state) != m_port_irq_state)
	{
		LOGINT("I/O port IRQ state now %u%s\n",
				state ? 1 : 0, (!m_irq_line_state && state && !m_irq_latch) ? ", set IRQ latch" : "");
		m_port_irq_state = bool(state);
		if (!m_irq_line_state && state)
		{
			m_irq_latch = 1;
			m_pending_interrupts |= M68HC05_INT_IRQ;
		}
	}
}



/****************************************************************************
 * M68HC705 base device
 ****************************************************************************/

m68hc705_device::m68hc705_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock,
		device_type type,
		address_map_delegate internal_map)
	: m68hc05_device(mconfig, tag, owner, clock, type, internal_map)
{
}



/****************************************************************************
 * MC68HC05C4 device
 ****************************************************************************/

DEVICE_ADDRESS_MAP_START( c4_map, 8, m68hc05c4_device )
	ADDRESS_MAP_GLOBAL_MASK(0x1fff)
	ADDRESS_MAP_UNMAP_HIGH

	AM_RANGE(0x0000, 0x0003) AM_READWRITE(port_r, port_latch_w)
	AM_RANGE(0x0004, 0x0006) AM_READWRITE(port_ddr_r, port_ddr_w)
	// 0x0007-0x0009 unused
	// 0x000a SPCR
	// 0x000b SPSR
	// 0x000c SPDR
	// 0x000d BAUD
	// 0x000e SCCR1
	// 0x000f SCCR2
	// 0x0010 SCSR
	// 0x0011 SCDR
	AM_RANGE(0x0012, 0x0012) AM_READWRITE(tcr_r, tcr_w)
	AM_RANGE(0x0013, 0x0013) AM_READ(tsr_r)
	AM_RANGE(0x0014, 0x0015) AM_READ(icr_r)
	AM_RANGE(0x0016, 0x0017) AM_READWRITE(ocr_r, ocr_w)
	AM_RANGE(0x0018, 0x001b) AM_READ(timer_r)
	// 0x001c-0x001f unused
	AM_RANGE(0x0020, 0x004f) AM_ROM // user ROM
	AM_RANGE(0x0050, 0x00ff) AM_RAM // RAM/stack
	AM_RANGE(0x0100, 0x10ff) AM_ROM // user ROM
	// 0x1100-0x1eff unused
	AM_RANGE(0x1f00, 0x1fef) AM_ROM // self-check
	// 0x1ff0-0x1ff3 unused
	AM_RANGE(0x1ff4, 0x1fff) AM_ROM // user vectors
ADDRESS_MAP_END


m68hc05c4_device::m68hc05c4_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: m68hc05_device(
			mconfig,
			tag,
			owner,
			clock,
			M68HC05C4,
			address_map_delegate(FUNC(m68hc05c4_device::c4_map), this))
{
	set_port_bits(std::array<u8, PORT_COUNT>{{ 0xff, 0xff, 0xff, 0xbf }});
}



void m68hc05c4_device::device_start()
{
	m68hc05_device::device_start();

	add_port_state(std::array<bool, PORT_COUNT>{{ true, true, true, false }});
	add_timer_state();
}


offs_t m68hc05c4_device::disasm_disassemble(
		std::ostream &stream,
		offs_t pc,
		const u8 *oprom,
		const u8 *opram,
		u32 options)
{
	return CPU_DISASSEMBLE_NAME(m68hc05)(this, stream, pc, oprom, opram, options, m68hc05c4_syms);
}



/****************************************************************************
 * MC68HC05C8 device
 ****************************************************************************/

DEVICE_ADDRESS_MAP_START( c8_map, 8, m68hc05c8_device )
	ADDRESS_MAP_GLOBAL_MASK(0x1fff)
	ADDRESS_MAP_UNMAP_HIGH

	AM_RANGE(0x0000, 0x0003) AM_READWRITE(port_r, port_latch_w)
	AM_RANGE(0x0004, 0x0006) AM_READWRITE(port_ddr_r, port_ddr_w)
	// 0x0007-0x0009 unused
	// 0x000a SPCR
	// 0x000b SPSR
	// 0x000c SPDR
	// 0x000d BAUD
	// 0x000e SCCR1
	// 0x000f SCCR2
	// 0x0010 SCSR
	// 0x0011 SCDR
	AM_RANGE(0x0012, 0x0012) AM_READWRITE(tcr_r, tcr_w)
	AM_RANGE(0x0013, 0x0013) AM_READ(tsr_r)
	AM_RANGE(0x0014, 0x0015) AM_READ(icr_r)
	AM_RANGE(0x0016, 0x0017) AM_READWRITE(ocr_r, ocr_w)
	AM_RANGE(0x0018, 0x001b) AM_READ(timer_r)
	// 0x001c-0x001f unused
	AM_RANGE(0x0020, 0x004f) AM_ROM // user ROM
	AM_RANGE(0x0050, 0x00ff) AM_RAM // RAM/stack
	AM_RANGE(0x0100, 0x1eff) AM_ROM // user ROM
	AM_RANGE(0x1f00, 0x1fef) AM_ROM // self-check
	// 0x1ff0-0x1ff3 unused
	AM_RANGE(0x1ff4, 0x1fff) AM_ROM // user vectors
ADDRESS_MAP_END


m68hc05c8_device::m68hc05c8_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: m68hc05_device(
			mconfig,
			tag,
			owner,
			clock,
			M68HC05C8,
			address_map_delegate(FUNC(m68hc05c8_device::c8_map), this))
{
	set_port_bits(std::array<u8, PORT_COUNT>{{ 0xff, 0xff, 0xff, 0xbf }});
}


void m68hc05c8_device::device_start()
{
	m68hc05_device::device_start();

	add_port_state(std::array<bool, PORT_COUNT>{{ true, true, true, false }});
	add_timer_state();
}


offs_t m68hc05c8_device::disasm_disassemble(
		std::ostream &stream,
		offs_t pc,
		const u8 *oprom,
		const u8 *opram,
		u32 options)
{
	// same I/O registers as MC68HC05C4
	return CPU_DISASSEMBLE_NAME(m68hc05)(this, stream, pc, oprom, opram, options, m68hc05c4_syms);
}



/****************************************************************************
 * MC68HC705C8A device
 ****************************************************************************/

DEVICE_ADDRESS_MAP_START( c8a_map, 8, m68hc705c8a_device )
	ADDRESS_MAP_GLOBAL_MASK(0x1fff)
	ADDRESS_MAP_UNMAP_HIGH

	AM_RANGE(0x0000, 0x0003) AM_READWRITE(port_r, port_latch_w)
	AM_RANGE(0x0004, 0x0006) AM_READWRITE(port_ddr_r, port_ddr_w)
	// 0x0007-0x0009 unused
	// 0x000a SPCR
	// 0x000b SPSR
	// 0x000c SPDR
	// 0x000d BAUD
	// 0x000e SCCR1
	// 0x000f SCCR2
	// 0x0010 SCSR
	// 0x0011 SCDR
	AM_RANGE(0x0012, 0x0012) AM_READWRITE(tcr_r, tcr_w)
	AM_RANGE(0x0013, 0x0013) AM_READ(tsr_r)
	AM_RANGE(0x0014, 0x0015) AM_READ(icr_r)
	AM_RANGE(0x0016, 0x0017) AM_READWRITE(ocr_r, ocr_w)
	AM_RANGE(0x0018, 0x001b) AM_READ(timer_r)
	// 0x001c PROG
	AM_RANGE(0x001d, 0x001d) AM_WRITE(coprst_w)
	AM_RANGE(0x001e, 0x001e) AM_READWRITE(copcr_r, copcr_w)
	// 0x001f unused
	AM_RANGE(0x0020, 0x004f) AM_ROM                                 // user PROM FIXME: banked with RAM
	AM_RANGE(0x0050, 0x00ff) AM_RAM                                 // RAM/stack
	AM_RANGE(0x0100, 0x015f) AM_ROM                                 // user PROM FIXME: banked with RAM
	AM_RANGE(0x0160, 0x1eff) AM_ROM                                 // user PROM
	AM_RANGE(0x1f00, 0x1fde) AM_ROM AM_REGION("bootstrap", 0x0000)  // bootloader
	// 0x1fdf option register FIXME: controls banking
	AM_RANGE(0x1fe0, 0x1fef) AM_ROM AM_REGION("bootstrap", 0x00e0)  // boot ROM vectors
	AM_RANGE(0x1ff0, 0x1ff0) AM_WRITE(copr_w)
	AM_RANGE(0x1ff0, 0x1fff) AM_ROM                                 // user vectors
ADDRESS_MAP_END


m68hc705c8a_device::m68hc705c8a_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: m68hc705_device(
			mconfig,
			tag,
			owner,
			clock,
			M68HC705C8A,
			address_map_delegate(FUNC(m68hc705c8a_device::c8a_map), this))
{
	set_port_bits(std::array<u8, PORT_COUNT>{{ 0xff, 0xff, 0xff, 0xbf }});
}


tiny_rom_entry const *m68hc705c8a_device::device_rom_region() const
{
	return ROM_NAME(m68hc705c8a);
}


void m68hc705c8a_device::device_start()
{
	m68hc705_device::device_start();

	add_port_state(std::array<bool, PORT_COUNT>{{ true, true, true, false }});
	add_timer_state();
	add_pcop_state();
	add_ncop_state();
}

void m68hc705c8a_device::device_reset()
{
	m68hc705_device::device_reset();

	// latch MOR registers on reset
	set_port_interrupt(std::array<u8, PORT_COUNT>{{ 0x00, u8(rdmem(0xfff0)), 0x00, 0x00 }});
	set_ncope(BIT(rdmem(0xfff1), 0));
}


offs_t m68hc705c8a_device::disasm_disassemble(
		std::ostream &stream,
		offs_t pc,
		const u8 *oprom,
		const u8 *opram,
		u32 options)
{
	return CPU_DISASSEMBLE_NAME(m68hc05)(this, stream, pc, oprom, opram, options, m68hc705c8a_syms);
}
