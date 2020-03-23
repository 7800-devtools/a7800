// license:BSD-3-Clause
// copyright-holders:hap
/*

  NEC uCOM-4 MCU family cores

  References:
  - 1981 NEC Microcomputers Catalog (later editions may have errors!)
  - Supplement to uCOM-43 Single Chip Microcomputer Users' Manual
  I've also looked at asterick's JavaScript D553 emulator for verification, with permission.

  TODO:
  - what happens with uCOM-43 opcodes on an uCOM-44/45 MCU?
  - what's the data after the ROM data for? (eg. 2000-2047, official ROM size is 2000)
  - is DPh internally 3-bit or 4-bit? (currently assume 4-bit, it could have effect
    on specific uCOM-43 exchange opcodes)
  - RAM access from 0x50-0x7f on data_96x4
  - invalid port accesses via DPl
  - documentation is conflicting if IRQ is level or edge triggered

*/

#include "emu.h"
#include "ucom4.h"
#include "debugger.h"


// uCOM-43 products: 2000x8 ROM, 96x4 RAM, supports full instruction set
DEFINE_DEVICE_TYPE(NEC_D546,  upd546_cpu_device,  "upd546",  "NEC uPD546") // 42-pin PMOS, 35 pins for I/O
DEFINE_DEVICE_TYPE(NEC_D553,  upd553_cpu_device,  "upd553",  "NEC uPD553") // 42-pin PMOS, 35 pins for I/O, high voltage
DEFINE_DEVICE_TYPE(NEC_D557L, upd557l_cpu_device, "upd557l", "NEC uPD557L") // 28-pin PMOS, 21 pins for I/O
DEFINE_DEVICE_TYPE(NEC_D650,  upd650_cpu_device,  "upd650",  "NEC uPD650") // 42-pin CMOS, 35 pins for I/O

// uCOM-44 products: 1000x8 ROM, 64x4 RAM, 1-level stack, does not support external interrupt
DEFINE_DEVICE_TYPE(NEC_D552,  upd552_cpu_device,  "upd552",  "NEC uPD552") // 42-pin PMOS, 35 pins for I/O, high voltage

// uCOM-45 products: 1000x8 or 640x8 ROM, 32x4 RAM, 1-level stack
//..


// internal memory maps
static ADDRESS_MAP_START(program_1k, AS_PROGRAM, 8, ucom4_cpu_device)
	AM_RANGE(0x0000, 0x03ff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START(program_2k, AS_PROGRAM, 8, ucom4_cpu_device)
	AM_RANGE(0x0000, 0x07ff) AM_ROM
ADDRESS_MAP_END


static ADDRESS_MAP_START(data_64x4, AS_DATA, 8, ucom4_cpu_device)
	AM_RANGE(0x00, 0x3f) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START(data_96x4, AS_DATA, 8, ucom4_cpu_device)
	AM_RANGE(0x00, 0x3f) AM_RAM
	AM_RANGE(0x40, 0x4f) AM_RAM
	AM_RANGE(0x70, 0x7f) AM_RAM
ADDRESS_MAP_END


// device definitions
ucom4_cpu_device::ucom4_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, int family, int stack_levels, int prgwidth, address_map_constructor program, int datawidth, address_map_constructor data)
	: cpu_device(mconfig, type, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_BIG, 8, prgwidth, 0, program)
	, m_data_config("data", ENDIANNESS_BIG, 8, datawidth, 0, data)
	, m_prgwidth(prgwidth)
	, m_datawidth(datawidth)
	, m_family(family)
	, m_stack_levels(stack_levels)
	, m_read_a(*this)
	, m_read_b(*this)
	, m_read_c(*this)
	, m_read_d(*this)
	, m_write_c(*this)
	, m_write_d(*this)
	, m_write_e(*this)
	, m_write_f(*this)
	, m_write_g(*this)
	, m_write_h(*this)
	, m_write_i(*this)
{
}

upd546_cpu_device::upd546_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: ucom4_cpu_device(mconfig, NEC_D546, tag, owner, clock, NEC_UCOM43, 3 /* stack levels */, 11 /* prg width */, ADDRESS_MAP_NAME(program_2k), 7 /* data width */, ADDRESS_MAP_NAME(data_96x4))
{
}

upd553_cpu_device::upd553_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: ucom4_cpu_device(mconfig, NEC_D553, tag, owner, clock, NEC_UCOM43, 3, 11, ADDRESS_MAP_NAME(program_2k), 7, ADDRESS_MAP_NAME(data_96x4))
{
}

upd557l_cpu_device::upd557l_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: ucom4_cpu_device(mconfig, NEC_D557L, tag, owner, clock, NEC_UCOM43, 3, 11, ADDRESS_MAP_NAME(program_2k), 7, ADDRESS_MAP_NAME(data_96x4))
{
}

upd650_cpu_device::upd650_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: ucom4_cpu_device(mconfig, NEC_D650, tag, owner, clock, NEC_UCOM43, 3, 11, ADDRESS_MAP_NAME(program_2k), 7, ADDRESS_MAP_NAME(data_96x4))
{
}

upd552_cpu_device::upd552_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: ucom4_cpu_device(mconfig, NEC_D552, tag, owner, clock, NEC_UCOM44, 1, 10, ADDRESS_MAP_NAME(program_1k), 6, ADDRESS_MAP_NAME(data_64x4))
{
}

device_memory_interface::space_config_vector ucom4_cpu_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config),
		std::make_pair(AS_DATA,    &m_data_config)
	};
}


// disasm
void ucom4_cpu_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		// obviously not from a single flags register
		case STATE_GENFLAGS:
			str = string_format("%c%c%c%c%c",
				m_inte_f    ? 'E':'e',
				m_int_f     ? 'I':'i',
				m_timer_f   ? 'T':'t',
				m_carry_s_f ? 'S':'s',
				m_carry_f   ? 'C':'c'
			);
			break;

		default: break;
	}
}

offs_t ucom4_cpu_device::disasm_disassemble(std::ostream &stream, offs_t pc, const u8 *oprom, const u8 *opram, u32 options)
{
	extern CPU_DISASSEMBLE(ucom4);
	return CPU_DISASSEMBLE_NAME(ucom4)(this, stream, pc, oprom, opram, options);
}



//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

enum
{
	UCOM4_PC=1, UCOM4_DPL, UCOM4_DPH,
	UCOM4_ACC
};

void ucom4_cpu_device::device_start()
{
	assert(NEC_UCOM4_PORTA == 0);

	m_program = &space(AS_PROGRAM);
	m_data = &space(AS_DATA);
	m_prgmask = (1 << m_prgwidth) - 1;
	m_datamask = (1 << m_datawidth) - 1;
	m_dph_mask = m_datamask >> 4;

	m_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(ucom4_cpu_device::simple_timer_cb), this));

	// resolve callbacks
	m_read_a.resolve_safe(0);
	m_read_b.resolve_safe(0);
	m_read_c.resolve_safe(0);
	m_read_d.resolve_safe(0);

	m_write_c.resolve_safe();
	m_write_d.resolve_safe();
	m_write_e.resolve_safe();
	m_write_f.resolve_safe();
	m_write_g.resolve_safe();
	m_write_h.resolve_safe();
	m_write_i.resolve_safe();

	// zerofill
	memset(m_stack, 0, sizeof(m_stack));
	memset(m_port_out, 0, sizeof(m_port_out));
	m_op = 0;
	m_prev_op = 0;
	m_skip = false;
	m_pc = 0;
	m_prev_pc = 0;
	m_acc = 0;
	m_dpl = 0;
	m_dph = 0;
	m_carry_f = 0;
	m_carry_s_f = 0;
	m_timer_f = 0;
	m_int_f = 0;
	m_inte_f = 0;
	m_int_line = CLEAR_LINE;

	// register for savestates
	save_item(NAME(m_stack));
	save_item(NAME(m_port_out));
	save_item(NAME(m_op));
	save_item(NAME(m_prev_op));
	save_item(NAME(m_skip));
	save_item(NAME(m_pc));
	save_item(NAME(m_prev_pc));
	save_item(NAME(m_acc));
	save_item(NAME(m_dpl));
	save_item(NAME(m_dph));
	save_item(NAME(m_carry_f));
	save_item(NAME(m_carry_s_f));
	save_item(NAME(m_timer_f));
	save_item(NAME(m_int_f));
	save_item(NAME(m_int_line));

	// register state for debugger
	state_add(UCOM4_PC, "PC",  m_pc).formatstr("%04X");
	state_add(UCOM4_DPL, "DPL", m_dpl).formatstr("%01X");
	state_add(UCOM4_DPH, "DPH", m_dph).formatstr("%01X");
	state_add(UCOM4_ACC, "ACC", m_acc).formatstr("%01X");

	state_add(STATE_GENPC, "GENPC", m_pc).formatstr("%04X").noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).formatstr("%04X").noshow();
	state_add(STATE_GENFLAGS, "GENFLAGS", m_carry_f).formatstr("%5s").noshow(); // dummy

	m_icountptr = &m_icount;
}



//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void ucom4_cpu_device::device_reset()
{
	m_pc = 0;
	m_op = 0;
	m_skip = false;

	m_timer->adjust(attotime::never);

	// clear interrupt
	m_int_line = CLEAR_LINE;
	m_int_f = 0;
	m_inte_f = (m_family == NEC_UCOM43) ? 0 : 1;

	// clear i/o
	for (int i = NEC_UCOM4_PORTC; i <= NEC_UCOM4_PORTI; i++)
		output_w(i, 0);
}



//-------------------------------------------------
//  i/o handling
//-------------------------------------------------

// default:
// A,B are inputs, C,D are input/output, E,F,G,H,I are output

u8 ucom4_cpu_device::input_r(int index)
{
	index &= 0xf;
	u8 inp = 0;

	switch (index)
	{
		case NEC_UCOM4_PORTA: inp = m_read_a(index, 0xff); break;
		case NEC_UCOM4_PORTB: inp = m_read_b(index, 0xff); break;
		case NEC_UCOM4_PORTC: inp = m_read_c(index, 0xff) | m_port_out[index]; break;
		case NEC_UCOM4_PORTD: inp = m_read_d(index, 0xff) | m_port_out[index]; break;

		default:
			logerror("%s read from unknown port %c at $%03X\n", tag(), 'A' + index, m_prev_pc);
			break;
	}

	return inp & 0xf;
}

void ucom4_cpu_device::output_w(int index, u8 data)
{
	index &= 0xf;
	data &= 0xf;

	switch (index)
	{
		case NEC_UCOM4_PORTC: m_write_c(index, data, 0xff); break;
		case NEC_UCOM4_PORTD: m_write_d(index, data, 0xff); break;
		case NEC_UCOM4_PORTE: m_write_e(index, data, 0xff); break;
		case NEC_UCOM4_PORTF: m_write_f(index, data, 0xff); break;
		case NEC_UCOM4_PORTG: m_write_g(index, data, 0xff); break;
		case NEC_UCOM4_PORTH: m_write_h(index, data, 0xff); break;
		case NEC_UCOM4_PORTI: m_write_i(index, data & 7, 0xff); break;

		default:
			logerror("%s write to unknown port %c = $%X at $%03X\n", tag(), 'A' + index, data, m_prev_pc);
			break;
	}

	m_port_out[index] = data;
}

// uPD557L:
// ports B,H,I are stripped, port G is reduced to 1 pin

u8 upd557l_cpu_device::input_r(int index)
{
	index &= 0xf;

	if (index == NEC_UCOM4_PORTB)
		logerror("%s read from unknown port %c at $%03X\n", tag(), 'A' + index, m_prev_pc);
	else
		return ucom4_cpu_device::input_r(index);

	return 0;
}

void upd557l_cpu_device::output_w(int index, u8 data)
{
	index &= 0xf;
	data &= 0xf;

	if (index == NEC_UCOM4_PORTH || index == NEC_UCOM4_PORTI)
		logerror("%s write to unknown port %c = $%X at $%03X\n", tag(), 'A' + index, data, m_prev_pc);
	else
	{
		// only G0 for port G
		if (index == NEC_UCOM4_PORTG)
			data &= 1;

		ucom4_cpu_device::output_w(index, data);
	}
}



//-------------------------------------------------
//  interrupt
//-------------------------------------------------

void ucom4_cpu_device::execute_set_input(int line, int state)
{
	switch (line)
	{
		case 0:
			// edge triggered
			if (m_int_line == CLEAR_LINE && state)
				m_int_f = 1;
			m_int_line = state;

			break;

		default:
			break;
	}
}

void ucom4_cpu_device::do_interrupt()
{
	m_icount--;
	push_stack();
	m_pc = 0xf << 2;
	m_int_f = 0;
	m_inte_f = (m_family == NEC_UCOM43) ? 0 : 1;

	standard_irq_callback(0);
}



//-------------------------------------------------
//  execute
//-------------------------------------------------

inline void ucom4_cpu_device::increment_pc()
{
	// upper bits (field register) don't auto-increment
	m_pc = (m_pc & ~0xff) | ((m_pc + 1) & 0xff);
}

inline void ucom4_cpu_device::fetch_arg()
{
	// 2-byte opcodes: STM/LDI/CLI/CI, JMP/CAL, OCD
	if ((m_op & 0xfc) == 0x14 || (m_op & 0xf0) == 0xa0 || m_op == 0x1e)
	{
		m_icount--;
		m_arg = m_program->read_byte(m_pc);
		increment_pc();
	}
}

void ucom4_cpu_device::execute_run()
{
	while (m_icount > 0)
	{
		// handle interrupt, but not during LI($9x) or EI($31) or while skipping
		if (m_int_f && m_inte_f && (m_op & 0xf0) != 0x90 && m_op != 0x31 && !m_skip)
		{
			do_interrupt();
			if (m_icount <= 0)
				break;
		}

		// remember previous state
		m_prev_op = m_op;
		m_prev_pc = m_pc;

		// fetch next opcode
		debugger_instruction_hook(this, m_pc);
		m_icount--;
		m_op = m_program->read_byte(m_pc);
		m_bitmask = 1 << (m_op & 0x03);
		increment_pc();
		fetch_arg();

		if (m_skip)
		{
			m_skip = false;
			m_op = 0; // nop
		}

		// handle opcode
		switch (m_op & 0xf0)
		{
			case 0x80: op_ldz(); break;
			case 0x90: op_li(); break;
			case 0xa0: op_jmpcal(); break;
			case 0xb0: op_czp(); break;

			case 0xc0: case 0xd0: case 0xe0: case 0xf0: op_jcp(); break;

			default:
				switch (m_op)
				{
			case 0x00: op_nop(); break;
			case 0x01: op_di(); break;
			case 0x02: op_s(); break;
			case 0x03: op_tit(); break;
			case 0x04: op_tc(); break;
			case 0x05: op_ttm(); break;
			case 0x06: op_daa(); break;
			case 0x07: op_tal(); break;
			case 0x08: op_ad(); break;
			case 0x09: op_ads(); break;
			case 0x0a: op_das(); break;
			case 0x0b: op_clc(); break;
			case 0x0c: op_cm(); break;
			case 0x0d: op_inc(); break;
			case 0x0e: op_op(); break;
			case 0x0f: op_dec(); break;
			case 0x10: op_cma(); break;
			case 0x11: op_cia(); break;
			case 0x12: op_tla(); break;
			case 0x13: op_ded(); break;
			case 0x14: op_stm(); break;
			case 0x15: op_ldi(); break;
			case 0x16: op_cli(); break;
			case 0x17: op_ci(); break;
			case 0x18: op_exl(); break;
			case 0x19: op_adc(); break;
			case 0x1a: op_xc(); break;
			case 0x1b: op_stc(); break;
			case 0x1c: op_illegal(); break;
			case 0x1d: op_inm(); break;
			case 0x1e: op_ocd(); break;
			case 0x1f: op_dem(); break;

			case 0x30: op_rar(); break;
			case 0x31: op_ei(); break;
			case 0x32: op_ip(); break;
			case 0x33: op_ind(); break;

			case 0x40: op_ia(); break;
			case 0x41: op_jpa(); break;
			case 0x42: op_taz(); break;
			case 0x43: op_taw(); break;
			case 0x44: op_oe(); break;
			case 0x45: op_illegal(); break;
			case 0x46: op_tly(); break;
			case 0x47: op_thx(); break;
			case 0x48: op_rt(); break;
			case 0x49: op_rts(); break;
			case 0x4a: op_xaz(); break;
			case 0x4b: op_xaw(); break;
			case 0x4c: op_xls(); break;
			case 0x4d: op_xhr(); break;
			case 0x4e: op_xly(); break;
			case 0x4f: op_xhx(); break;

			default:
				switch (m_op & 0xfc)
				{
			case 0x20: op_fbf(); break;
			case 0x24: op_tab(); break;
			case 0x28: op_xm(); break;
			case 0x2c: op_xmd(); break;

			case 0x34: op_cmb(); break;
			case 0x38: op_lm(); break;
			case 0x3c: op_xmi(); break;

			case 0x50: op_tpb(); break;
			case 0x54: op_tpa(); break;
			case 0x58: op_tmb(); break;
			case 0x5c: op_fbt(); break;
			case 0x60: op_rpb(); break;
			case 0x64: op_reb(); break;
			case 0x68: op_rmb(); break;
			case 0x6c: op_rfb(); break;
			case 0x70: op_spb(); break;
			case 0x74: op_seb(); break;
			case 0x78: op_smb(); break;
			case 0x7c: op_sfb(); break;
				}
				break; // 0xfc

				}
				break; // 0xff

		} // big switch
	}
}
