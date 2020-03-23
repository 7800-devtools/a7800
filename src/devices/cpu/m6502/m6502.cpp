// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    m6502.c

    Mostek 6502, original NMOS variant

***************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "m6502.h"

DEFINE_DEVICE_TYPE(M6502, m6502_device, "m6502", "M6502")

m6502_device::m6502_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	m6502_device(mconfig, M6502, tag, owner, clock)
{
}

m6502_device::m6502_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	cpu_device(mconfig, type, tag, owner, clock),
	sync_w(*this),
	program_config("program", ENDIANNESS_LITTLE, 8, 16),
	sprogram_config("decrypted_opcodes", ENDIANNESS_LITTLE, 8, 16), PPC(0), NPC(0), PC(0), SP(0), TMP(0), TMP2(0), A(0), X(0), Y(0), P(0), IR(0), inst_state_base(0), mintf(nullptr),
	inst_state(0), inst_substate(0), icount(0), nmi_state(false), irq_state(false), apu_irq_state(false), v_state(false), irq_taken(false), sync(false), inhibit_interrupts(false)
{
	direct_disabled = false;
}

void m6502_device::device_start()
{
	if(direct_disabled)
		mintf = new mi_default_nd;
	else
		mintf = new mi_default_normal;

	init();
}

void m6502_device::init()
{
	mintf->program  = &space(AS_PROGRAM);
	mintf->sprogram = has_space(AS_OPCODES) ? &space(AS_OPCODES) : mintf->program;

	mintf->direct  = &mintf->program->direct();
	mintf->sdirect = &mintf->sprogram->direct();

	sync_w.resolve_safe();

	state_add(STATE_GENPC,     "GENPC",     NPC).noshow();
	state_add(STATE_GENPCBASE, "CURPC",     PPC).noshow();
	state_add(STATE_GENSP,     "GENSP",     SP).noshow();
	state_add(STATE_GENFLAGS,  "GENFLAGS",  P).callimport().formatstr("%6s").noshow();
	state_add(M6502_PC,        "PC",        NPC).callimport();
	state_add(M6502_A,         "A",         A);
	state_add(M6502_X,         "X",         X);
	state_add(M6502_Y,         "Y",         Y);
	state_add(M6502_P,         "P",         P).callimport();
	state_add(M6502_S,         "SP",        SP);
	state_add(M6502_IR,        "IR",        IR);

	save_item(NAME(PC));
	save_item(NAME(NPC));
	save_item(NAME(A));
	save_item(NAME(X));
	save_item(NAME(Y));
	save_item(NAME(P));
	save_item(NAME(SP));
	save_item(NAME(TMP));
	save_item(NAME(TMP2));
	save_item(NAME(IR));
	save_item(NAME(nmi_state));
	save_item(NAME(irq_state));
	save_item(NAME(apu_irq_state));
	save_item(NAME(v_state));
	save_item(NAME(inst_state));
	save_item(NAME(inst_substate));
	save_item(NAME(inst_state_base));
	save_item(NAME(irq_taken));
	save_item(NAME(inhibit_interrupts));

	m_icountptr = &icount;

	PC = 0x0000;
	NPC = 0x0000;
	A = 0x00;
	X = 0x80;
	Y = 0x00;
	P = 0x36;
	SP = 0x01bd;
	TMP = 0x0000;
	TMP2 = 0x00;
	IR = 0x00;
	nmi_state = false;
	irq_state = false;
	apu_irq_state = false;
	irq_taken = false;
	v_state = false;
	inst_state = STATE_RESET;
	inst_substate = 0;
	inst_state_base = 0;
	sync = false;
	inhibit_interrupts = false;
}

void m6502_device::device_reset()
{
	inst_state = STATE_RESET;
	inst_substate = 0;
	inst_state_base = 0;
	nmi_state = false;
	irq_state = false;
	apu_irq_state = false;
	irq_taken = false;
	v_state = false;
	sync = false;
	sync_w(CLEAR_LINE);
	inhibit_interrupts = false;
}


uint32_t m6502_device::execute_min_cycles() const
{
	return 1;
}

uint32_t m6502_device::execute_max_cycles() const
{
	return 10;
}

uint32_t m6502_device::execute_input_lines() const
{
	return NMI_LINE+1;
}

void m6502_device::do_adc_d(uint8_t val)
{
	uint8_t c = P & F_C ? 1 : 0;
	P &= ~(F_N|F_V|F_Z|F_C);
	uint8_t al = (A & 15) + (val & 15) + c;
	if(al > 9)
		al += 6;
	uint8_t ah = (A >> 4) + (val >> 4) + (al > 15);
	if(!uint8_t(A + val + c))
		P |= F_Z;
	else if(ah & 8)
		P |= F_N;
	if(~(A^val) & (A^(ah << 4)) & 0x80)
		P |= F_V;
	if(ah > 9)
		ah += 6;
	if(ah > 15)
		P |= F_C;
	A = (ah << 4) | (al & 15);
}

void m6502_device::do_adc_nd(uint8_t val)
{
	uint16_t sum;
	sum = A + val + (P & F_C ? 1 : 0);
	P &= ~(F_N|F_V|F_Z|F_C);
	if(!uint8_t(sum))
		P |= F_Z;
	else if(int8_t(sum) < 0)
		P |= F_N;
	if(~(A^val) & (A^sum) & 0x80)
		P |= F_V;
	if(sum & 0xff00)
		P |= F_C;
	A = sum;
}

void m6502_device::do_adc(uint8_t val)
{
	if(P & F_D)
		do_adc_d(val);
	else
		do_adc_nd(val);
}

void m6502_device::do_arr_nd()
{
	bool c = P & F_C;
	P &= ~(F_N|F_Z|F_C|F_V);
	A >>= 1;
	if(c)
		A |= 0x80;
	if(!A)
		P |= F_Z;
	else if(int8_t(A)<0)
		P |= F_N;
	if(A & 0x40)
		P |= F_V|F_C;
	if(A & 0x20)
		P ^= F_V;
}

void m6502_device::do_arr_d()
{
	// The adc/ror interaction gives an extremely weird result
	bool c = P & F_C;
	P &= ~(F_N|F_Z|F_C|F_V);
	uint8_t a = A >> 1;
	if(c)
		a |= 0x80;
	if(!a)
		P |= F_Z;
	else if(int8_t(a) < 0)
		P |= F_N;
	if((a ^ A) & 0x40)
		P |= F_V;

	if((A & 0x0f) >= 0x05)
		a = ((a + 6) & 0x0f) | (a & 0xf0);

	if((A & 0xf0) >= 0x50) {
		a += 0x60;
		P |= F_C;
	}
	A = a;
}

void m6502_device::do_arr()
{
	if(P & F_D)
		do_arr_d();
	else
		do_arr_nd();
}

void m6502_device::do_cmp(uint8_t val1, uint8_t val2)
{
	P &= ~(F_N|F_Z|F_C);
	uint16_t r = val1-val2;
	if(!r)
		P |= F_Z;
	else if(int8_t(r) < 0)
		P |= F_N;
	if(!(r & 0xff00))
		P |= F_C;
}

void m6502_device::do_sbc_d(uint8_t val)
{
	uint8_t c = P & F_C ? 0 : 1;
	P &= ~(F_N|F_V|F_Z|F_C);
	uint16_t diff = A - val - c;
	uint8_t al = (A & 15) - (val & 15) - c;
	if(int8_t(al) < 0)
		al -= 6;
	uint8_t ah = (A >> 4) - (val >> 4) - (int8_t(al) < 0);
	if(!uint8_t(diff))
		P |= F_Z;
	else if(diff & 0x80)
		P |= F_N;
	if((A^val) & (A^diff) & 0x80)
		P |= F_V;
	if(!(diff & 0xff00))
		P |= F_C;
	if(int8_t(ah) < 0)
		ah -= 6;
	A = (ah << 4) | (al & 15);
}

void m6502_device::do_sbc_nd(uint8_t val)
{
	uint16_t diff = A - val - (P & F_C ? 0 : 1);
	P &= ~(F_N|F_V|F_Z|F_C);
	if(!uint8_t(diff))
		P |= F_Z;
	else if(int8_t(diff) < 0)
		P |= F_N;
	if((A^val) & (A^diff) & 0x80)
		P |= F_V;
	if(!(diff & 0xff00))
		P |= F_C;
	A = diff;
}

void m6502_device::do_sbc(uint8_t val)
{
	if(P & F_D)
		do_sbc_d(val);
	else
		do_sbc_nd(val);
}

void m6502_device::do_bit(uint8_t val)
{
	P &= ~(F_N|F_Z|F_V);
	uint8_t r = A & val;
	if(!r)
		P |= F_Z;
	if(val & 0x80)
		P |= F_N;
	if(val & 0x40)
		P |= F_V;
}

uint8_t m6502_device::do_asl(uint8_t v)
{
	P &= ~(F_N|F_Z|F_C);
	uint8_t r = v<<1;
	if(!r)
		P |= F_Z;
	else if(int8_t(r) < 0)
		P |= F_N;
	if(v & 0x80)
		P |= F_C;
	return r;
}

uint8_t m6502_device::do_lsr(uint8_t v)
{
	P &= ~(F_N|F_Z|F_C);
	if(v & 1)
		P |= F_C;
	v >>= 1;
	if(!v)
		P |= F_Z;
	return v;
}

uint8_t m6502_device::do_ror(uint8_t v)
{
	bool c = P & F_C;
	P &= ~(F_N|F_Z|F_C);
	if(v & 1)
		P |= F_C;
	v >>= 1;
	if(c)
		v |= 0x80;
	if(!v)
		P |= F_Z;
	else if(int8_t(v)<0)
		P |= F_N;
	return v;
}

uint8_t m6502_device::do_rol(uint8_t v)
{
	bool c = P & F_C;
	P &= ~(F_N|F_Z|F_C);
	if(v & 0x80)
		P |= F_C;
	v <<= 1;
	if(c)
		v |= 0x01;
	if(!v)
		P |= F_Z;
	else if(int8_t(v)<0)
		P |= F_N;
	return v;
}

uint8_t m6502_device::do_asr(uint8_t v)
{
	P &= ~(F_N|F_Z|F_C);
	if(v & 1)
		P |= F_C;
	v >>= 1;
	if(!v)
		P |= F_Z;
	else if(v & 0x40) {
		P |= F_N;
		v |= 0x80;
	}
	return v;
}

void m6502_device::execute_run()
{
	if(inst_substate)
		do_exec_partial();

	while(icount > 0) {
		if(inst_state < 0xff00) {
			PPC = NPC;
			inst_state = IR | inst_state_base;
			if(machine().debug_flags & DEBUG_FLAG_ENABLED)
				debugger_instruction_hook(this, NPC);
		}
		do_exec_full();
	}
}

void m6502_device::execute_set_input(int inputnum, int state)
{
	switch(inputnum) {
	case IRQ_LINE: irq_state = state == ASSERT_LINE; break;
	case APU_IRQ_LINE: apu_irq_state = state == ASSERT_LINE; break;
	case NMI_LINE: nmi_state = nmi_state || (state == ASSERT_LINE); break;
	case V_LINE:
		if(!v_state && state == ASSERT_LINE)
			P |= F_V;
		v_state = state == ASSERT_LINE;
		break;
	}
}


device_memory_interface::space_config_vector m6502_device::memory_space_config() const
{
	if(has_configured_map(AS_OPCODES))
		return space_config_vector {
			std::make_pair(AS_PROGRAM, &program_config),
			std::make_pair(AS_OPCODES, &sprogram_config)
		};
	else
		return space_config_vector {
			std::make_pair(AS_PROGRAM, &program_config)
		};
}


void m6502_device::state_import(const device_state_entry &entry)
{
	switch(entry.index()) {
	case STATE_GENFLAGS:
	case M6502_P:
		P = P | (F_B|F_E);
		break;
	case M6502_PC:
		PC = NPC;
		irq_taken = false;
		prefetch();
		PPC = NPC;
		inst_state = IR | inst_state_base;
		break;
	}
}

void m6502_device::state_export(const device_state_entry &entry)
{
}

void m6502_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch(entry.index()) {
	case STATE_GENFLAGS:
	case M6502_P:
		str = string_format("%c%c%c%c%c%c",
						P & F_N ? 'N' : '.',
						P & F_V ? 'V' : '.',
						P & F_D ? 'D' : '.',
						P & F_I ? 'I' : '.',
						P & F_Z ? 'Z' : '.',
						P & F_C ? 'C' : '.');
		break;
	}
}


uint32_t m6502_device::disasm_min_opcode_bytes() const
{
	return 1;
}

uint32_t m6502_device::disasm_max_opcode_bytes() const
{
	return 4;
}

offs_t m6502_device::disassemble_generic(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options, const disasm_entry *table)
{
	const disasm_entry &e = table[oprom[0] | inst_state_base];
	uint32_t flags = e.flags | DASMFLAG_SUPPORTED;
	util::stream_format(stream, "%s", e.opcode);

	switch(e.mode) {
	case DASM_non:
		flags |= 1;
		break;

	case DASM_aba:
		util::stream_format(stream, " $%02x%02x", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_abx:
		util::stream_format(stream, " $%02x%02x, x", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_aby:
		util::stream_format(stream, " $%02x%02x, y", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_acc:
		util::stream_format(stream, " a");
		flags |= 1;
		break;

	case DASM_adr:
		util::stream_format(stream, " $%02x%02x", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_bzp:
		util::stream_format(stream, "%d $%02x", (oprom[0] >> 4) & 7, opram[1]);
		flags |= 2;
		break;

	case DASM_iax:
		util::stream_format(stream, " ($%02x%02x, x)", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_idx:
		util::stream_format(stream, " ($%02x, x)", opram[1]);
		flags |= 2;
		break;

	case DASM_idy:
		util::stream_format(stream, " ($%02x), y", opram[1]);
		flags |= 2;
		break;

	case DASM_idz:
		util::stream_format(stream, " ($%02x), z", opram[1]);
		flags |= 2;
		break;

	case DASM_imm:
		util::stream_format(stream, " #$%02x", opram[1]);
		flags |= 2;
		break;

	case DASM_imp:
		flags |= 1;
		break;

	case DASM_ind:
		util::stream_format(stream, " ($%02x%02x)", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_isy:
		util::stream_format(stream, " ($%02x, s), y", opram[1]);
		flags |= 2;
		break;

	case DASM_iw2:
		util::stream_format(stream, " #$%02x%02x", opram[2], opram[1]);
		flags |= 3;
		break;

	case DASM_iw3:
		util::stream_format(stream, " #$%02x%02x%02x", opram[3], opram[2], opram[1]);
		flags |= 4;
		break;

	case DASM_rel:
		util::stream_format(stream, " $%04x", (pc & 0xf0000) | uint16_t(pc + 2 + int8_t(opram[1])));
		flags |= 2;
		break;

	case DASM_rw2:
		util::stream_format(stream, " $%04x", (pc & 0xf0000) | uint16_t(pc + 2 + int16_t((opram[2] << 8) | opram[1])));
		flags |= 3;
		break;

	case DASM_zpb:
		util::stream_format(stream, "%d $%02x, $%04x", (oprom[0] >> 4) & 7, opram[1], (pc & 0xf0000) | uint16_t(pc + 3 + int8_t(opram[2])));
		flags |= 3;
		break;

	case DASM_zpg:
		util::stream_format(stream, " $%02x", opram[1]);
		flags |= 2;
		break;

	case DASM_zpi:
		util::stream_format(stream, " ($%02x)", opram[1]);
		flags |= 2;
		break;

	case DASM_zpx:
		util::stream_format(stream, " $%02x, x", opram[1]);
		flags |= 2;
		break;

	case DASM_zpy:
		util::stream_format(stream, " $%02x, y", opram[1]);
		flags |= 2;
		break;

	case DASM_imz:
		util::stream_format(stream, " #$%02x, $%02x", opram[1], opram[2]);
		flags |= 3;
		break;

	case DASM_spg:
		util::stream_format(stream, " \\$%02x", opram[1]);
		flags |= 2;
		break;

	case DASM_biz:
		util::stream_format(stream, " %d, $%02x", (opram[0] >> 5) & 7, opram[1]);
		flags |= 2;
		break;

	case DASM_bzr:
		util::stream_format(stream, " %d, $%02x, $%04x", (opram[0] >> 5) & 7, opram[1], (pc & 0xf0000) | uint16_t(pc + 3 + int8_t(opram[2])));
		flags |= 3;
		break;

	case DASM_bar:
		util::stream_format(stream, " %d, a, $%04x", (opram[0] >> 5) & 7, (pc & 0xf0000) | uint16_t(pc + 3 + int8_t(opram[1])));
		flags |= 2;
		break;

	case DASM_bac:
		util::stream_format(stream, " %d, a", (opram[0] >> 5) & 7);
		flags |= 1;
		break;

	default:
		fprintf(stderr, "Unhandled dasm mode %d\n", e.mode);
		abort();
	}
	return flags;
}

void m6502_device::prefetch()
{
	sync = true;
	sync_w(ASSERT_LINE);
	NPC = PC;
	IR = mintf->read_sync(PC);
	sync = false;
	sync_w(CLEAR_LINE);

	if((nmi_state || ((irq_state || apu_irq_state) && !(P & F_I))) && !inhibit_interrupts) {
		irq_taken = true;
		IR = 0x00;
	} else
		PC++;
}

void m6502_device::prefetch_noirq()
{
	sync = true;
	sync_w(ASSERT_LINE);
	NPC = PC;
	IR = mintf->read_sync(PC);
	sync = false;
	sync_w(CLEAR_LINE);
	PC++;
}

void m6502_device::set_nz(uint8_t v)
{
	P &= ~(F_Z|F_N);
	if(v & 0x80)
		P |= F_N;
	if(!v)
		P |= F_Z;
}

offs_t m6502_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	return disassemble_generic(stream, pc, oprom, opram, options, disasm_entries);
}


uint8_t m6502_device::memory_interface::read_9(uint16_t adr)
{
	return read(adr);
}

void m6502_device::memory_interface::write_9(uint16_t adr, uint8_t val)
{
	write(adr, val);
}


uint8_t m6502_device::mi_default_normal::read(uint16_t adr)
{
	return program->read_byte(adr);
}

uint8_t m6502_device::mi_default_normal::read_sync(uint16_t adr)
{
	return sdirect->read_byte(adr);
}

uint8_t m6502_device::mi_default_normal::read_arg(uint16_t adr)
{
	return direct->read_byte(adr);
}


void m6502_device::mi_default_normal::write(uint16_t adr, uint8_t val)
{
	program->write_byte(adr, val);
}

uint8_t m6502_device::mi_default_nd::read_sync(uint16_t adr)
{
	return sprogram->read_byte(adr);
}

uint8_t m6502_device::mi_default_nd::read_arg(uint16_t adr)
{
	return program->read_byte(adr);
}


#include "cpu/m6502/m6502.hxx"
